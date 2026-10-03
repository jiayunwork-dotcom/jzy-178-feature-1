// Package lsq 实现加权单点最小二乘定位及定位后的几何量：
// 标准化残差（残差投影 S）、DOP、RAIM 斜率。
//
// 观测模型（多模接收机，线性化于估计位置）。同一历元内来自不同 GNSS 的
// 伪距一起参与解算；接收机对每套系统时看到的钟差不同，因此模型除三维位置外，
// 为每个参与系统各设一个钟差列（GPS 列作为基准，恒为 1，其系数即接收机钟差，
// 非 GPS 系统列的系数是该系统时相对 GPS 时的系统间偏差）：
//
//	y = G·dx + ε，G 每行 = [-ux -uy -uz | 每系统一列钟差]
//	W = diag(1/σ_i²)
//	dx = (GᵀWG)^{-1} GᵀW y
//	加权残差：r = y - G·dx
//	标准化残差投影：S = I - G (GᵀWG)^{-1} GᵀW（S·y 即加权残差）
//
// 只含 GPS 时只有一个钟差列，模型与升级前完全一致（逐位相同的法方程与消元路径）。
package lsq

import (
	"encoding/json"
	"math"

	"raim/pkg/apierr"
	"raim/pkg/geo"
	"raim/pkg/gnss"
)

// maxIter 是定位迭代的硬上限（需求：最多 10 轮）。
const maxIter = 10

// Satellite 是一颗参与解算的卫星：所属系统、ECEF 坐标、伪距（米）与
// 伪距误差标准差（米，用于定权）。
type Satellite struct {
	System gnss.System
	ID     int     `json:"id"`
	Pos    geo.Vec `json:"pos"`
	PR     float64 `json:"pr"`
	Sigma  float64 `json:"sigma"`
}

// satelliteJSON 是 Satellite 的落盘形态：GPS 省略 system（旧格式），
// 缺 system 的旧数据按 GPS 反序列化。
type satelliteJSON struct {
	System string  `json:"system,omitempty"`
	ID     int     `json:"id"`
	Pos    geo.Vec `json:"pos"`
	PR     float64 `json:"pr"`
	Sigma  float64 `json:"sigma"`
}

// MarshalJSON：GPS 不写 system，保持与升级前记录逐字段一致。
func (s Satellite) MarshalJSON() ([]byte, error) {
	v := satelliteJSON{ID: s.ID, Pos: s.Pos, PR: s.PR, Sigma: s.Sigma}
	if s.System != gnss.GPS && s.System != "" {
		v.System = string(s.System)
	}
	return json.Marshal(v)
}

// UnmarshalJSON：没有 system 的旧星按 GPS。
func (s *Satellite) UnmarshalJSON(b []byte) error {
	var v satelliteJSON
	if err := json.Unmarshal(b, &v); err != nil {
		return err
	}
	s.ID, s.Pos, s.PR, s.Sigma = v.ID, v.Pos, v.PR, v.Sigma
	if v.System == "" {
		s.System = gnss.GPS
	} else {
		s.System = gnss.System(v.System)
	}
	return nil
}

// Epoch 是一个历元的定位输入。Approx 为概略位置初值（可为零值，此时取 0,0,0 附近不可用）。
type Epoch struct {
	Approx geo.Vec
	Sats   []Satellite
}

// Solution 是定位结果。
type Solution struct {
	Pos       geo.Vec   // ECEF 位置
	LLA       geo.LLA   // 经纬度高
	ClockBias float64   // 接收机钟差（GPS 时基准，米）
	Iter      int       // 实际迭代轮数
	Converged bool      // 是否在 10 轮内满足 1mm 收敛
	Resid     []float64 // 每颗星残差（观测伪距 - 预测伪距，含该系统钟差），米
	// G 为最终几何矩阵，W 为权重对角，S 为残差投影；供检测/保护级使用。
	G [][]float64
	W []float64
	S [][]float64
	// DOP（加权尺度）
	HDOP, VDOP, TDOP, GDOP float64
	// Sigma 为各星误差标准差，顺序与输入一致。
	Sigma []float64

	// 多模时间基准：参与系统（顺序即时钟列顺序，GPS 恒在第一列）、
	// 基准系统（GPS）、各系统钟差估计（相对 GPS 时，米；GPS 项=接收机钟差）。
	Systems    []gnss.System
	RefSystem  gnss.System
	ClockBySys map[gnss.System]float64
	// SatIDs 为参与本解的星（系统+编号），顺序与 Resid/Sigma/G 行一致。
	SatIDs []gnss.SatID
}

func badFloat(x float64) bool { return math.IsNaN(x) || math.IsInf(x, 0) }

// Validate 按需求逐字段校验输入，错误指到具体字段。
func (e *Epoch) Validate() error {
	seen := map[gnss.SatID]bool{}
	sysSeen := map[gnss.System]bool{}
	for i, s := range e.Sats {
		field := func(name string) string {
			return "satellites[" + itoa(i) + "]." + name
		}
		if s.System != "" && !s.System.Valid() {
			return apierr.New(field("system"),
				"不认识的系统标识 "+string(s.System)+"（只支持 GPS/GAL/BDS）")
		}
		if s.ID <= 0 {
			return apierr.New(field("id"), "卫星编号必须为正整数")
		}
		k := gnss.SatID{System: s.System, ID: s.ID}
		if seen[k] {
			return apierr.New(field("id"),
				"卫星编号在同一系统内重复："+string(s.System)+" "+itoa(s.ID))
		}
		seen[k] = true
		sysSeen[s.System] = true
		if badFloat(s.Pos.X) || badFloat(s.Pos.Y) || badFloat(s.Pos.Z) {
			return apierr.New(field("pos"), "卫星坐标含 NaN 或无穷")
		}
		if badFloat(s.PR) {
			return apierr.New(field("pr"), "伪距含 NaN 或无穷")
		}
		if badFloat(s.Sigma) || s.Sigma <= 0 {
			return apierr.New(field("sigma"), "伪距误差标准差必须为有限正数")
		}
	}
	// 位置未知量 3 个，每套系统一个钟差未知量：至少 3+k 颗星才能定位。
	if need := 3 + len(sysSeen); len(e.Sats) < need {
		return apierr.New("satellites",
			"可见星不足 "+itoa(need)+" 颗（3 个位置未知量 + 每个系统 1 个钟差），无法定位")
	}
	if badFloat(e.Approx.X) || badFloat(e.Approx.Y) || badFloat(e.Approx.Z) {
		return apierr.New("approx", "概略位置含 NaN 或无穷")
	}
	// “几乎贴着地心”：ECEF 模长小于地球极半径的 1%（约 64 km）。
	if r := geo.Norm(e.Approx); r < 0.01*geo.B {
		return apierr.New("approx", "概略位置几乎贴着地心，请给出合理的地面初值")
	}
	return nil
}

// Solve 执行加权最小二乘迭代定位。
func Solve(e *Epoch) (*Solution, error) {
	// 缺省兼容：没有系统标识（包内直接构造）的星按 GPS。
	for i := range e.Sats {
		if e.Sats[i].System == "" {
			e.Sats[i].System = gnss.GPS
		}
	}
	if err := e.Validate(); err != nil {
		return nil, err
	}
	n := len(e.Sats)

	systems := orderedSystems(e.Sats)
	ref := systems[0] // GPS 参与时恒为 GPS
	colOf := map[gnss.System]int{}
	for j, sys := range systems {
		colOf[sys] = j
	}
	m := 3 + len(systems)

	pos := e.Approx
	clock := make([]float64, len(systems))
	iter := 0
	converged := false

	var G [][]float64
	var y, wdiag, resid []float64

	for iter = 1; iter <= maxIter; iter++ {
		G = make([][]float64, n)
		y = make([]float64, n)
		wdiag = make([]float64, n)
		for i, s := range e.Sats {
			d := geo.Sub(s.Pos, pos)
			r := geo.Norm(d)
			ux, uy, uz := d.X/r, d.Y/r, d.Z/r
			row := make([]float64, m)
			row[0], row[1], row[2] = -ux, -uy, -uz
			row[3+colOf[s.System]] = 1
			G[i] = row
			y[i] = s.PR - (r + clock[colOf[s.System]])
			wdiag[i] = 1 / (s.Sigma * s.Sigma)
		}
		dx, err := solveNormal(G, y, wdiag)
		if err != nil {
			return nil, err
		}
		pos = geo.Add(pos, geo.Vec{X: dx[0], Y: dx[1], Z: dx[2]})
		for j := range clock {
			clock[j] += dx[3+j]
		}
		if math.Sqrt(dx[0]*dx[0]+dx[1]*dx[1]+dx[2]*dx[2]) < 0.001 {
			converged = true
			break
		}
	}
	if iter > maxIter {
		iter = maxIter // 跑满 10 轮仍未收敛：实际轮数记 10，Converged=false
	}

	// 最终残差
	resid = make([]float64, n)
	for i, s := range e.Sats {
		r := geo.Norm(geo.Sub(s.Pos, pos))
		resid[i] = s.PR - (r + clock[colOf[s.System]])
	}
	S := projection(G, wdiag)

	clockBySys := make(map[gnss.System]float64, len(systems))
	for j, sys := range systems {
		clockBySys[sys] = clock[j]
	}
	satIDs := make([]gnss.SatID, n)
	for i, s := range e.Sats {
		satIDs[i] = gnss.SatID{System: s.System, ID: s.ID}
	}
	sol := &Solution{
		Pos:        pos,
		LLA:        geo.ECEFToLLA(pos),
		ClockBias:  clock[colOf[ref]],
		Iter:       iter,
		Converged:  converged,
		Resid:      resid,
		G:          G,
		W:          wdiag,
		S:          S,
		Sigma:      make([]float64, n),
		Systems:    systems,
		RefSystem:  ref,
		ClockBySys: clockBySys,
		SatIDs:     satIDs,
	}
	for i, s := range e.Sats {
		sol.Sigma[i] = s.Sigma
	}
	fillDOP(sol)
	return sol, nil
}

// orderedSystems 按规范顺序返回本历元实际参与的系统（GPS 恒排第一列）。
func orderedSystems(sats []Satellite) []gnss.System {
	present := map[gnss.System]bool{}
	for _, s := range sats {
		present[s.System] = true
	}
	var out []gnss.System
	for _, sys := range gnss.Systems() {
		if present[sys] {
			out = append(out, sys)
		}
	}
	return out
}

// solveNormal 解 (GᵀWG) dx = GᵀWy，并对奇异几何报错。
// 消元路径（列归一化、部分选主元、相同的阈值）与升级前 m=4 实现逐位一致。
func solveNormal(G [][]float64, y, w []float64) ([]float64, error) {
	n := len(G)
	m := len(G[0])
	N := make([][]float64, m)
	u := make([]float64, m)
	for i := range N {
		N[i] = make([]float64, m)
	}
	for k := 0; k < n; k++ {
		for i := 0; i < m; i++ {
			u[i] += w[k] * G[k][i] * y[k]
			for j := 0; j < m; j++ {
				N[i][j] += w[k] * G[k][i] * G[k][j]
			}
		}
	}
	// 先对列做归一化，再按相对主元判奇异（卫星共面等情形）。
	scale := make([]float64, m)
	for i := 0; i < m; i++ {
		scale[i] = math.Sqrt(math.Abs(N[i][i]))
		if scale[i] == 0 {
			scale[i] = 1
		}
	}
	for i := 0; i < m; i++ {
		for j := 0; j < m; j++ {
			N[i][j] /= scale[i] * scale[j]
		}
		u[i] /= scale[i]
	}
	for col := 0; col < m; col++ {
		// 部分选主元
		piv := col
		for r := col + 1; r < m; r++ {
			if math.Abs(N[r][col]) > math.Abs(N[piv][col]) {
				piv = r
			}
		}
		if math.Abs(N[piv][col]) < 1e-9 {
			return nil, apierr.New("satellites", "几何矩阵奇异（卫星可能共面或几何退化），无法定位")
		}
		if piv != col {
			N[piv], N[col] = N[col], N[piv]
			u[piv], u[col] = u[col], u[piv]
		}
		for r := 0; r < m; r++ {
			if r == col {
				continue
			}
			f := N[r][col] / N[col][col]
			for j := col; j < m; j++ {
				N[r][j] -= f * N[col][j]
			}
			u[r] -= f * u[col]
		}
	}
	x := make([]float64, m)
	for i := 0; i < m; i++ {
		x[i] = u[i] / N[i][i] / scale[i]
	}
	return x, nil
}

// normalForSol 组装解的法方程矩阵 GᵀWG（维度 m=3+系统数）。
func normalForSol(s *Solution) [][]float64 {
	m := len(s.G[0])
	nRows := len(s.G)
	N := make([][]float64, m)
	for i := range N {
		N[i] = make([]float64, m)
	}
	for k := 0; k < nRows; k++ {
		for i := 0; i < m; i++ {
			for j := 0; j < m; j++ {
				N[i][j] += s.W[k] * s.G[k][i] * s.G[k][j]
			}
		}
	}
	return N
}

// CovPos 返回 (GᵀWG)^{-1} 的左上角 3x3（位置协方差，单位按 sigma 尺度）。
func (s *Solution) CovPos() [3][3]float64 {
	inv := invertMatrix(normalForSol(s))
	var cov [3][3]float64
	for i := 0; i < 3; i++ {
		for j := 0; j < 3; j++ {
			cov[i][j] = inv[i][j]
		}
	}
	return cov
}

// invertMatrix 高斯-若尔当求方阵逆（调用处已保证非奇异）。
// m=4 时与升级前的 invert4 走完全相同的运算序列，结果逐位相同。
func invertMatrix(N [][]float64) [][]float64 {
	m := len(N)
	a := make([][]float64, m)
	for i := range a {
		a[i] = make([]float64, 2*m)
		copy(a[i], N[i])
		a[i][m+i] = 1
	}
	for col := 0; col < m; col++ {
		piv := col
		for r := col + 1; r < m; r++ {
			if math.Abs(a[r][col]) > math.Abs(a[piv][col]) {
				piv = r
			}
		}
		a[piv], a[col] = a[col], a[piv]
		d := a[col][col]
		for j := 0; j < 2*m; j++ {
			a[col][j] /= d
		}
		for r := 0; r < m; r++ {
			if r == col {
				continue
			}
			f := a[r][col]
			for j := 0; j < 2*m; j++ {
				a[r][j] -= f * a[col][j]
			}
		}
	}
	out := make([][]float64, m)
	for i := 0; i < m; i++ {
		out[i] = make([]float64, m)
		for j := 0; j < m; j++ {
			out[i][j] = a[i][m+j]
		}
	}
	return out
}

// invert4 保留旧签名（4x4），内部走通用实现；GPS 单系统结果与升级前逐位一致。
func invert4(N [][]float64) [4][4]float64 {
	inv := invertMatrix(N)
	var out [4][4]float64
	for i := 0; i < 4; i++ {
		for j := 0; j < 4; j++ {
			out[i][j] = inv[i][j]
		}
	}
	return out
}

// projection 计算 S = I - G (GᵀWG)^{-1} GᵀW。
func projection(G [][]float64, w []float64) [][]float64 {
	n := len(G)
	m := len(G[0])
	inv := invertMatrix(normal(G, w, m))
	S := make([][]float64, n)
	for i := 0; i < n; i++ {
		S[i] = make([]float64, n)
		for j := 0; j < n; j++ {
			v := 0.0
			for a := 0; a < m; a++ {
				for b := 0; b < m; b++ {
					v += G[i][a] * inv[a][b] * G[j][b] * w[j]
				}
			}
			if i == j {
				v = 1 - v
			} else {
				v = -v
			}
			S[i][j] = v
		}
	}
	return S
}

func normal(G [][]float64, w []float64, m int) [][]float64 {
	n := len(G)
	N := make([][]float64, m)
	for i := range N {
		N[i] = make([]float64, m)
	}
	for k := 0; k < n; k++ {
		for i := 0; i < m; i++ {
			for j := 0; j < m; j++ {
				N[i][j] += w[k] * G[k][i] * G[k][j]
			}
		}
	}
	return N
}

// fillDOP 用 ENU 旋转把位置协方差换算到水平/垂直 DOP。
func fillDOP(sol *Solution) {
	cov := sol.CovPos()
	rot := geo.ENURotation(sol.LLA.Lon, sol.LLA.Lat)
	// D_ENU = R D_ECEF Rᵀ
	var c [3][3]float64
	for i := 0; i < 3; i++ {
		for j := 0; j < 3; j++ {
			v := 0.0
			for a := 0; a < 3; a++ {
				for b := 0; b < 3; b++ {
					v += rot[i][a] * cov[a][b] * rot[j][b]
				}
			}
			c[i][j] = v
		}
	}
	sol.HDOP = math.Sqrt(math.Max(0, c[0][0]+c[1][1]))
	sol.VDOP = math.Sqrt(math.Max(0, c[2][2]))
	// 基准钟差（GPS 列，恒为第 4 列）精度 = (GᵀWG)^{-1}_{33}
	inv := invertMatrix(normalForSol(sol))
	sol.TDOP = math.Sqrt(math.Max(0, inv[3][3]))
	sol.GDOP = math.Sqrt(math.Max(0, c[0][0]+c[1][1]+c[2][2]+inv[3][3]))
}

// WeightedSSEDOF 返回加权残差平方和 Σ (r_i/σ_i)² 与自由度 n-m
// （m = 3 位置未知量 + 参与系统数）。
func WeightedSSEDOF(resid, sigma []float64, unknowns int) (float64, int) {
	sse := 0.0
	for i := range resid {
		z := resid[i] / sigma[i]
		sse += z * z
	}
	return sse, len(resid) - unknowns
}

// WeightedSSE 是 GPS 单系统（4 个未知量）的旧签名包装，供既有调用使用。
func WeightedSSE(resid, sigma []float64) (float64, int) {
	return WeightedSSEDOF(resid, sigma, 4)
}

// Slopes 返回每颗星的 RAIM 斜率（按输入顺序，单位 m/m）。
//
// 采用 Brown (1992) 的经典定义：第 i 颗星存在 1 m 伪距偏差 b 时，
// 定位解的水平位移与该星检验统计量增量之比。多模下未知量含各系统钟差列，
// 投影与增益均基于 m 维（m=3+k）法方程逆矩阵。
//
//	令 K = (GᵀWG)^{-1}，A = K GᵀW（最小二乘增益矩阵）。
//	b 作用下位置水平位移 = ‖(A 的前两行)[:,i]‖·|b|；
//	该星加权残差的改变量 = S_ii·b，检验统计量 √SSE 的增量
//	（小偏差近似）正比于 S_ii/(σ_i·√S_ii) = √S_ii/σ_i。
//
// 故 slope_i = ‖A_xy[:,i]‖·σ_i/√S_ii，取最大者计算 HPL。
func (s *Solution) Slopes() []float64 {
	n := len(s.G)
	m := len(s.G[0])
	inv := invertMatrix(normalForSol(s))
	rot := geo.ENURotation(s.LLA.Lon, s.LLA.Lat)

	slopes := make([]float64, n)
	for i := 0; i < n; i++ {
		// A 的前 3 行第 i 列（ECEF 位置对第 i 个观测偏差的增益）
		var dEcef [3]float64
		for a := 0; a < 3; a++ {
			v := 0.0
			for b := 0; b < m; b++ {
				v += inv[a][b] * s.G[i][b]
			}
			dEcef[a] = v * s.W[i]
		}
		east := rot[0][0]*dEcef[0] + rot[0][1]*dEcef[1] + rot[0][2]*dEcef[2]
		north := rot[1][0]*dEcef[0] + rot[1][1]*dEcef[1] + rot[1][2]*dEcef[2]
		hShift := math.Sqrt(east*east + north*north)
		sii := math.Sqrt(math.Max(1e-30, s.S[i][i]))
		slopes[i] = hShift * s.Sigma[i] / sii
	}
	return slopes
}

func itoa(i int) string {
	if i == 0 {
		return "0"
	}
	var b [20]byte
	pos := len(b)
	for i > 0 {
		pos--
		b[pos] = byte('0' + i%10)
		i /= 10
	}
	return string(b[pos:])
}
