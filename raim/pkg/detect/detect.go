// Package detect 实现单历元 RAIM 的故障检测（FDE）与单星排除：
//
//   - 加权残差平方和 SSE=Σ(r_i/σ_i)² 在 H0 下服从 χ²(n-m)，
//     m=3+k（3 个位置未知量 + k 个参与系统各自的钟差未知量）；
//   - 阈值 T 由虚警概率 Pfa 确定：P(χ²(n-m)>T)=Pfa；
//   - 少于 m+1 颗星只有定位、无冗余，完好性不可用；
//   - 检出后逐星剔一重解重检，通过检验且“只有剔这颗能解释故障”才排除成功；
//   - 剔后至少剩 1 个冗余（n-1 ≥ m）才能再检，故需要 n ≥ m+2 才能排除。
//
// 卫星一律以“系统+编号”标识，跨系统同编号（GPS 3 与北斗 3）互不影响。
package detect

import (
	"encoding/json"

	"raim/pkg/chisq"
	"raim/pkg/gnss"
	"raim/pkg/lsq"
)

// Mode 是单历元检测状态。
type Mode string

const (
	ModeOK          Mode = "ok"          // 定位+检验通过
	ModeUnavailable Mode = "unavailable" // 无冗余：只定位，完好性不可用
	ModeDetected    Mode = "detected"    // 检出但无法排除（冗余不足或多星可疑）
	ModeExcluded    Mode = "excluded"    // 检出并成功排除单星
)

// SatResult 是参与解算的单星残差信息。
type SatResult struct {
	System gnss.System `json:"-"`
	ID     int         `json:"id"`
	Resid  float64     `json:"resid"`  // 米
	Sigma  float64     `json:"sigma"`  // 米
	StdRes float64     `json:"stdres"` // 标准化残差 r/σ
}

// MarshalJSON：GPS 省略 system，保持升级前单模记录形态。
func (s SatResult) MarshalJSON() ([]byte, error) {
	type alias struct {
		System string  `json:"system,omitempty"`
		ID     int     `json:"id"`
		Resid  float64 `json:"resid"`
		Sigma  float64 `json:"sigma"`
		StdRes float64 `json:"stdres"`
	}
	v := alias{ID: s.ID, Resid: s.Resid, Sigma: s.Sigma, StdRes: s.StdRes}
	if s.System != gnss.GPS && s.System != "" {
		v.System = string(s.System)
	}
	return json.Marshal(v)
}

// UnmarshalJSON：缺 system 的旧记录按 GPS。
func (s *SatResult) UnmarshalJSON(b []byte) error {
	var v struct {
		System string  `json:"system"`
		ID     int     `json:"id"`
		Resid  float64 `json:"resid"`
		Sigma  float64 `json:"sigma"`
		StdRes float64 `json:"stdres"`
	}
	if err := json.Unmarshal(b, &v); err != nil {
		return err
	}
	s.ID, s.Resid, s.Sigma, s.StdRes = v.ID, v.Resid, v.Sigma, v.StdRes
	if v.System == "" {
		s.System = gnss.GPS
	} else {
		s.System = gnss.System(v.System)
	}
	return nil
}

// Key 返回该星的跨系统唯一身份。
func (s SatResult) Key() gnss.SatID { return gnss.SatID{System: s.System, ID: s.ID} }

// Trial 是一次剔星试解的结果。
type Trial struct {
	System     gnss.System `json:"-"`
	ExcludedID int         `json:"excluded_id"`
	SSE        float64     `json:"sse"`
	Passes     bool        `json:"passes"`
	Used       int         `json:"used"`
}

// MarshalJSON：GPS 省略 system。
func (t Trial) MarshalJSON() ([]byte, error) {
	type alias struct {
		System     string  `json:"system,omitempty"`
		ExcludedID int     `json:"excluded_id"`
		SSE        float64 `json:"sse"`
		Passes     bool    `json:"passes"`
		Used       int     `json:"used"`
	}
	v := alias{ExcludedID: t.ExcludedID, SSE: t.SSE, Passes: t.Passes, Used: t.Used}
	if t.System != gnss.GPS && t.System != "" {
		v.System = string(t.System)
	}
	return json.Marshal(v)
}

// UnmarshalJSON：缺 system 的旧记录按 GPS。
func (t *Trial) UnmarshalJSON(b []byte) error {
	var v struct {
		System     string  `json:"system"`
		ExcludedID int     `json:"excluded_id"`
		SSE        float64 `json:"sse"`
		Passes     bool    `json:"passes"`
		Used       int     `json:"used"`
	}
	if err := json.Unmarshal(b, &v); err != nil {
		return err
	}
	t.ExcludedID, t.SSE, t.Passes, t.Used = v.ExcludedID, v.SSE, v.Passes, v.Used
	if v.System == "" {
		t.System = gnss.GPS
	} else {
		t.System = gnss.System(v.System)
	}
	return nil
}

// ExcludedKey 返回试解剔掉的星的跨系统身份。
func (t Trial) ExcludedKey() gnss.SatID { return gnss.SatID{System: t.System, ID: t.ExcludedID} }

// Assessment 是一个历元的单历元检测结论。
type Assessment struct {
	Mode Mode `json:"mode"`

	Used         int           `json:"used"`          // 实际参与解算的星数
	Systems      []gnss.System `json:"-"`             // 参与解算的系统（时钟列顺序）
	DOF          int           `json:"dof"`           // 全解自由度 n-m（无冗余时 0）
	SolDOF       int           `json:"sol_dof"`       // 报告位置解的自由度（排除成功后为 n-1-m）
	Threshold    float64       `json:"threshold"`     // 本历元全解阈值（Pfa 与 dof 决定）
	SolThreshold float64       `json:"sol_threshold"` // 报告解对应门限
	SSE          float64       `json:"sse"`           // 检验统计量
	SatResults   []SatResult   `json:"sat_results"`

	// 排除信息（系统+编号）
	ExcludedSystem gnss.System `json:"-"`
	ExcludedID     int         `json:"excluded_id,omitempty"`
	Trials         []Trial     `json:"trials,omitempty"`
	Reason         string      `json:"reason,omitempty"` // detected/unavailable 的原因

	// 解（ModeOK/Unavailable 为全解；Excluded 为剔星后的干净解；Detected 为全解）
	Sol *lsq.Solution `json:"-"`
}

// ExcludedKey 返回被排除星的跨系统身份（无排除时零值）。
func (a *Assessment) ExcludedKey() gnss.SatID {
	return gnss.SatID{System: a.ExcludedSystem, ID: a.ExcludedID}
}

// Options 为检测参数。
type Options struct {
	Pfa float64 // 虚警概率
}

// unknowns 返回本历元未知量个数 m=3+k。
func unknowns(ep *lsq.Epoch) int {
	present := map[gnss.System]bool{}
	for _, s := range ep.Sats {
		present[s.System] = true
	}
	return 3 + len(present)
}

// Assess 对一个已通过字段校验的历元做定位、检测、（必要时）排除。
func Assess(ep *lsq.Epoch, opt Options) (*Assessment, error) {
	sol, err := lsq.Solve(ep)
	if err != nil {
		return nil, err
	}
	n := len(ep.Sats)
	m := unknowns(ep)
	a := &Assessment{Used: n, Systems: sol.Systems, Sol: sol}
	for i, s := range ep.Sats {
		a.SatResults = append(a.SatResults, SatResult{
			System: s.System, ID: s.ID,
			Resid: sol.Resid[i], Sigma: s.Sigma,
			StdRes: sol.Resid[i] / s.Sigma,
		})
	}
	sse, dof := lsq.WeightedSSEDOF(sol.Resid, sol.Sigma, m)
	a.SSE, a.DOF, a.SolDOF = sse, dof, dof

	if dof < 1 {
		// n < m+1：无冗余，只能定位
		a.Mode = ModeUnavailable
		a.SolDOF = 0
		a.SolThreshold = 0
		if len(a.Systems) <= 1 {
			a.Reason = "少于 5 颗星，无冗余观测量，只能定位、完好性不可用"
		} else {
			a.Reason = "冗余不足（星数不超过未知量数 3+系统数），只能定位、完好性不可用"
		}
		a.Threshold = 0
		return a, nil
	}
	a.Threshold = chisq.Threshold(dof, opt.Pfa)
	a.SolThreshold = a.Threshold
	if sse <= a.Threshold {
		a.Mode = ModeOK
		return a, nil
	}

	// 检出故障：剔后须仍有至少 1 个冗余（n-1 ≥ m）才能再检，即 n ≥ m+2。
	if n < m+2 {
		a.Mode = ModeDetected
		if len(a.Systems) <= 1 {
			a.Reason = "检出故障但少于 6 颗星，无法排除"
		} else {
			a.Reason = "检出故障但剔星后无冗余再检，无法排除"
		}
		return a, nil
	}

	// 逐星剔一重解重检
	var passing []Trial
	for idx, s := range ep.Sats {
		sub := removeSat(ep, idx)
		subSol, err := lsq.Solve(sub)
		if err != nil {
			// 剔后几何奇异：该试解视为不能解释故障
			a.Trials = append(a.Trials, Trial{System: s.System, ExcludedID: s.ID, Used: n - 1, Passes: false})
			continue
		}
		subM := 3 + len(subSol.Systems)
		subSSE, subDOF := lsq.WeightedSSEDOF(subSol.Resid, subSol.Sigma, subM)
		thr := 0.0
		ok := false
		if subDOF >= 1 {
			thr = chisq.Threshold(subDOF, opt.Pfa)
			ok = subSSE <= thr
		}
		tr := Trial{System: s.System, ExcludedID: s.ID, SSE: subSSE, Passes: ok, Used: n - 1}
		a.Trials = append(a.Trials, tr)
		if ok {
			passing = append(passing, tr)
			// 记录第一个通过的干净解及其自由度；唯一性通过后再用
			if len(passing) == 1 {
				a.Sol = subSol
				a.SolDOF = subDOF
				a.SolThreshold = thr
			}
		}
	}
	switch len(passing) {
	case 1:
		a.Mode = ModeExcluded
		a.ExcludedSystem = passing[0].System
		a.ExcludedID = passing[0].ExcludedID
	default:
		a.Mode = ModeDetected
		if len(passing) == 0 {
			a.Reason = "检出故障但没有任何单星剔法能通过检验（可能多星故障）"
		} else {
			a.Reason = "多颗星剔后都能通过检验，无法唯一识别故障星，按检测失败处理"
		}
		// 多解不唯一时不能使用剔星解，恢复为全解
		sol2, _ := lsq.Solve(ep)
		a.Sol = sol2
		a.SolDOF = dof
		a.SolThreshold = a.Threshold
	}
	return a, nil
}

func removeSat(ep *lsq.Epoch, idx int) *lsq.Epoch {
	sats := make([]lsq.Satellite, 0, len(ep.Sats)-1)
	sats = append(sats, ep.Sats[:idx]...)
	sats = append(sats, ep.Sats[idx+1:]...)
	return &lsq.Epoch{Approx: ep.Approx, Sats: sats}
}
