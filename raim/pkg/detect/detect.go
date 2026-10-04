// Package detect 实现单历元 RAIM 的故障检测（FDE）与单星排除：
//
//   - 加权残差平方和 SSE=Σ(r_i/σ_i)² 在 H0 下服从 χ²(n-p)，p=3+k
//     （3 个位置未知量 + 每套参与系统 1 个钟差未知量）；
//   - 阈值 T 由虚警概率 Pfa 确定：P(χ²(n-p)>T)=Pfa；
//   - n < p+1（无冗余）只定位、完好性不可用；
//   - 检出后逐星剔一重解重检，剔后仍有冗余且通过检验、且“只有剔这颗能解释
//     故障”才排除成功；没有任何合格试法则无法排除。
//   - 单 GPS 输入时 k=1、p=4，全部规则退化为升级前的 5 星检测 / 6 星排除。
package detect

import (
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
	ID     int         `json:"id"`
	Sys    gnss.System `json:"sys,omitempty"` // 所属系统；GPS 省略（升级前兼容）
	Resid  float64     `json:"resid"`         // 米
	Sigma  float64     `json:"sigma"`         // 米
	StdRes float64     `json:"stdres"`        // 标准化残差 r/σ
}

// Key 返回该结果对应卫星的“系统+编号”主键。
func (r SatResult) Key() gnss.SatKey {
	sys := r.Sys
	if sys == "" {
		sys = gnss.GPS
	}
	return gnss.Key(sys, r.ID)
}

// Trial 是一次剔星试解的结果。
type Trial struct {
	ExcludedID  int         `json:"excluded_id"`
	ExcludedSys gnss.System `json:"excluded_sys,omitempty"`
	SSE         float64     `json:"sse"`
	Passes      bool        `json:"passes"`
	Used        int         `json:"used"`
}

// Key 返回该试法剔除卫星的主键。
func (t Trial) Key() gnss.SatKey {
	sys := t.ExcludedSys
	if sys == "" {
		sys = gnss.GPS
	}
	return gnss.Key(sys, t.ExcludedID)
}

// Assessment 是一个历元的单历元检测结论。
type Assessment struct {
	Mode Mode `json:"mode"`

	Used         int           `json:"used"`                   // 实际参与解算的星数
	SystemsUsed  []gnss.System `json:"systems_used,omitempty"` // 实际参与解算的系统（规范次序）
	DOF          int           `json:"dof"`                    // 全解自由度 n-p（无冗余时为 0）
	SolDOF       int           `json:"sol_dof"`                // 报告位置解的自由度（排除成功后按子集）
	Threshold    float64       `json:"threshold"`              // 本历元全解阈值（Pfa 与 dof 决定）
	SolThreshold float64       `json:"sol_threshold"`          // 报告解对应门限（排除成功后按子集）
	SSE          float64       `json:"sse"`                    // 检验统计量
	SatResults   []SatResult   `json:"sat_results"`

	// 排除信息
	ExcludedID  int         `json:"excluded_id,omitempty"`
	ExcludedSys gnss.System `json:"excluded_sys,omitempty"`
	Trials      []Trial     `json:"trials,omitempty"`
	Reason      string      `json:"reason,omitempty"` // detected/unavailable 的原因

	// 解（ModeOK/Unavailable 为全解；Excluded 为剔星后的干净解；Detected 为全解）
	Sol *lsq.Solution `json:"-"`
}

// ExcludedKey 返回被排除卫星的“系统+编号”主键（未排除时为零值）。
func (a *Assessment) ExcludedKey() gnss.SatKey {
	sys := a.ExcludedSys
	if sys == "" {
		sys = gnss.GPS
	}
	return gnss.Key(sys, a.ExcludedID)
}

// Options 为检测参数。
type Options struct {
	Pfa float64 // 虚警概率
}

// Assess 对一个已通过字段校验的历元做定位、检测、（必要时）排除。
func Assess(ep *lsq.Epoch, opt Options) (*Assessment, error) {
	sol, err := lsq.Solve(ep)
	if err != nil {
		return nil, err
	}
	n := len(ep.Sats)
	a := &Assessment{Used: n, Sol: sol, SystemsUsed: append([]gnss.System(nil), sol.Systems...)}
	for i, s := range ep.Sats {
		sys := s.Sys
		if sys == "" {
			sys = gnss.GPS
		}
		a.SatResults = append(a.SatResults, SatResult{
			ID: s.ID, Sys: omitemptySys(sys),
			Resid: sol.Resid[i], Sigma: s.Sigma,
			StdRes: sol.Resid[i] / s.Sigma,
		})
	}
	sse, dof := sol.SSE()
	a.SSE, a.DOF, a.SolDOF = sse, dof, dof

	if dof < 1 {
		a.Mode = ModeUnavailable
		a.SolDOF = 0
		a.SolThreshold = 0
		if len(sol.Systems) == 1 {
			a.Reason = "少于 5 颗星，无冗余观测量，只能定位、完好性不可用"
		} else {
			a.Reason = "可用冗余不足（星数 ≤ 3+系统数），只能定位、完好性不可用"
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

	// 逐星剔一重解重检。剔除某星后子系统数可能减 1（该星是其系统唯一成员），
	// 只有子集仍有冗余（dof≥1）的试法才参与“唯一解释”判定。
	sysCount := map[gnss.System]int{}
	for _, sys := range sol.RowSys {
		sysCount[sys]++
	}
	var passing []Trial
	eligible := 0
	for idx, s := range ep.Sats {
		sys := s.Sys
		if sys == "" {
			sys = gnss.GPS
		}
		sub := removeSat(ep, idx)
		subK := len(sol.Systems)
		if sysCount[sys] == 1 {
			subK--
		}
		subDOFIfSolved := (n - 1) - (3 + subK)
		tr := Trial{ExcludedID: s.ID, ExcludedSys: omitemptySys(sys), Used: n - 1}
		if subDOFIfSolved < 1 {
			// 剔后无冗余，无法再检：该试法不能解释故障
			a.Trials = append(a.Trials, tr)
			continue
		}
		eligible++
		subSol, err := lsq.Solve(sub)
		if err != nil {
			// 剔后几何奇异：该试解视为不能解释故障
			a.Trials = append(a.Trials, tr)
			continue
		}
		subSSE, subDOF := subSol.SSE()
		thr := chisq.Threshold(subDOF, opt.Pfa)
		ok := subSSE <= thr
		tr.SSE, tr.Passes = subSSE, ok
		a.Trials = append(a.Trials, tr)
		if ok {
			passing = append(passing, tr)
			// 记录第一个通过的干净解及其自由度；唯一性通过后再用
			if len(passing) == 1 {
				a.Sol = subSol
				a.SolDOF = subDOF
				a.SolThreshold = thr
				a.SystemsUsed = append([]gnss.System(nil), subSol.Systems...)
			}
		}
	}
	switch len(passing) {
	case 1:
		a.Mode = ModeExcluded
		a.ExcludedID = passing[0].ExcludedID
		a.ExcludedSys = passing[0].ExcludedSys
	default:
		a.Mode = ModeDetected
		switch {
		case eligible == 0 && len(sol.Systems) == 1:
			a.Reason = "检出故障但少于 6 颗星，无法排除"
		case eligible == 0:
			a.Reason = "检出故障但剔除任何单星后都没有冗余再检，无法排除"
		case len(passing) == 0:
			a.Reason = "检出故障但没有任何单星剔法能通过检验（可能多星故障）"
		default:
			a.Reason = "多颗星剔后都能通过检验，无法唯一识别故障星，按检测失败处理"
		}
		// 多解不唯一时不能使用剔星解，恢复为全解
		sol2, _ := lsq.Solve(ep)
		a.Sol = sol2
		a.SolDOF = dof
		a.SolThreshold = a.Threshold
		a.SystemsUsed = append([]gnss.System(nil), sol.Systems...)
	}
	return a, nil
}

// omitemptySys 让 GPS 星在 JSON 里省略系统字段（升级前记录保持原样）。
func omitemptySys(sys gnss.System) gnss.System {
	if sys == gnss.GPS {
		return ""
	}
	return sys
}

func removeSat(ep *lsq.Epoch, idx int) *lsq.Epoch {
	sats := make([]lsq.Satellite, 0, len(ep.Sats)-1)
	sats = append(sats, ep.Sats[:idx]...)
	sats = append(sats, ep.Sats[idx+1:]...)
	return &lsq.Epoch{Approx: ep.Approx, Sats: sats}
}
