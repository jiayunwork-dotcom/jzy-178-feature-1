// Package statem 实现跨历元的两类持续性状态：
//
//  1. 故障星隔离/恢复计数（卫星级，按“系统+编号”区分）：
//     被排除的星进入隔离；此后须连续 MinEpochs 个历元“本星残差正常且
//     加回后整体检验通过”才解除。星不可见或任一条件不满足，计数清零重来。
//
//  2. 告警状态（系统级）：
//     - snapshot：一个历元超限即告警，下一个历元正常即撤警；
//     - persistence：连续 ConfirmEpochs 超限才告警，连续 ClearEpochs 正常才撤警；
//     - combined：普通超限走窗口，恶劣超限（达 GrossFactor 倍）立即告警，
//     撤警仍走窗口（保证恢复确认）。
//
// 状态机本身不做任何 GNSS 计算，输入是上层算好的每历元事件，便于单测与复用。
// 多模时间偏差在本实现中按历元自由估计（不跨历元携带），因此这里没有
// 额外的钟差状态；选型理由见 docs/design.md。
package statem

import (
	"encoding/json"

	"raim/pkg/gnss"
	"raim/pkg/profile"
)

// IsolationEntry 记录一颗隔离星的恢复计数。
type IsolationEntry struct {
	System        gnss.System `json:"-"`
	ID            int         `json:"id"`
	NormalStreak  int         `json:"normal_streak"`   // 连续正常（且加回通过）历元数
	SinceEpochSeq int         `json:"since_epoch_seq"` // 从第几个历元开始隔离
}

// AlertMachine 是告警持续状态机。
type AlertMachine struct {
	Active     bool `json:"active"`
	BadStreak  int  `json:"bad_streak"`
	GoodStreak int  `json:"good_streak"`
	// 本告警片段内是否已经拉起过告警（用于误警片段记账）
}

// State 是会话内跨历元的全部持久状态。
type State struct {
	Isolated map[gnss.SatID]*IsolationEntry `json:"-"`
	Alert    AlertMachine                   `json:"alert"`
}

// stateJSON 是 State 的落盘形态：隔离星以 "GPS:4" 文本为键，
// 旧版本裸编号键（"4"）反序列化时按 GPS 处理。
type stateJSON struct {
	Isolated map[string]*IsolationEntry `json:"isolated"`
	Alert    AlertMachine               `json:"alert"`
}

// MarshalJSON 用紧凑文本键落盘隔离集合。
func (s State) MarshalJSON() ([]byte, error) {
	v := stateJSON{Isolated: map[string]*IsolationEntry{}, Alert: s.Alert}
	for k, e := range s.Isolated {
		// 防御性：条目的系统与其键保持一致
		cp := *e
		if cp.System == "" {
			cp.System = k.System
		}
		v.Isolated[k.String()] = &cp
	}
	return json.Marshal(v)
}

// UnmarshalJSON 兼容升级前的裸编号隔离状态（按 GPS）。
func (s *State) UnmarshalJSON(b []byte) error {
	var v stateJSON
	if err := json.Unmarshal(b, &v); err != nil {
		return err
	}
	s.Alert = v.Alert
	s.Isolated = map[gnss.SatID]*IsolationEntry{}
	for key, e := range v.Isolated {
		var k gnss.SatID
		if err := k.UnmarshalText([]byte(key)); err != nil {
			return err
		}
		if e.System == "" {
			e.System = k.System
		}
		s.Isolated[k] = e
	}
	return nil
}

// NewState 创建空状态。
func NewState() State {
	return State{Isolated: map[gnss.SatID]*IsolationEntry{}}
}

// SatRecovery 是上层对某颗隔离星在本历元的恢复评估结果。
type SatRecovery struct {
	Sat    gnss.SatID
	Normal bool // 本星残差正常 且 加回后整体检验通过
}

// UpdateIsolation 推进卫星隔离状态。
//
//	excludedNow: 本历元新被排除的星（零值表示无）——进入隔离，计数清零。
//	recovery:   对当前已隔离且本历元可见的星的评估。
//	visible:    本历元可见星集合（隔离星不可见则计数清零）。
//	seq:        历元序号（仅用于记录起始）。
//
// 返回本历元仍处于隔离的星集合。
func (s *State) UpdateIsolation(prof profile.Profile, excludedNow gnss.SatID,
	recovery []SatRecovery, visible map[gnss.SatID]bool, seq int) map[gnss.SatID]bool {

	if excludedNow.ID != 0 {
		s.Isolated[excludedNow] = &IsolationEntry{
			System: excludedNow.System, ID: excludedNow.ID, SinceEpochSeq: seq,
		}
	}
	recMap := map[gnss.SatID]bool{}
	for _, r := range recovery {
		recMap[r.Sat] = r.Normal
	}
	still := map[gnss.SatID]bool{}
	for id, e := range s.Isolated {
		if id == excludedNow {
			still[id] = true
			continue
		}
		if !visible[id] {
			e.NormalStreak = 0 // 不可见，连续性中断
			still[id] = true
			continue
		}
		if recMap[id] {
			e.NormalStreak++
		} else {
			e.NormalStreak = 0
		}
		if e.NormalStreak >= prof.Isolation.MinEpochs {
			delete(s.Isolated, id) // 满足恢复条件，解除隔离
		} else {
			still[id] = true
		}
	}
	return still
}

// EpochStatus 是上层给出的本历元完好性状态。
type EpochStatus struct {
	// IntegrityAvailable 单历元 RAIM 是否可用（星数足够且定位成功）。
	IntegrityAvailable bool
	// IntegrityBad 检测失败（检出无法排除）或 HPL>HAL。
	IntegrityBad bool
	// IdentifiedRisk 本历元确实识别出风险（检出故障，含可/不可排除）。
	// 用于把告警片段区分为“真故障告警”与“误警”。
	IdentifiedRisk bool
	// Gross 恶劣超限（统计量≥GrossFactor×阈值 或 HPL≥GrossFactor×HAL）。
	Gross bool
}

// StepAlert 推进告警状态机，返回本历元告警是否激活。
func (s *State) StepAlert(prof profile.Profile, st EpochStatus) bool {
	m := &s.Alert
	switch prof.Alert.Mode {
	case profile.Snapshot:
		m.Active = st.IntegrityBad
		m.BadStreak = 0
		m.GoodStreak = 0
		if m.Active {
			m.BadStreak = 1
		} else {
			m.GoodStreak = 1
		}
	case profile.Persistence:
		m.stepWindow(prof, st, false)
	case profile.Combined:
		m.stepWindow(prof, st, st.Gross)
	}
	return m.Active
}

func (m *AlertMachine) stepWindow(prof profile.Profile, st EpochStatus, immediate bool) {
	if st.IntegrityBad {
		m.BadStreak++
		m.GoodStreak = 0
		if immediate || m.BadStreak >= prof.Alert.ConfirmEpochs {
			m.Active = true
		}
	} else {
		m.GoodStreak++
		m.BadStreak = 0
		if m.Active && m.GoodStreak >= prof.Alert.ClearEpochs {
			m.Active = false
		}
	}
}
