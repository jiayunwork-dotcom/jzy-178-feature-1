// Package session 实现回放会话：绑定运行档、逐历元（或批量）提交，
// 编排“单历元多模定位/检测/排除/保护级”与“跨历元隔离、告警状态机”，
// 并做时间戳去重/倒退校验与统计。
//
// 多模要点：
//   - 同一历元的 GPS/GAL/BDS 伪距一起进入定位、检测、排除与保护级；
//   - 每套参与系统各有一个钟差未知量（ISB 由钟差列之差体现），按每历元
//     自由未知量估计，不跨历元携带（选型理由见 docs/design.md）；
//   - 卫星一律按“系统+编号”区分：GPS 3 号与北斗 3 号是两颗不同的星；
//   - 无系统标识的星一律按 GPS（升级前会话/运行档直接接着提交）。
//
// 同一段数据无论一次性批量提交还是逐历元实时提交，都走同一个 Step，
// 因此逐历元结果严格一致；状态持久化后重启续跑同样复用该 Step。
package session

import (
	"fmt"
	"math"
	"sort"
	"time"

	"raim/pkg/apierr"
	"raim/pkg/chisq"
	"raim/pkg/detect"
	"raim/pkg/geo"
	"raim/pkg/gnss"
	"raim/pkg/lsq"
	"raim/pkg/profile"
	"raim/pkg/protect"
	"raim/pkg/statem"
)

// sortedIsolatedKeys 返回当前隔离且本历元可见的卫星主键（系统次序 GPS<GAL<BDS，再按编号）。
func sortedIsolatedKeys(isolated map[gnss.SatKey]*statem.IsolationEntry,
	visible map[gnss.SatKey]bool) []gnss.SatKey {
	var keys []gnss.SatKey
	for k := range isolated {
		if visible[k] {
			keys = append(keys, k)
		}
	}
	sort.Slice(keys, func(i, j int) bool { return gnss.Less(keys[i], keys[j]) })
	return keys
}

// SatInput 是调用方提交的单星观测。Sys 为所属系统标识，省略按 GPS。
type SatInput struct {
	ID    int        `json:"id"`
	Sys   string     `json:"sys,omitempty"` // GPS/GAL/BDS，大小写不敏感；空=GPS
	Pos   [3]float64 `json:"pos"`           // ECEF x,y,z
	PR    float64    `json:"pr"`
	Sigma float64    `json:"sigma"`
}

// EpochInput 是一个历元的提交内容。Approx 为可选概略位置（首历元建议提供）。
type EpochInput struct {
	Timestamp int64       `json:"timestamp"` // 单调时间戳（任意单位，如 Unix 秒）
	Approx    *[3]float64 `json:"approx,omitempty"`
	Sats      []SatInput  `json:"satellites"`
}

// SatRef 用“系统+编号”引用一颗卫星；GPS 星省略系统字段（升级前兼容）。
type SatRef struct {
	Sys gnss.System `json:"sys,omitempty"`
	ID  int         `json:"id"`
}

func refOf(k gnss.SatKey) SatRef {
	sys := k.Sys
	if sys == gnss.GPS {
		sys = ""
	}
	return SatRef{Sys: sys, ID: k.ID}
}

// SysClockResult 给出一套系统在某历元的时间偏差估计与参与情况。
type SysClockResult struct {
	Sys          gnss.System `json:"sys"`
	Participated bool        `json:"participated"`                // 本历元是否实际参与报告的定位解
	Visible      int         `json:"visible"`                     // 本历元可见星数
	Used         int         `json:"used"`                        // 实际参与解算的星数
	ClockBias    float64     `json:"clock_bias,omitempty"`        // 该系统接收机钟差（米）；未参与省略
	ISB          float64     `json:"inter_system_bias,omitempty"` // 相对参考系统（优先 GPS）的时间偏差（米）
}

// EpochRecord 是一个历元的完整判定快照（逐历元结果）。
type EpochRecord struct {
	Seq       int   `json:"seq"`
	Timestamp int64 `json:"timestamp"`

	// 所用档与当时阈值（每个历元都注明）
	ProfileName string  `json:"profile_name"`
	Pfa         float64 `json:"pfa"`
	Pmd         float64 `json:"pmd"`
	HAL         float64 `json:"hal"`
	Threshold   float64 `json:"chi_square_threshold"`
	DOF         int     `json:"dof"`

	// 单历元结果
	Mode         string             `json:"mode"` // ok/unavailable/detected/excluded
	SSE          float64            `json:"sse"`
	HPL          float64            `json:"hpl"`
	ExcludedID   int                `json:"excluded_id,omitempty"`
	ExcludedSys  gnss.System        `json:"excluded_sys,omitempty"`
	Isolated     []int              `json:"isolated"`                // 本历元处于隔离（不参与解算）的星编号
	IsolatedSats []SatRef           `json:"isolated_sats,omitempty"` // 多模历元：带系统的隔离星主键
	Systems      []SysClockResult   `json:"systems,omitempty"`       // 多模历元：各系统钟差与参与情况
	Position     Position           `json:"position"`
	ClockBias    float64            `json:"clock_bias"`
	Iterations   int                `json:"iterations"`
	Converged    bool               `json:"converged"`
	SatResults   []detect.SatResult `json:"sat_results"`
	Trials       []detect.Trial     `json:"trials,omitempty"`

	// 跨历元状态
	Alert      bool `json:"alert"`
	BadStreak  int  `json:"bad_streak"`
	GoodStreak int  `json:"good_streak"`
	RAIMAvail  bool `json:"raim_available"`

	Reason string `json:"reason,omitempty"`
}

// Position 同时给出 ECEF 与经纬度高。
type Position struct {
	ECEF [3]float64 `json:"ecef"`
	Lon  float64    `json:"lon"`
	Lat  float64    `json:"lat"`
	Alt  float64    `json:"alt"`
}

// Stats 是会话级统计。
type Stats struct {
	Epochs        int     `json:"epochs"`
	RAIMAvailable int     `json:"raim_available_epochs"`
	Alerts        int     `json:"alert_epochs"`
	Detections    int     `json:"detections"`           // 检出故障的历元数
	AlertEpisodes int     `json:"alert_episodes"`       // 告警片段数
	FalseAlarms   int     `json:"false_alarm_episodes"` // 未识别出故障星的告警片段数
	RAIMAvailRate float64 `json:"raim_availability"`    // RAIM 可用历元占比
	ServiceAvail  float64 `json:"service_availability"` // 未告警历元占比
}

// Session 是一个回放会话的全部持久内容。
type Session struct {
	ID          string         `json:"id"`
	ProfileName string         `json:"profile_name"`
	CreatedAt   time.Time      `json:"created_at"`
	LastTS      *int64         `json:"last_ts"`
	State       statem.State   `json:"state"`
	Stats       Stats          `json:"stats"`
	Records     []*EpochRecord `json:"records"`

	// 当前告警片段内是否已识别出真实风险（误警记账用，需持久化）
	AlertOpenRisk bool `json:"alert_open_risk"`
}

// Service 在无存储依赖的情况下驱动会话计算（Store 负责持久化）。
type Service struct {
	profiles map[string]profile.Profile
}

// NewService 用给定运行档集合构造服务。
func NewService(ps []profile.Profile) *Service {
	m := map[string]profile.Profile{}
	for _, p := range ps {
		m[p.Name] = p
	}
	return &Service{profiles: m}
}

// Profile 返回已注册运行档。
func (svc *Service) Profile(name string) (profile.Profile, bool) {
	p, ok := svc.profiles[name]
	return p, ok
}

// NewSession 创建绑定运行档的会话。
func (svc *Service) NewSession(id, profileName string) (*Session, error) {
	if _, ok := svc.profiles[profileName]; !ok {
		return nil, apierr.ErrProfileNotFound
	}
	return &Session{
		ID:          id,
		ProfileName: profileName,
		CreatedAt:   time.Now().UTC(),
		State:       statem.NewState(),
	}, nil
}

// validateEpochInput 在解算前做调用方输入校验（错误指到具体字段）。
// 更深一层的几何/近地心等校验仍由 lsq.Validate 负责。
func validateEpochInput(in EpochInput) error {
	if len(in.Sats) < 4 {
		return apierr.New("satellites",
			fmt.Sprintf("历元 %d 可见星 %d 颗，不足 4 颗", in.Timestamp, len(in.Sats)))
	}
	seen := map[gnss.SatKey]bool{}
	for i := range in.Sats {
		s := &in.Sats[i]
		field := func(name string) string {
			return "satellites[" + itoa(i) + "]." + name
		}
		sys, ok := gnss.ParseSystem(s.Sys)
		if !ok {
			return apierr.New(field("sys"),
				fmt.Sprintf("未知卫星系统标识 %q（仅支持 GPS/GAL/BDS，缺省按 GPS）", s.Sys))
		}
		if s.ID <= 0 {
			return apierr.New(field("id"), "卫星编号必须为正整数")
		}
		key := gnss.Key(sys, s.ID)
		if seen[key] {
			return apierr.New(field("id"),
				"同一系统内卫星编号出现两次（跨系统同号是不同的星，允许重号）")
		}
		seen[key] = true
		if badFloat(s.Pos[0]) || badFloat(s.Pos[1]) || badFloat(s.Pos[2]) {
			return apierr.New(field("pos"), "卫星坐标含 NaN 或无穷")
		}
		if badFloat(s.PR) {
			return apierr.New(field("pr"), "伪距含 NaN 或无穷")
		}
		if badFloat(s.Sigma) || s.Sigma <= 0 {
			return apierr.New(field("sigma"), "伪距误差标准差必须为有限正数")
		}
	}
	if in.Approx != nil {
		if badFloat((*in.Approx)[0]) || badFloat((*in.Approx)[1]) || badFloat((*in.Approx)[2]) {
			return apierr.New("approx", "概略位置含 NaN 或无穷")
		}
	}
	return nil
}

func badFloat(x float64) bool { return math.IsNaN(x) || math.IsInf(x, 0) }

// Step 处理一个历元并返回该历元记录。重复/倒退时间戳分别返回哨兵错误，不推进状态。
func (svc *Service) Step(sess *Session, in EpochInput) (*EpochRecord, error) {
	prof, ok := svc.profiles[sess.ProfileName]
	if !ok {
		return nil, apierr.ErrProfileNotFound
	}
	if sess.LastTS != nil {
		if in.Timestamp == *sess.LastTS {
			return nil, apierr.ErrDuplicateEpoch
		}
		if in.Timestamp < *sess.LastTS {
			return nil, apierr.ErrStaleEpoch
		}
	}
	if err := validateEpochInput(in); err != nil {
		return nil, err
	}

	visible := map[gnss.SatKey]bool{}
	visibleSys := map[gnss.System]int{}
	for _, s := range in.Sats {
		sys, _ := gnss.ParseSystem(s.Sys)
		visible[gnss.Key(sys, s.ID)] = true
		visibleSys[sys]++
	}
	multi := len(visibleSys) > 1
	approx := svc.approxFor(sess, in)

	rec := &EpochRecord{
		Seq:         sess.Stats.Epochs + 1,
		Timestamp:   in.Timestamp,
		ProfileName: prof.Name,
		Pfa:         prof.Pfa,
		Pmd:         prof.Pmd,
		HAL:         prof.HAL,
	}

	// 1) 活动星（剔除隔离星）
	var active []lsq.Satellite
	for _, s := range in.Sats {
		sys, _ := gnss.ParseSystem(s.Sys)
		if sess.State.Isolated[gnss.Key(sys, s.ID)] != nil {
			continue
		}
		active = append(active, toLSQSat(s))
	}

	// 2) 单历元检测/排除
	ep := &lsq.Epoch{Approx: approx, Sats: active}
	if len(active) < 4 {
		svc.fillUnusable(sess, prof, rec, in,
			"隔离后活动星不足 4 颗，无法定位", visible, multi, visibleSys)
	} else {
		a, err := detect.Assess(ep, detect.Options{Pfa: prof.Pfa})
		if err != nil {
			// 走到这里的错误只可能是隔离后星数不足/几何退化（脏字段已在入口拦下），
			// 按本历元完好性不可用处理而不是拒收整段。
			reason := "活动星无法定位"
			if fe, ok := err.(*apierr.FieldError); ok {
				reason = "活动星无法定位：" + fe.Reason
			}
			svc.fillUnusable(sess, prof, rec, in, reason, visible, multi, visibleSys)
			svc.finishEpoch(sess, in, rec, visible, multi)
			return rec, nil
		}
		svc.fillFromAssessment(prof, rec, a)
		fillSystems(rec, in, a.Sol, visibleSys)
		// 3) 隔离星恢复评估（隔离期间仍逐历元算检验量）
		recovery := svc.evalRecovery(prof, in, sess.State.Isolated, approx)
		// 4) 推进隔离状态
		var newExcluded gnss.SatKey
		if rec.Mode == string(detect.ModeExcluded) {
			newExcluded = a.ExcludedKey()
		}
		sess.State.UpdateIsolation(prof, newExcluded, recovery, visible, rec.Seq)
		// 5) 告警
		svc.stepAlertAndStats(sess, prof, rec)
	}

	svc.finishEpoch(sess, in, rec, visible, multi)
	return rec, nil
}

// finishEpoch 写隔离列表并落账（正常路径与不可用路径共用）。
func (svc *Service) finishEpoch(sess *Session, in EpochInput, rec *EpochRecord,
	visible map[gnss.SatKey]bool, multi bool) {
	keys := sortedIsolatedKeys(sess.State.Isolated, visible)
	rec.Isolated = make([]int, 0, len(keys))
	for _, k := range keys {
		rec.Isolated = append(rec.Isolated, k.ID)
	}
	if multi {
		rec.IsolatedSats = make([]SatRef, 0, len(keys))
		for _, k := range keys {
			rec.IsolatedSats = append(rec.IsolatedSats, refOf(k))
		}
	}
	svc.commit(sess, in.Timestamp, rec)
}

func (svc *Service) fillUnusable(sess *Session, prof profile.Profile,
	rec *EpochRecord, in EpochInput, reason string,
	visible map[gnss.SatKey]bool, multi bool, visibleSys map[gnss.System]int) {
	rec.Mode = string(detect.ModeUnavailable)
	rec.RAIMAvail = false
	rec.Reason = reason
	fillSystems(rec, in, nil, visibleSys)
	// 活动星不足时隔离计数按“不可见/无法评估”处理：恢复评估依赖定位，
	// 无法完成，故所有本历元可见的隔离星 streak 清零（Normal=false）。
	var recovery []statem.SatRecovery
	for k := range sess.State.Isolated {
		if visible[k] {
			recovery = append(recovery, statem.SatRecovery{Sat: k, Normal: false})
		}
	}
	sess.State.UpdateIsolation(prof, gnss.SatKey{}, recovery, visible, rec.Seq)
	svc.stepAlert(sess, prof, rec, statem.EpochStatus{
		IntegrityAvailable: false, IntegrityBad: true,
	})
}

func (svc *Service) fillFromAssessment(prof profile.Profile,
	rec *EpochRecord, a *detect.Assessment) {
	rec.Mode = string(a.Mode)
	rec.SSE = a.SSE
	rec.DOF = a.DOF
	rec.Threshold = a.Threshold
	rec.SatResults = a.SatResults
	rec.Trials = a.Trials
	rec.Reason = a.Reason
	rec.ExcludedID = a.ExcludedID
	rec.ExcludedSys = a.ExcludedSys
	if a.Sol != nil {
		rec.Position = Position{
			ECEF: [3]float64{a.Sol.Pos.X, a.Sol.Pos.Y, a.Sol.Pos.Z},
			Lon:  a.Sol.LLA.Lon, Lat: a.Sol.LLA.Lat, Alt: a.Sol.LLA.Alt,
		}
		rec.ClockBias = a.Sol.ClockBias
		rec.Iterations = a.Sol.Iter
		rec.Converged = a.Sol.Converged
	}
	// RAIM 可用 ⇔ 全解存在冗余（dof≥1）；单 GPS 即升级前的 Used≥5。
	rec.RAIMAvail = a.DOF >= 1
	if rec.RAIMAvail {
		rec.HPL = protect.AssessmentHPL(a, prof.Pmd)
	}
}

// fillSystems 汇总各系统本历元的参与情况与钟差估计。
// sol 为报告解（排除后为干净子集解；不可用时为 nil）。
func fillSystems(rec *EpochRecord, in EpochInput, sol *lsq.Solution,
	visibleSys map[gnss.System]int) {
	if len(visibleSys) <= 1 {
		return // 单 GPS 历元不输出多系统字段（与升级前记录保持一致）
	}
	// 参考系统：报告解中按规范次序（GPS 优先）第一个参与系统。
	var refSys gnss.System
	refClock := 0.0
	if sol != nil {
		for _, sys := range gnss.All {
			if c, ok := sol.ClockOf(sys); ok {
				refSys, refClock = sys, c
				break
			}
		}
	}
	usedCount := map[gnss.System]int{}
	if sol != nil {
		for _, sys := range sol.RowSys {
			usedCount[sys]++
		}
	}
	for _, sys := range gnss.All {
		nv := visibleSys[sys]
		if nv == 0 {
			continue
		}
		r := SysClockResult{Sys: sys, Visible: nv}
		if sol != nil {
			if c, ok := sol.ClockOf(sys); ok {
				r.Participated = true
				r.Used = usedCount[sys]
				r.ClockBias = c
				if sys == refSys {
					r.ISB = 0
				} else {
					r.ISB = c - refClock
				}
			}
		}
		rec.Systems = append(rec.Systems, r)
	}
}

func (svc *Service) stepAlertAndStats(sess *Session, prof profile.Profile, rec *EpochRecord) {
	bad, gross := false, false
	switch rec.Mode {
	case string(detect.ModeDetected):
		bad = true
	case string(detect.ModeUnavailable):
		bad = true
	}
	if rec.RAIMAvail && rec.HPL > prof.HAL {
		bad = true
		if prof.Alert.Mode == profile.Combined && rec.HPL >= prof.Alert.GrossFactor*prof.HAL {
			gross = true
		}
	}
	if rec.Mode == string(detect.ModeDetected) && rec.Threshold > 0 &&
		prof.Alert.Mode == profile.Combined &&
		rec.SSE >= prof.Alert.GrossFactor*rec.Threshold {
		gross = true
	}
	svc.stepAlert(sess, prof, rec, statem.EpochStatus{
		IntegrityAvailable: rec.RAIMAvail,
		IntegrityBad:       bad,
		IdentifiedRisk: rec.Mode == string(detect.ModeDetected) ||
			rec.Mode == string(detect.ModeExcluded),
		Gross: gross,
	})
}

func (svc *Service) stepAlert(sess *Session, prof profile.Profile,
	rec *EpochRecord, st statem.EpochStatus) {
	wasActive := sess.State.Alert.Active
	active := sess.State.StepAlert(prof, st)
	rec.Alert = active
	rec.BadStreak = sess.State.Alert.BadStreak
	rec.GoodStreak = sess.State.Alert.GoodStreak

	if active && !wasActive {
		sess.Stats.AlertEpisodes++
		sess.AlertOpenRisk = false
	}
	if active && st.IdentifiedRisk {
		// 片段内任一年元识别出真实风险（检出故障），该片段不算误警
		sess.AlertOpenRisk = true
	}
	if !active && wasActive && !sess.AlertOpenRisk {
		sess.Stats.FalseAlarms++
	}
}

func (svc *Service) commit(sess *Session, ts int64, rec *EpochRecord) {
	sess.LastTS = &ts
	sess.Stats.Epochs++
	if rec.RAIMAvail {
		sess.Stats.RAIMAvailable++
	}
	if rec.Mode == string(detect.ModeDetected) || rec.Mode == string(detect.ModeExcluded) {
		sess.Stats.Detections++
	}
	if rec.Alert {
		sess.Stats.Alerts++
	}
	sess.Records = append(sess.Records, rec)
	if n := sess.Stats.Epochs; n > 0 {
		sess.Stats.RAIMAvailRate = float64(sess.Stats.RAIMAvailable) / float64(n)
		sess.Stats.ServiceAvail = float64(n-sess.Stats.Alerts) / float64(n)
	}
}

// approxFor 决定本历元迭代初值：显式提供 > 上一历元定位 > 首历元卫星反推。
func (svc *Service) approxFor(sess *Session, in EpochInput) geo.Vec {
	if in.Approx != nil {
		return geo.Vec{X: in.Approx[0], Y: in.Approx[1], Z: in.Approx[2]}
	}
	if n := len(sess.Records); n > 0 {
		p := sess.Records[n-1].Position.ECEF
		return geo.Vec{X: p[0], Y: p[1], Z: p[2]}
	}
	return firstSatApprox(in)
}

// firstSatApprox 在首历元未提供初值时，由卫星位置反推一个“指向该星的地表点”
// （单位矢量缩到 WGS84 长半轴）。距接收机通常在数百 km 内，足以让线性化收敛。
func firstSatApprox(in EpochInput) geo.Vec {
	for _, s := range in.Sats {
		p := geo.Vec{X: s.Pos[0], Y: s.Pos[1], Z: s.Pos[2]}
		r := geo.Norm(p)
		if r > 1 {
			return geo.Scale(p, geo.A/r)
		}
	}
	return geo.Vec{X: geo.A, Y: 0, Z: 0}
}

// evalRecovery 对每个“当前隔离且本历元可见”的星做恢复评估：
//   - 本星在含自身的全解中标准化残差正常；
//   - 把它加回去后整体检验通过（SSE ≤ 阈值）。
//
// 两者同时满足才算一个恢复历元。全部隔离星共用同一个“含所有可见星”的全解。
func (svc *Service) evalRecovery(prof profile.Profile, in EpochInput,
	isolated map[gnss.SatKey]*statem.IsolationEntry, approx geo.Vec) []statem.SatRecovery {
	if len(isolated) == 0 {
		return nil
	}
	all := make([]lsq.Satellite, 0, len(in.Sats))
	byKey := map[gnss.SatKey]lsq.Satellite{}
	for _, s := range in.Sats {
		l := toLSQSat(s)
		all = append(all, l)
		byKey[l.Key()] = l
	}
	var out []statem.SatRecovery
	// 解一次即可（全解与被评估的具体隔离星无关）。
	sol, solErr := lsq.Solve(&lsq.Epoch{Approx: approx, Sats: all})
	for k := range isolated {
		if _, ok := byKey[k]; !ok {
			continue // 不可见：上层 UpdateIsolation 会清零
		}
		if solErr != nil {
			out = append(out, statem.SatRecovery{Sat: k, Normal: false})
			continue
		}
		sse, dof := sol.SSE()
		overallPass := false
		selfZ := 0.0
		if dof >= 1 {
			thr := chisq.Threshold(dof, prof.Pfa)
			overallPass = sse <= thr
			for i, s := range all {
				if s.Key() == k {
					selfZ = math.Abs(sol.Resid[i]) / s.Sigma
					break
				}
			}
			// 单星正常门限取 √thr（与整体卡方阈值一致的保守判据）
			if overallPass && selfZ <= math.Sqrt(thr) {
				out = append(out, statem.SatRecovery{Sat: k, Normal: true})
				continue
			}
		}
		out = append(out, statem.SatRecovery{Sat: k, Normal: false})
	}
	return out
}

func toLSQSat(s SatInput) lsq.Satellite {
	sys, _ := gnss.ParseSystem(s.Sys)
	return lsq.Satellite{
		ID:    s.ID,
		Sys:   sys,
		Pos:   geo.Vec{X: s.Pos[0], Y: s.Pos[1], Z: s.Pos[2]},
		PR:    s.PR,
		Sigma: s.Sigma,
	}
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
