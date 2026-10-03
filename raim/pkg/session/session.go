// Package session 实现回放会话：绑定运行档、逐历元（或批量）提交，
// 编排“单历元定位/检测/排除/保护级”与“跨历元隔离、告警状态机”，
// 并做时间戳去重/倒退校验与统计。
//
// 多模（GPS/Galileo/北斗）同一历元的伪距一起参与定位、检测、排除与保护级；
// 每套系统各有一个钟差未知量（GPS 为基准），按历元自由估计，不跨历元携带
// （选型理由见 docs/design.md）。
//
// 同一段数据无论一次性批量提交还是逐历元实时提交，都走同一个 Step，
// 因此逐历元结果严格一致；状态持久化后重启续跑同样复用该 Step。
package session

import (
	"encoding/json"
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

// SatInput 是调用方提交的单星观测。System 缺省（空串）时按 GPS 处理，
// 以兼容升级前没有系统标识的旧会话与自建运行档。
type SatInput struct {
	System gnss.System `json:"system,omitempty"`
	ID     int         `json:"id"`
	Pos    [3]float64  `json:"pos"` // ECEF x,y,z
	PR     float64     `json:"pr"`
	Sigma  float64     `json:"sigma"`
}

// Key 返回该星“系统+编号”的唯一身份。
func (s SatInput) Key() gnss.SatID { return gnss.SatID{System: s.System, ID: s.ID} }

// EpochInput 是一个历元的提交内容。Approx 为可选概略位置（首历元建议提供）。
type EpochInput struct {
	Timestamp int64       `json:"timestamp"` // 单调时间戳（任意单位，如 Unix 秒）
	Approx    *[3]float64 `json:"approx,omitempty"`
	Sats      []SatInput  `json:"satellites"`
}

// SystemClock 是一个历元内一套系统的时间偏差估计结果。
type SystemClock struct {
	System gnss.System `json:"system"`
	// ClockBias 该系统时相对 GPS 时的偏差（米）；GPS 项即接收机钟差。
	ClockBias float64 `json:"clock_bias"`
	// Used 本历元该系统是否实际参与解算（可见且未被整体隔离）。
	Used bool `json:"used"`
	// SatCount 该系统实际参与解算的星数。
	SatCount int `json:"sat_count"`
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
	Mode       string             `json:"mode"` // ok/unavailable/detected/excluded
	SSE        float64            `json:"sse"`
	HPL        float64            `json:"hpl"`
	ExcludedID int                `json:"excluded_id,omitempty"`
	Isolated   []gnss.SatRef      `json:"isolated"` // 本历元处于隔离（不参与解算）的星
	Position   Position           `json:"position"`
	ClockBias  float64            `json:"clock_bias"`
	Iterations int                `json:"iterations"`
	Converged  bool               `json:"converged"`
	SatResults []detect.SatResult `json:"sat_results"`
	Trials     []detect.Trial     `json:"trials,omitempty"`

	// 跨历元状态
	Alert      bool `json:"alert"`
	BadStreak  int  `json:"bad_streak"`
	GoodStreak int  `json:"good_streak"`
	RAIMAvail  bool `json:"raim_available"`

	Reason string `json:"reason,omitempty"`

	// 多模新增（升级前落盘的旧记录没有这些字段，读取时为零值，绝不回写旧记录）。
	// ExcludedSystem：被排除星所属系统（GPS 时省略，保持旧形态）。
	ExcludedSystem gnss.System   `json:"excluded_system,omitempty"`
	SystemClocks   []SystemClock `json:"system_clocks,omitempty"`

	// raw 保存从磁盘读出的原始 JSON（仅旧记录有），重新落盘时原样保留，
	// 保证“已经落盘的历元记录不能改写”。内存新记录为 nil，正常序列化。
	raw json.RawMessage `json:"-"`
}

// MarshalJSON：旧记录（带 raw）原样输出，新记录按结构序列化；
// GPS 单模时省略 excluded_system，保持升级前记录形态。
func (r EpochRecord) MarshalJSON() ([]byte, error) {
	if len(r.raw) > 0 {
		return r.raw, nil
	}
	type alias EpochRecord // 去掉本方法，避免递归
	v := alias(r)
	if r.ExcludedSystem == gnss.GPS || r.ExcludedSystem == "" {
		v.ExcludedSystem = ""
	}
	return json.Marshal(v)
}

// UnmarshalJSON：保留原始字节以便后续原样落盘。
func (r *EpochRecord) UnmarshalJSON(b []byte) error {
	type alias EpochRecord
	var v alias
	if err := json.Unmarshal(b, &v); err != nil {
		return err
	}
	*r = EpochRecord(v)
	r.raw = append(json.RawMessage(nil), b...)
	return nil
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

// validateSats 做本层输入校验：未知系统标识、同系统重号都要拒（跨系统同号允许）。
// 错误指到具体字段；调用方在批量路径上再统一加 epochs[i]. 前缀。
func validateSats(sats []SatInput) error {
	seen := map[gnss.SatID]bool{}
	sysCount := map[gnss.System]bool{}
	for i, s := range sats {
		field := func(name string) string {
			return "satellites[" + itoa(i) + "]." + name
		}
		if s.System != "" && !s.System.Valid() {
			return apierr.New(field("system"),
				fmt.Sprintf("不认识的系统标识 %q（只支持 GPS/GAL/BDS）", string(s.System)))
		}
		k := s.Key()
		if seen[k] {
			return apierr.New(field("id"),
				fmt.Sprintf("卫星编号在同一系统内重复：%s %d", string(s.System), s.ID))
		}
		seen[k] = true
		sysCount[s.System] = true
	}
	// 定位需要 3+k 颗星（k 个不同系统）。
	if need := 3 + len(sysCount); len(sats) < need {
		return apierr.New("satellites",
			fmt.Sprintf("历元可见星 %d 颗，不足定位所需 %d 颗（3 个位置未知量 + 每个系统 1 个钟差）",
				len(sats), need))
	}
	return nil
}

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
	// 兼容：缺系统标识的星按 GPS（先归一化再校验，旧数据无需带标识）。
	for i := range in.Sats {
		if in.Sats[i].System == "" {
			in.Sats[i].System = gnss.GPS
		}
	}
	if err := validateSats(in.Sats); err != nil {
		return nil, err
	}

	visible := map[gnss.SatID]bool{}
	for _, s := range in.Sats {
		visible[s.Key()] = true
	}
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
	activeSysCount := map[gnss.System]int{}
	for _, s := range in.Sats {
		if sess.State.Isolated[s.Key()] != nil {
			continue
		}
		active = append(active, toLSQSat(s))
		activeSysCount[s.System]++
	}

	// 2) 单历元检测/排除
	ep := &lsq.Epoch{Approx: approx, Sats: active}
	if len(active) < 3+len(activeSysCount) {
		svc.fillUnusable(sess, prof, rec, "隔离后活动星不足（3+系统数），无法定位", visible)
	} else {
		a, err := detect.Assess(ep, detect.Options{Pfa: prof.Pfa})
		if err != nil {
			return nil, err
		}
		svc.fillFromAssessment(prof, rec, a)
		// 3) 隔离星恢复评估（隔离期间仍逐历元算检验量）
		recovery := svc.evalRecovery(prof, in, sess.State.Isolated, approx)
		// 4) 推进隔离状态
		var newExcluded gnss.SatID
		if rec.Mode == string(detect.ModeExcluded) {
			newExcluded = gnss.SatID{System: rec.ExcludedSystem, ID: rec.ExcludedID}
		}
		sess.State.UpdateIsolation(prof, newExcluded, recovery, visible, rec.Seq)
		// 5) 告警
		svc.stepAlertAndStats(sess, prof, rec)
	}

	rec.Isolated = sortedIsolated(sess.State.Isolated, visible)

	// 6) 落账
	svc.commit(sess, in.Timestamp, rec)
	return rec, nil
}

// sortedIsolated 返回当前隔离且本历元可见的星引用（按系统、编号升序）。
func sortedIsolated(isolated map[gnss.SatID]*statem.IsolationEntry,
	visible map[gnss.SatID]bool) []gnss.SatRef {
	var keys []gnss.SatID
	for k := range isolated {
		if visible[k] {
			keys = append(keys, k)
		}
	}
	sort.Slice(keys, func(i, j int) bool { return gnss.Compare(keys[i], keys[j]) < 0 })
	out := make([]gnss.SatRef, 0, len(keys))
	for _, k := range keys {
		out = append(out, gnss.Ref(k))
	}
	return out
}

func (svc *Service) fillUnusable(sess *Session, prof profile.Profile,
	rec *EpochRecord, reason string, visible map[gnss.SatID]bool) {
	rec.Mode = string(detect.ModeUnavailable)
	rec.RAIMAvail = false
	rec.Reason = reason
	rec.SystemClocks = unusedSystemClocks()
	// 活动星不足时隔离计数按“不可见/无法评估”处理：恢复评估依赖定位，
	// 无法完成，故所有可见隔离星 streak 清零（Normal=false）。
	var recovery []statem.SatRecovery
	for id := range sess.State.Isolated {
		if visible[id] {
			recovery = append(recovery, statem.SatRecovery{Sat: id, Normal: false})
		}
	}
	sess.State.UpdateIsolation(prof, gnss.SatID{}, recovery, visible, rec.Seq)
	svc.stepAlert(sess, prof, rec, statem.EpochStatus{
		IntegrityAvailable: false, IntegrityBad: true,
	})
}

// unusedSystemClocks 构造固定三系统顺序的占位钟差表（无法定位时）。
func unusedSystemClocks() []SystemClock {
	out := make([]SystemClock, 0, len(gnss.Systems()))
	for _, sys := range gnss.Systems() {
		out = append(out, SystemClock{System: sys})
	}
	return out
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
	if a.ExcludedSystem != "" {
		rec.ExcludedSystem = a.ExcludedSystem
	}
	if a.Sol != nil {
		rec.Position = Position{
			ECEF: [3]float64{a.Sol.Pos.X, a.Sol.Pos.Y, a.Sol.Pos.Z},
			Lon:  a.Sol.LLA.Lon, Lat: a.Sol.LLA.Lat, Alt: a.Sol.LLA.Alt,
		}
		rec.ClockBias = a.Sol.ClockBias
		rec.Iterations = a.Sol.Iter
		rec.Converged = a.Sol.Converged
		// 报告解（排除成功后是剔星干净解）各系统钟差与参与计数
		rec.SystemClocks = systemClocksFor(a.Sol)
	}
	rec.RAIMAvail = a.SolDOF >= 1
	if rec.RAIMAvail {
		rec.HPL = protect.AssessmentHPL(a, prof.Pmd)
	}
}

// systemClocksFor 按报告解实际包含的系统统计钟差与每系统星数。
func systemClocksFor(sol *lsq.Solution) []SystemClock {
	count := map[gnss.System]int{}
	for _, k := range sol.SatIDs {
		count[k.System]++
	}
	out := make([]SystemClock, 0, len(gnss.Systems()))
	for _, sys := range gnss.Systems() {
		sc := SystemClock{System: sys}
		if b, ok := sol.ClockBySys[sys]; ok {
			sc.Used = true
			sc.ClockBias = b
			sc.SatCount = count[sys]
		}
		out = append(out, sc)
	}
	return out
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
// 两者同时满足才算一个恢复历元。全解按多模模型（每系统一个钟差列）重算。
func (svc *Service) evalRecovery(prof profile.Profile, in EpochInput,
	isolated map[gnss.SatID]*statem.IsolationEntry, approx geo.Vec) []statem.SatRecovery {
	if len(isolated) == 0 {
		return nil
	}
	all := make([]lsq.Satellite, 0, len(in.Sats))
	byKey := map[gnss.SatID]lsq.Satellite{}
	sysCount := map[gnss.System]bool{}
	for _, s := range in.Sats {
		l := toLSQSat(s)
		all = append(all, l)
		byKey[s.Key()] = l
		sysCount[s.System] = true
	}
	var out []statem.SatRecovery
	// 隔离星不可见直接跳过（上层按不可见清零）。
	for id := range isolated {
		if _, ok := byKey[id]; !ok {
			continue
		}
		if len(all) < 3+len(sysCount) {
			out = append(out, statem.SatRecovery{Sat: id, Normal: false})
			continue
		}
		sol, err := lsq.Solve(&lsq.Epoch{Approx: approx, Sats: all})
		if err != nil {
			out = append(out, statem.SatRecovery{Sat: id, Normal: false})
			continue
		}
		sse, dof := lsq.WeightedSSEDOF(sol.Resid, sol.Sigma, 3+len(sol.Systems))
		if dof < 1 {
			out = append(out, statem.SatRecovery{Sat: id, Normal: false})
			continue
		}
		thr := chisq.Threshold(dof, prof.Pfa)
		overallPass := sse <= thr
		selfZ := 0.0
		for i, s := range all {
			if s.System == id.System && s.ID == id.ID {
				selfZ = math.Abs(sol.Resid[i]) / s.Sigma
				break
			}
		}
		// 单星正常门限取 √thr（与整体卡方阈值一致的保守判据）
		if overallPass && selfZ <= math.Sqrt(thr) {
			out = append(out, statem.SatRecovery{Sat: id, Normal: true})
		} else {
			out = append(out, statem.SatRecovery{Sat: id, Normal: false})
		}
	}
	return out
}

func toLSQSat(s SatInput) lsq.Satellite {
	return lsq.Satellite{
		System: s.System,
		ID:     s.ID,
		Pos:    geo.Vec{X: s.Pos[0], Y: s.Pos[1], Z: s.Pos[2]},
		PR:     s.PR,
		Sigma:  s.Sigma,
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
