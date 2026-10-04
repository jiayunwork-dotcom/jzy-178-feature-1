package session_test

import (
	"math"
	"math/rand"
	"os"
	"path/filepath"
	"reflect"
	"testing"

	"raim/internal/sim"
	"raim/pkg/apierr"
	"raim/pkg/gnss"
	"raim/pkg/lsq"
	"raim/pkg/profile"
	"raim/pkg/session"
)

// toMultiInput 把多模 lsq 历元转成提交输入，保留系统标识（GPS 星省略）。
func toMultiInput(ts int64, ep *lsq.Epoch) session.EpochInput {
	sats := make([]session.SatInput, len(ep.Sats))
	for i, s := range ep.Sats {
		in := session.SatInput{
			ID:    s.ID,
			Pos:   [3]float64{s.Pos.X, s.Pos.Y, s.Pos.Z},
			PR:    s.PR,
			Sigma: s.Sigma,
		}
		if s.Sys != "" && s.Sys != gnss.GPS {
			in.Sys = string(s.Sys)
		}
		sats[i] = in
	}
	ap := [3]float64{ep.Approx.X, ep.Approx.Y, ep.Approx.Z}
	return session.EpochInput{Timestamp: ts, Approx: &ap, Sats: sats}
}

// multiConfig 构造三系统共视星座（每系统 7 颗，几何充分）。
func multiConfig() *sim.Constellation {
	rec := sim.DefaultReceiver()
	return sim.MultiSky(rec,
		map[gnss.System]int{gnss.GPS: 7, gnss.Galileo: 7, gnss.BeiDou: 7},
		15, 20261004)
}

type multiGen struct {
	c        *sim.Constellation
	sigma    float64
	seed     int64
	biasAt   func(seq0 int) map[gnss.SatKey]float64
	sysClock func(seq0 int) map[gnss.System]float64
}

func (g multiGen) epochs(n int) []session.EpochInput {
	rng := rand.New(rand.NewSource(g.seed))
	out := make([]session.EpochInput, n)
	for i := range out {
		o := sim.Obs{Rng: rng, Sigma: g.sigma}
		if g.biasAt != nil {
			o.BiasK = g.biasAt(i)
		}
		if g.sysClock != nil {
			o.SysClock = g.sysClock(i)
		}
		out[i] = toMultiInput(int64(i+1), g.c.Observe(o))
	}
	return out
}

func findSys(rec *session.EpochRecord, sys gnss.System) *session.SysClockResult {
	for i := range rec.Systems {
		if rec.Systems[i].Sys == sys {
			return &rec.Systems[i]
		}
	}
	return nil
}

func TestMultiSystemsReportedPerEpoch(t *testing.T) {
	c := multiConfig()
	g := multiGen{c: c, sigma: 1, seed: 11}
	svc := session.NewService(profile.Builtins())
	sess, err := svc.NewSession("s", "enroute")
	if err != nil {
		t.Fatal(err)
	}
	rec, err := svc.Step(sess, g.epochs(1)[0])
	if err != nil {
		t.Fatal(err)
	}
	if len(rec.Systems) != 3 {
		t.Fatalf("三系统都应出现在 systems, got %d", len(rec.Systems))
	}
	for _, sys := range gnss.All {
		r := findSys(rec, sys)
		if r == nil {
			t.Fatalf("缺少系统 %s 的结果", sys)
		}
		if !r.Participated || r.Used != 7 || r.Visible != 7 {
			t.Fatalf("%s 应参与且 7 星: %+v", sys, *r)
		}
	}
	if gps := findSys(rec, gnss.GPS); gps.ISB != 0 {
		t.Fatalf("参考系统 GPS 的 ISB 应为 0, got %v", gps.ISB)
	}
	// 定位在真值附近
	if d := norm3(rec.Position.ECEF, [3]float64{c.TruePos().X, c.TruePos().Y, c.TruePos().Z}); d > 10 {
		t.Fatalf("定位偏差 %.2f m", d)
	}
}

func TestGalileoConstantStepPositionUnchanged(t *testing.T) {
	// Galileo 全部伪距从第 6 历元起加 300 km 常数：
	// 选了“每历元自由估计 ISB”，台阶在出现的当历元即被 GAL 钟差列完全吸收，
	// 位置、SSE、检测结论逐历元不受影响，不存在跨历元过渡。
	c := multiConfig()
	const step = 300_000.0
	g := multiGen{
		c: c, sigma: 1, seed: 23,
		sysClock: func(i int) map[gnss.System]float64 {
			if i >= 5 {
				return map[gnss.System]float64{gnss.Galileo: step}
			}
			return nil
		},
	}
	epochs := g.epochs(20)

	// 同一数据无台阶的对照（只用于位置比对：台阶前后位置应与干净轨迹一致）
	gClean := multiGen{c: c, sigma: 1, seed: 23}
	clean := gClean.epochs(20)

	run := func(ins []session.EpochInput) []*session.EpochRecord {
		svc := session.NewService(profile.Builtins())
		sess, _ := svc.NewSession("s", "enroute")
		var recs []*session.EpochRecord
		for _, in := range ins {
			r, err := svc.Step(sess, in)
			if err != nil {
				t.Fatal(err)
			}
			recs = append(recs, r)
		}
		return recs
	}
	stepped := run(epochs)
	cleanRecs := run(clean)

	for i := range stepped {
		r := stepped[i]
		if r.Mode == "detected" {
			t.Fatalf("历元 %d 整系统常数台阶不应检出: SSE=%.2f thr=%.2f", i+1, r.SSE, r.Threshold)
		}
		// 位置与干净轨迹（同噪声种子）一致到毫米级——台阶完全不影响位置
		if d := norm3(r.Position.ECEF, cleanRecs[i].Position.ECEF); d > 1e-3 {
			t.Fatalf("历元 %d 台阶后位置偏移 %.6f m", i+1, d)
		}
		// SSE 也必须一致（台阶是可估参数，不进残差）
		if math.Abs(r.SSE-cleanRecs[i].SSE) > 1e-6 {
			t.Fatalf("历元 %d SSE 因台阶改变: %.9f vs %.9f", i+1, r.SSE, cleanRecs[i].SSE)
		}
		gal := findSys(r, gnss.Galileo)
		if gal == nil || !gal.Participated {
			t.Fatalf("历元 %d GAL 应参与", i+1)
		}
		galClean := findSys(cleanRecs[i], gnss.Galileo)
		got := gal.ClockBias - galClean.ClockBias
		if i < 5 {
			if math.Abs(got) > 1e-3 {
				t.Fatalf("台阶前 GAL 钟差不应偏, got %v", got)
			}
		} else {
			// 台阶当历元（第 6 历元）即完整 +300km：零过渡、零瞬态
			if math.Abs(got-step) > 1e-3 {
				t.Fatalf("历元 %d GAL 钟差应已吸收台阶 %v, got %v", i+1, step, got)
			}
		}
	}
}

func TestGPS3BiasIsolationDoesNotTouchBDS3(t *testing.T) {
	c := multiConfig()
	const minIso = 4
	prof := profile.Profile{
		Name: "iso", Pfa: 1e-5, Pmd: 1e-3, HAL: 1e9,
		Isolation: profile.Isolation{MinEpochs: minIso},
		Alert:     profile.Alert{Mode: profile.Snapshot, ConfirmEpochs: 1, ClearEpochs: 1},
	}
	g := multiGen{
		c: c, sigma: 1, seed: 31,
		biasAt: func(i int) map[gnss.SatKey]float64 {
			if i >= 3 && i <= 10 {
				return map[gnss.SatKey]float64{gnss.Key(gnss.GPS, 3): 150}
			}
			return nil
		},
	}
	epochs := g.epochs(18)
	svc := session.NewService([]profile.Profile{prof})
	sess, _ := svc.NewSession("s", "iso")
	var recs []*session.EpochRecord
	for _, e := range epochs {
		r, err := svc.Step(sess, e)
		if err != nil {
			t.Fatal(err)
		}
		recs = append(recs, r)
	}

	// 首个故障历元排除 GPS:3
	first := recs[3]
	if first.Mode != "excluded" ||
		first.ExcludedID != 3 || first.ExcludedSys != gnss.System("") {
		t.Fatalf("应排除 GPS:3, got mode=%s %s:%d", first.Mode, first.ExcludedSys, first.ExcludedID)
	}
	if excludedRecordKey(first) != gnss.Key(gnss.GPS, 3) {
		t.Fatal("排除记录主键应为 GPS:3")
	}
	// 隔离列表主键必须带系统，且只能找到 GPS:3
	for i := 4; i <= 11; i++ {
		r := recs[i]
		if !containsRef(r.IsolatedSats, gnss.Key(gnss.GPS, 3)) {
			t.Fatalf("历元 %d 应隔离 GPS:3, isolated=%+v", i+1, r.IsolatedSats)
		}
		if containsRef(r.IsolatedSats, gnss.Key(gnss.BeiDou, 3)) {
			t.Fatalf("历元 %d 绝不能隔离 BDS:3", i+1)
		}
		// GPS:3 不参与，BDS:3 照常参与
		gps3Used, bds3Used := false, false
		for _, sr := range r.SatResults {
			if sr.Key() == gnss.Key(gnss.GPS, 3) {
				gps3Used = true
			}
			if sr.Key() == gnss.Key(gnss.BeiDou, 3) {
				bds3Used = true
			}
		}
		if gps3Used {
			t.Fatalf("历元 %d GPS:3 隔离期间不应参与", i+1)
		}
		if !bds3Used {
			t.Fatalf("历元 %d BDS:3 应照常参与", i+1)
		}
	}
	// 故障期间位置贴近真值
	for i := 3; i <= 10; i++ {
		d := norm3(recs[i].Position.ECEF,
			[3]float64{c.TruePos().X, c.TruePos().Y, c.TruePos().Z})
		if d > 10 {
			t.Fatalf("历元 %d 定位偏差 %.2f m", i+1, d)
		}
	}
	// 偏差撤掉后连续 minIso 历元恢复
	for i := 11; i <= 13; i++ {
		if !containsRef(recs[i].IsolatedSats, gnss.Key(gnss.GPS, 3)) {
			t.Fatalf("历元 %d 恢复期未满应仍隔离", i+1)
		}
	}
	if containsRef(recs[14].IsolatedSats, gnss.Key(gnss.GPS, 3)) {
		t.Fatal("历元 15 应解除 GPS:3 隔离")
	}
}

func containsRef(refs []session.SatRef, k gnss.SatKey) bool {
	for _, r := range refs {
		sys := r.Sys
		if sys == "" {
			sys = gnss.GPS
		}
		if sys == k.Sys && r.ID == k.ID {
			return true
		}
	}
	return false
}

// excludedRecordKey 从历元记录取被排除星主键（空系统按 GPS）。
func excludedRecordKey(r *session.EpochRecord) gnss.SatKey {
	sys := r.ExcludedSys
	if sys == "" {
		sys = gnss.GPS
	}
	return gnss.Key(sys, r.ExcludedID)
}

func TestMultiBatchEqualsIndividual(t *testing.T) {
	c := multiConfig()
	g := multiGen{
		c: c, sigma: 1, seed: 55,
		biasAt: func(i int) map[gnss.SatKey]float64 {
			if i >= 6 && i <= 14 {
				return map[gnss.SatKey]float64{gnss.Key(gnss.BeiDou, 2): 130}
			}
			return nil
		},
		sysClock: func(i int) map[gnss.System]float64 {
			if i >= 10 {
				return map[gnss.System]float64{gnss.Galileo: 12_345}
			}
			return nil
		},
	}
	epochs := g.epochs(40)

	stA := newStore(t)
	if _, err := stA.CreateSession("a", "enroute"); err != nil {
		t.Fatal(err)
	}
	recsA, err := stA.AppendBatch("a", epochs)
	if err != nil {
		t.Fatal(err)
	}
	sessA, _ := stA.GetSession("a")

	stB := newStore(t)
	if _, err := stB.CreateSession("b", "enroute"); err != nil {
		t.Fatal(err)
	}
	var recsB []*session.EpochRecord
	for _, e := range epochs {
		r, err := stB.AppendEpoch("b", e)
		if err != nil {
			t.Fatal(err)
		}
		recsB = append(recsB, r)
	}
	sessB, _ := stB.GetSession("b")

	assertMultiRecordsEqual(t, recsA, recsB)
	assertMultiRecordsEqual(t, sessA.Records, sessB.Records)
	if !reflect.DeepEqual(sessA.Stats, sessB.Stats) {
		t.Fatalf("统计不一致:\n%+v\n%+v", sessA.Stats, sessB.Stats)
	}
	if len(sessA.State.Isolated) != len(sessB.State.Isolated) {
		t.Fatalf("隔离状态数量不同")
	}
	for k, ea := range sessA.State.Isolated {
		eb := sessB.State.Isolated[k]
		if eb == nil || !reflect.DeepEqual(ea, eb) {
			t.Fatalf("隔离星 %s 状态不同 %+v vs %+v", k, ea, eb)
		}
	}
}

func assertMultiRecordsEqual(t *testing.T, a, b []*session.EpochRecord) {
	t.Helper()
	if len(a) != len(b) {
		t.Fatalf("历元数不同 %d vs %d", len(a), len(b))
	}
	for i := range a {
		if !reflect.DeepEqual(a[i], b[i]) {
			t.Fatalf("历元 %d (ts=%d) 不一致:\nA=%+v\nB=%+v", i, a[i].Timestamp, a[i], b[i])
		}
	}
}

func TestMultiRestartResumeEqualsContinuous(t *testing.T) {
	c := multiConfig()
	g := multiGen{
		c: c, sigma: 1, seed: 77,
		biasAt: func(i int) map[gnss.SatKey]float64 {
			if i >= 5 && i <= 25 {
				return map[gnss.SatKey]float64{
					gnss.Key(gnss.Galileo, 4): 140,
					gnss.Key(gnss.BeiDou, 6):  20,
				}
			}
			return nil
		},
	}
	epochs := g.epochs(50)
	dir := filepath.Join(t.TempDir(), "data")

	stFull, err := session.NewStore(dir + "_full")
	if err != nil {
		t.Fatal(err)
	}
	if _, err := stFull.CreateSession("s", "enroute"); err != nil {
		t.Fatal(err)
	}
	if _, err := stFull.AppendBatch("s", epochs); err != nil {
		t.Fatal(err)
	}
	sessFull, _ := stFull.GetSession("s")

	const split = 21
	st1, err := session.NewStore(dir)
	if err != nil {
		t.Fatal(err)
	}
	if _, err := st1.CreateSession("s", "enroute"); err != nil {
		t.Fatal(err)
	}
	for _, e := range epochs[:split] {
		if _, err := st1.AppendEpoch("s", e); err != nil {
			t.Fatal(err)
		}
	}
	// 重启：新 Store 指向同一目录
	st2, err := session.NewStore(dir)
	if err != nil {
		t.Fatal(err)
	}
	for _, e := range epochs[split:] {
		if _, err := st2.AppendEpoch("s", e); err != nil {
			t.Fatal(err)
		}
	}
	sessResumed, _ := st2.GetSession("s")

	assertMultiRecordsEqual(t, sessResumed.Records, sessFull.Records)
	if !reflect.DeepEqual(sessResumed.Stats, sessFull.Stats) ||
		!reflect.DeepEqual(sessResumed.State, sessFull.State) {
		t.Fatalf("重启续跑状态不一致:\n%+v\n%+v", sessResumed.State, sessFull.State)
	}
}

func TestMultiValidationErrors(t *testing.T) {
	c := multiConfig()
	g := multiGen{c: c, sigma: 1, seed: 3}
	epochs := g.epochs(4)

	// 未知系统标识 -> 指到 sys 字段
	bad := epochs[0]
	bad.Timestamp = 100
	bad.Sats[2].Sys = "GLONASS"
	st := newStore(t)
	if _, err := st.CreateSession("s", "enroute"); err != nil {
		t.Fatal(err)
	}
	_, err := st.AppendEpoch("s", bad)
	fe, ok := err.(*apierr.FieldError)
	if !ok || fe.Field != "satellites[2].sys" {
		t.Fatalf("未知系统应指到 satellites[2].sys, got %v", err)
	}

	// 同系统重号 -> 拒，指到 id 字段
	bad2 := epochs[1]
	bad2.Timestamp = 101
	bad2.Sats[1].ID = bad2.Sats[0].ID
	bad2.Sats[1].Sys = bad2.Sats[0].Sys
	_, err = st.AppendEpoch("s", bad2)
	fe, ok = err.(*apierr.FieldError)
	if !ok || fe.Field != "satellites[1].id" {
		t.Fatalf("同系统重号应指到 satellites[1].id, got %v", err)
	}

	// 跨系统同号合法：GPS 1 号 + GAL 1 号，应被接受
	ok3 := epochs[2]
	ok3.Timestamp = 102
	for i := range ok3.Sats {
		if ok3.Sats[i].Sys == "GAL" {
			ok3.Sats[i].ID = 1
			break
		}
	}
	// GPS 1 号本来就存在（编号从 1 起）
	if _, err := st.AppendEpoch("s", ok3); err != nil {
		t.Fatalf("跨系统同号应被接受: %v", err)
	}
}

func TestMultiBatchErrorHasEpochIndex(t *testing.T) {
	c := multiConfig()
	g := multiGen{c: c, sigma: 1, seed: 4}
	epochs := g.epochs(6)
	epochs[3].Sats[5].Sys = "SBAS" // 第 4 个历元非法系统
	st := newStore(t)
	if _, err := st.CreateSession("s", "enroute"); err != nil {
		t.Fatal(err)
	}
	_, err := st.AppendBatch("s", epochs)
	fe, ok := err.(*apierr.FieldError)
	if !ok {
		t.Fatalf("期望字段错误, got %v", err)
	}
	if fe.Field != "epochs[3].satellites[5].sys" {
		t.Fatalf("批量错误字段应带历元下标, got %s", fe.Field)
	}
	sess, _ := st.GetSession("s")
	if sess.Stats.Epochs != 0 {
		t.Fatalf("整批非法不应推进, epochs=%d", sess.Stats.Epochs)
	}
}

// TestGPSOnlyMultiPathIdenticalToLegacy 用升级后代码跑纯 GPS 数据，
// 逐字段对比升级前黄金文件 docs/replay_results.json，并确认多系统字段为空。
func TestGPSOnlyNoMultiFields(t *testing.T) {
	c := sim.EvenSky(sim.DefaultReceiver(), 9, 15, 5)
	epochs := genEpochs(c, 10, 1, 5, faultWindow(4, 5, 7, 80))
	st := newStore(t)
	if _, err := st.CreateSession("s", "enroute"); err != nil {
		t.Fatal(err)
	}
	recs, err := st.AppendBatch("s", epochs)
	if err != nil {
		t.Fatal(err)
	}
	for i, r := range recs {
		if len(r.Systems) != 0 {
			t.Fatalf("纯 GPS 历元 %d 不应有 systems 字段, got %+v", i+1, r.Systems)
		}
		if len(r.IsolatedSats) != 0 {
			t.Fatalf("纯 GPS 历元 %d 不应有 isolated_sats", i+1)
		}
		if r.ExcludedSys != "" {
			t.Fatalf("纯 GPS 排除不应有 excluded_sys")
		}
		for _, sr := range r.SatResults {
			if sr.Sys != "" {
				t.Fatalf("纯 GPS 每星结果 sys 应省略")
			}
		}
	}
}

// TestLegacySessionFileContinues 手工写一个升级前格式的会话文件，
// 升级后必须能直接接着提交：旧记录原样、裸编号隔离状态被解释为 GPS、新历元推进。
func TestLegacySessionFileContinues(t *testing.T) {
	dir := filepath.Join(t.TempDir(), "data")
	st, err := session.NewStore(dir)
	if err != nil {
		t.Fatal(err)
	}
	// 先建一个新会话拿到目录结构
	if _, err := st.CreateSession("legacy", "enroute"); err != nil {
		t.Fatal(err)
	}
	// 用升级前的 JSON 形态覆盖：isolated 键是裸 "5"，记录里没有 systems/sys 字段
	legacyJSON := `{
  "id": "legacy",
  "profile_name": "enroute",
  "created_at": "2026-09-30T08:00:00Z",
  "last_ts": 1,
  "state": {
    "isolated": {"5": {"id": 5, "normal_streak": 2, "since_epoch_seq": 1}},
    "alert": {"active": false, "bad_streak": 0, "good_streak": 1}
  },
  "stats": {"epochs":1,"raim_available_epochs":1,"alert_epochs":0,"detections":1,
            "alert_episodes":0,"false_alarm_episodes":0,
            "raim_availability":1,"service_availability":1},
  "records": [
    {"seq":1,"timestamp":1,"profile_name":"enroute","pfa":1e-5,"pmd":1e-3,"hal":3704,
     "chi_square_threshold":0,"dof":5,"mode":"excluded","sse":123.4,"hpl":0,
     "excluded_id":5,"isolated":[5],
     "position":{"ecef":[4096000,3096000,3830000],"lon":116.39,"lat":39.9,"alt":50},
     "clock_bias":1.5,"iterations":3,"converged":true,
     "sat_results":[{"id":5,"resid":80,"sigma":1,"stdres":80}],
     "alert":false,"bad_streak":0,"good_streak":1,"raim_available":true}
  ],
  "alert_open_risk": false
}`
	path := filepath.Join(st.Dir(), "sessions", "legacy.json")
	if err := os.WriteFile(path, []byte(legacyJSON), 0o644); err != nil {
		t.Fatal(err)
	}

	sess, err := st.GetSession("legacy")
	if err != nil {
		t.Fatalf("旧会话应能读取: %v", err)
	}
	if len(sess.Records) != 1 || sess.Records[0].ExcludedID != 5 {
		t.Fatal("旧记录应原样读入")
	}
	entry, ok := sess.State.Isolated[gnss.Key(gnss.GPS, 5)]
	if !ok {
		t.Fatalf("裸编号 5 应被迁移为 GPS:5, isolated=%v", sess.State.Isolated)
	}
	if entry.NormalStreak != 2 || entry.Sat != gnss.Key(gnss.GPS, 5) {
		t.Fatalf("迁移后的隔离条目异常: %+v", entry)
	}
	oldRec := sess.Records[0]

	// 接着提交新历元（纯 GPS 9 星）
	c := sim.EvenSky(sim.DefaultReceiver(), 9, 15, 5)
	ep := genEpochs(c, 1, 1, 999, nil)[0]
	ep.Timestamp = 2
	rec, err := st.AppendEpoch("legacy", ep)
	if err != nil {
		t.Fatalf("旧会话应能继续提交: %v", err)
	}
	if rec.Seq != 2 {
		t.Fatalf("新历元 seq 应为 2, got %d", rec.Seq)
	}
	sess2, _ := st.GetSession("legacy")
	// 旧记录不得被改写：关键字段保持
	if !reflect.DeepEqual(sess2.Records[0], oldRec) {
		t.Fatalf("已落盘旧记录被改写:\nold=%+v\nnew=%+v", oldRec, sess2.Records[0])
	}
	if sess2.Stats.Epochs != 2 {
		t.Fatalf("历元数应推进到 2, got %d", sess2.Stats.Epochs)
	}
}

func TestSystemDisappearsAndReappears(t *testing.T) {
	// 选每历元自由估计：Galileo 消失若干历元后再出现，首历元即正常，
	// 不携带任何陈旧偏差，位置与检测不受影响。
	rec := sim.DefaultReceiver()
	c := sim.MultiSky(rec,
		map[gnss.System]int{gnss.GPS: 7, gnss.Galileo: 5, gnss.BeiDou: 7},
		15, 88)
	g := multiGen{
		c: c, sigma: 1, seed: 88,
		sysClock: func(i int) map[gnss.System]float64 {
			// 全段 GAL 有一个大的系统偏差；中间 5 个历元 GAL 不可见
			return map[gnss.System]float64{gnss.Galileo: 90_000}
		},
	}
	epochs := g.epochs(20)
	hide := func(e session.EpochInput) session.EpochInput {
		var kept []session.SatInput
		for _, s := range e.Sats {
			if s.Sys != "GAL" {
				kept = append(kept, s)
			}
		}
		e.Sats = kept
		return e
	}
	for i := 8; i <= 12; i++ {
		epochs[i] = hide(epochs[i])
	}
	svc := session.NewService(profile.Builtins())
	sess, _ := svc.NewSession("s", "enroute")
	var recs []*session.EpochRecord
	for _, e := range epochs {
		r, err := svc.Step(sess, e)
		if err != nil {
			t.Fatal(err)
		}
		recs = append(recs, r)
	}
	for i := 8; i <= 12; i++ {
		if gal := findSys(recs[i], gnss.Galileo); gal != nil {
			t.Fatalf("历元 %d GAL 不可见，不应出现在 systems", i+1)
		}
		if recs[i].Mode == "detected" {
			t.Fatalf("历元 %d 缺 GAL 不应误检", i+1)
		}
	}
	// 重现首历元（13）：GAL 立即参与，其钟差台阶被当历元重新估出
	r := recs[13]
	gal := findSys(r, gnss.Galileo)
	if gal == nil || !gal.Participated || gal.Used != 5 {
		t.Fatalf("GAL 重现首历元应立即 5 星参与: %+v", gal)
	}
	if math.Abs(gal.ClockBias-90_000) > 0.5 {
		t.Fatalf("GAL 钟差应在重现首历元即估到 ~90000, got %v", gal.ClockBias)
	}
	if r.Mode == "detected" {
		t.Fatalf("GAL 重现不应有跨历元瞬态误检, SSE=%.2f", r.SSE)
	}
}
