package session_test

import (
	"bytes"
	"encoding/json"
	"math"
	"math/rand"
	"path/filepath"
	"testing"

	"raim/internal/sim"
	"raim/pkg/apierr"
	"raim/pkg/detect"
	"raim/pkg/gnss"
	"raim/pkg/lsq"
	"raim/pkg/profile"
	"raim/pkg/session"
)

// multiGen 配置一段多模回放数据。
type multiGen struct {
	c      *sim.Constellation
	n      int
	seed   int64
	sigma  float64
	stepAt int // 从该 0-based 历元起给 Galileo 全部伪距加常数（<0 表示无）
	step   float64
	biasAt func(seq0 int) map[gnss.SatID]float64
	ggto   map[gnss.System]float64
	hidden func(seq0 int) []gnss.SatID
}

func genMulti(g multiGen) []session.EpochInput {
	rng := rand.New(rand.NewSource(g.seed))
	out := make([]session.EpochInput, g.n)
	for i := range out {
		ggto := map[gnss.System]float64{}
		for k, v := range g.ggto {
			ggto[k] = v
		}
		if g.stepAt >= 0 && i >= g.stepAt {
			ggto[gnss.GAL] += g.step
		}
		var bias map[gnss.SatID]float64
		if g.biasAt != nil {
			bias = g.biasAt(i)
		}
		var hidden []gnss.SatID
		if g.hidden != nil {
			hidden = g.hidden(i)
		}
		ep := g.c.ObserveMulti(sim.MultiObs{
			Rng: rng, Sigma: g.sigma, GGTO: ggto, Bias: bias, Hidden: hidden,
		})
		out[i] = toMultiInput(int64(i+1), ep)
	}
	return out
}

func toMultiInput(ts int64, ep *lsq.Epoch) session.EpochInput {
	sats := make([]session.SatInput, len(ep.Sats))
	for i, s := range ep.Sats {
		sats[i] = session.SatInput{
			System: s.System, ID: s.ID,
			Pos: [3]float64{s.Pos.X, s.Pos.Y, s.Pos.Z},
			PR:  s.PR, Sigma: s.Sigma,
		}
	}
	return session.EpochInput{Timestamp: ts, Sats: sats}
}

func multiSky() *sim.Constellation {
	return sim.MultiSky(sim.DefaultReceiver(),
		[]gnss.System{gnss.GPS, gnss.GAL, gnss.BDS}, 9, 15, 2026)
}

func customMultiProfile(name string, minIso int) profile.Profile {
	return profile.Profile{
		Name: name, Pfa: 1e-5, Pmd: 1e-3, HAL: 1e9,
		Isolation: profile.Isolation{MinEpochs: minIso},
		Alert:     profile.Alert{Mode: profile.Combined, ConfirmEpochs: 3, ClearEpochs: 3, GrossFactor: 3},
	}
}

func runSteps(t *testing.T, svc *session.Service, sess *session.Session,
	epochs []session.EpochInput) []*session.EpochRecord {
	t.Helper()
	recs := make([]*session.EpochRecord, 0, len(epochs))
	for _, e := range epochs {
		r, err := svc.Step(sess, e)
		if err != nil {
			t.Fatal(err)
		}
		recs = append(recs, r)
	}
	return recs
}

// 验收 1：Galileo 全部伪距同加常数——位置不变；逐历元自由估计（白模型）
// 对台阶即时吸收，第一个台阶历元即达稳态，无跨历元过渡。
func TestMultiSystemGalileoStepPositionInvariant(t *testing.T) {
	c := multiSky()
	const step = 60_000.0
	const stepAt = 10
	n := 20

	base := genMulti(multiGen{c: c, n: n, seed: 42, sigma: 1})
	stepped := genMulti(multiGen{
		c: c, n: n, seed: 42, sigma: 1, stepAt: stepAt, step: step,
	})

	prof := customMultiProfile("m", 5)
	svc := session.NewService([]profile.Profile{prof})
	sessBase, _ := svc.NewSession("b", "m")
	sessStep, _ := svc.NewSession("s", "m")
	rb := runSteps(t, svc, sessBase, base)
	rs := runSteps(t, svc, sessStep, stepped)

	for i := 0; i < n; i++ {
		d := norm3(rb[i].Position.ECEF, rs[i].Position.ECEF)
		if d > 1e-6 {
			t.Fatalf("历元 %d 位置随 Galileo 台阶移动 %.9f m（应严格不变）", i+1, d)
		}
	}

	// 台阶前：Galileo 钟差两会话一致（噪声相同，GGTO 均为 0）
	for i := 0; i < stepAt; i++ {
		cb := clockOf(rb[i], gnss.GAL)
		cs := clockOf(rs[i], gnss.GAL)
		if math.Abs(cb-cs) > 1e-6 {
			t.Fatalf("台阶前历元 %d Galileo 钟差应一致: %v vs %v", i+1, cb, cs)
		}
	}
	// 台阶出现的第一个历元（seq=stepAt+1）即完全吸收：
	// Galileo 钟差较基线恰好 +step，且该历元不检出、不告警。
	i := stepAt
	cb := clockOf(rb[i], gnss.GAL)
	cs := clockOf(rs[i], gnss.GAL)
	if math.Abs((cs-cb)-step) > 1e-6 {
		t.Fatalf("台阶首历元 Galileo 钟差应即时 +%v，实际增量 %v", step, cs-cb)
	}
	if rs[i].Mode == "detected" || rs[i].Alert {
		t.Fatalf("台阶首历元不应检出/告警: mode=%s alert=%v sse=%.2f thr=%.2f",
			rs[i].Mode, rs[i].Alert, rs[i].SSE, rs[i].Threshold)
	}
	// 稳态：台阶后每一历元增量都恒为 step（无收敛过程，0 个过渡历元）
	for j := stepAt; j < n; j++ {
		dClock := clockOf(rs[j], gnss.GAL) - clockOf(rb[j], gnss.GAL)
		if math.Abs(dClock-step) > 1e-6 {
			t.Fatalf("台阶后历元 %d Galileo 钟差增量 %v ≠ %v（存在跨历元过渡）",
				j+1, dClock, step)
		}
	}
	// 每历元都能看到三套系统各自的钟差估计与参与情况
	sc := rs[stepAt].SystemClocks
	if len(sc) != 3 {
		t.Fatalf("应输出三套系统钟差, got %d", len(sc))
	}
	for _, s := range sc {
		if !s.Used || s.SatCount != 9 {
			t.Fatalf("%s 本历元应参与且 9 颗星: %+v", s.System, s)
		}
	}
	if clockOf(rs[stepAt], gnss.GPS) == 0 && clockOf(rs[stepAt], gnss.BDS) == 0 {
		// 仅提示性：GPS/BDS 钟差受噪声影响一般不恰好为 0，这里不做硬断言
	}
}

// 验收 2：GPS 3 号单独出偏差，只剔 GPS:3，北斗 3 号照常参与。
func TestMultiSystemGPS3ExcludedBDS3Kept(t *testing.T) {
	c := multiSky()
	prof := customMultiProfile("m", 5)
	svc := session.NewService([]profile.Profile{prof})
	sess, _ := svc.NewSession("s", "m")

	epochs := genMulti(multiGen{
		c: c, n: 12, seed: 5, sigma: 1,
		biasAt: func(i int) map[gnss.SatID]float64 {
			if i >= 2 && i <= 8 {
				return map[gnss.SatID]float64{gnss.Key(gnss.GPS, 3): 90}
			}
			return nil
		},
	})
	recs := runSteps(t, svc, sess, epochs)

	r := recs[2]
	if r.Mode != "excluded" || r.ExcludedID != 3 || r.ExcludedSystem != gnss.GPS {
		t.Fatalf("应排除 GPS:3, got mode=%s %s:%d", r.Mode, r.ExcludedSystem, r.ExcludedID)
	}
	// 隔离记录必须带系统
	if !containsRef(r.Isolated, gnss.GPS, 3) {
		t.Fatalf("隔离列表应含 GPS:3: %v", r.Isolated)
	}
	if containsRef(r.Isolated, gnss.BDS, 3) {
		t.Fatal("绝不能把北斗 3 号隔离")
	}
	// 故障期间：北斗 3 号始终在解里；GPS 3 号从“排除的下一历元”起被隔离，
	// 不再参与解算（排除当历元采用的是剔星干净解，故当历元也不参与）。
	for i := 2; i <= 8; i++ {
		if !satParticipates(recs[i].SatResults, gnss.BDS, 3) {
			t.Fatalf("历元 %d 北斗 3 号应照常参与", i+1)
		}
		if i == 2 {
			// 排除当历元：SatResults 是全解残差（用于留痕），含被剔星，
			// 但位置解已是剔星解。通过隔离列表确认它被唯一锁定。
			if !containsRef(recs[i].Isolated, gnss.GPS, 3) {
				t.Fatalf("排除当历元 GPS:3 应进入隔离")
			}
			continue
		}
		if satParticipates(recs[i].SatResults, gnss.GPS, 3) {
			t.Fatalf("历元 %d GPS 3 号应已被隔离、不参与解算", i+1)
		}
	}
}

// 验收 3a：同一段多模数据，整段批量 vs 逐历元，每一项相同（含系统钟差、隔离）。
func TestMultiSystemBatchEqualsIndividual(t *testing.T) {
	c := multiSky()
	epochs := genMulti(multiGen{
		c: c, n: 40, seed: 13, sigma: 1,
		stepAt: 15, step: 30_000, // 中途 Galileo 时间台阶
		biasAt: func(i int) map[gnss.SatID]float64 {
			if i >= 6 && i <= 16 {
				return map[gnss.SatID]float64{gnss.Key(gnss.BDS, 5): 90}
			}
			return nil
		},
	})

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

	if len(recsA) != len(recsB) {
		t.Fatalf("记录数 %d vs %d", len(recsA), len(recsB))
	}
	for i := range recsA {
		ba, _ := json.Marshal(recsA[i])
		bb, _ := json.Marshal(recsB[i])
		if !bytes.Equal(ba, bb) {
			t.Fatalf("历元 %d 批量与逐历元不一致:\nA=%s\nB=%s", i, ba, bb)
		}
	}
	assertStateEqual(t, sessA, sessB)
}

// 验收 3b：跑到中途重启服务再续上，与不停机跑完没有差别（含所有跨历元状态）。
func TestMultiSystemRestartResumeEqualsContinuous(t *testing.T) {
	c := multiSky()
	epochs := genMulti(multiGen{
		c: c, n: 36, seed: 17, sigma: 1,
		stepAt: 12, step: 45_000,
		biasAt: func(i int) map[gnss.SatID]float64 {
			if i >= 4 && i <= 20 {
				return map[gnss.SatID]float64{gnss.Key(gnss.GAL, 2): 90}
			}
			return nil
		},
	})

	dirFull := filepath.Join(t.TempDir(), "dataFull")
	stFull, err := session.NewStore(dirFull)
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

	dir := filepath.Join(t.TempDir(), "data")
	st1, err := session.NewStore(dir)
	if err != nil {
		t.Fatal(err)
	}
	if _, err := st1.CreateSession("s", "enroute"); err != nil {
		t.Fatal(err)
	}
	const split = 15
	for _, e := range epochs[:split] {
		if _, err := st1.AppendEpoch("s", e); err != nil {
			t.Fatal(err)
		}
	}
	st2, err := session.NewStore(dir) // 重启：重新从磁盘加载
	if err != nil {
		t.Fatal(err)
	}
	for _, e := range epochs[split:] {
		if _, err := st2.AppendEpoch("s", e); err != nil {
			t.Fatal(err)
		}
	}
	sessResumed, _ := st2.GetSession("s")

	if len(sessResumed.Records) != len(sessFull.Records) {
		t.Fatalf("记录数 %d vs %d", len(sessResumed.Records), len(sessFull.Records))
	}
	for i := range sessFull.Records {
		ba, _ := json.Marshal(sessFull.Records[i])
		bb, _ := json.Marshal(sessResumed.Records[i])
		if !bytes.Equal(ba, bb) {
			t.Fatalf("历元 %d 重启续跑与连续跑完不一致:\nA=%s\nB=%s", i, ba, bb)
		}
	}
	assertStateEqual(t, sessResumed, sessFull)
}

// 验收 6a：系统标识不认识被拒，指出字段与历元下标。
func TestMultiSystemUnknownSystemRejected(t *testing.T) {
	c := multiSky()
	epochs := genMulti(multiGen{c: c, n: 4, seed: 1, sigma: 1})
	// 单历元提交：字段精确到 satellites[i].system
	bad := epochs[1]
	bad.Timestamp = 400
	bad.Sats[2].System = gnss.System("XXX")
	st := newStore(t)
	if _, err := st.CreateSession("s", "enroute"); err != nil {
		t.Fatal(err)
	}
	_, err := st.AppendEpoch("s", bad)
	fe, ok := err.(*apierr.FieldError)
	if !ok || fe.Field != "satellites[2].system" {
		t.Fatalf("单历元错误字段应为 satellites[2].system, got %v", err)
	}
	// 批量提交：带 epochs[k]. 前缀
	_, err = st.AppendBatch("s", epochs[:3])
	fe, ok = err.(*apierr.FieldError)
	if !ok || fe.Field != "epochs[1].satellites[2].system" {
		t.Fatalf("批量错误字段应为 epochs[1].satellites[2].system, got %v", err)
	}
}

// 验收 6b：同系统重号被拒；跨系统同号不拒。
func TestMultiSystemDuplicateWithinSystemRejected(t *testing.T) {
	c := multiSky()
	epochs := genMulti(multiGen{c: c, n: 3, seed: 2, sigma: 1})

	dup := epochs[0]
	dup.Timestamp = 500
	dup.Sats[1].System = dup.Sats[0].System
	dup.Sats[1].ID = dup.Sats[0].ID
	st := newStore(t)
	if _, err := st.CreateSession("s", "enroute"); err != nil {
		t.Fatal(err)
	}
	_, err := st.AppendEpoch("s", dup)
	fe, ok := err.(*apierr.FieldError)
	if !ok || fe.Field != "satellites[1].id" {
		t.Fatalf("同系统重号应指向 satellites[1].id, got %v", err)
	}

	// 跨系统同号合法：用一个三系统各 3 颗的小星座，三系统都含 7 号星
	rec := sim.DefaultReceiver()
	c3 := sim.MultiSky(rec, []gnss.System{gnss.GPS, gnss.GAL, gnss.BDS}, 8, 20, 99)
	small := genMulti(multiGen{c: c3, n: 1, seed: 1, sigma: 1})
	okEpoch := small[0]
	okEpoch.Timestamp = 501
	okEpoch.Sats[0].ID = 9
	okEpoch.Sats[8].ID = 9
	okEpoch.Sats[16].ID = 9
	if _, err := st.AppendEpoch("s", okEpoch); err != nil {
		t.Fatalf("跨系统同号不应报错: %v", err)
	}
}

// 无系统标识的星按 GPS 处理（升级前数据直接接着提交）。
func TestMissingSystemDefaultsToGPS(t *testing.T) {
	c := sim.EvenSky(sim.DefaultReceiver(), 9, 15, 5)
	epochs := genEpochs(c, 3, 1, 9, nil) // 旧构造路径：SatInput 不带 system
	for _, e := range epochs {
		for _, s := range e.Sats {
			if s.System != "" {
				t.Fatalf("旧路径不应带系统标识, got %q", s.System)
			}
		}
	}
	st := newStore(t)
	if _, err := st.CreateSession("s", "enroute"); err != nil {
		t.Fatal(err)
	}
	r, err := st.AppendEpoch("s", epochs[0])
	if err != nil {
		t.Fatal(err)
	}
	if r.Mode != "ok" {
		t.Fatalf("无标识星按 GPS 应正常定位, mode=%s", r.Mode)
	}
	for _, sc := range r.SystemClocks {
		wantUsed := sc.System == gnss.GPS
		if sc.Used != wantUsed {
			t.Fatalf("只有 GPS 应参与: %+v", sc)
		}
	}
}

// 断续可见：某系统消失若干历元再回来，白模型下回来首历元即正常（无热身）。
func TestMultiSystemIntermittentVisibility(t *testing.T) {
	c := multiSky()
	prof := customMultiProfile("m", 3)
	svc := session.NewService([]profile.Profile{prof})
	sess, _ := svc.NewSession("s", "m")

	epochs := genMulti(multiGen{
		c: c, n: 12, seed: 21, sigma: 1,
		hidden: func(i int) []gnss.SatID {
			if i >= 4 && i <= 7 {
				// 历元 5~8：Galileo 全部不可见
				var hs []gnss.SatID
				for id := 1; id <= 9; id++ {
					hs = append(hs, gnss.Key(gnss.GAL, id))
				}
				return hs
			}
			return nil
		},
	})
	recs := runSteps(t, svc, sess, epochs)

	// Galileo 不可见期间：used=false
	for i := 4; i <= 7; i++ {
		sc := clockRec(recs[i], gnss.GAL)
		if sc.Used {
			t.Fatalf("历元 %d Galileo 不可见不应参与", i+1)
		}
	}
	// 回来的第一个历元：立即参与，定位正常（白估计不需要重新热身）
	back := recs[8]
	sc := clockRec(back, gnss.GAL)
	if !sc.Used || sc.SatCount != 9 {
		t.Fatalf("Galileo 回来首历元应立即参与: %+v", sc)
	}
	if back.Mode == "detected" {
		t.Fatalf("回来首历元不应检出: SSE=%.2f", back.SSE)
	}
}

func clockOf(r *session.EpochRecord, sys gnss.System) float64 {
	return clockRec(r, sys).ClockBias
}

func clockRec(r *session.EpochRecord, sys gnss.System) session.SystemClock {
	for _, sc := range r.SystemClocks {
		if sc.System == sys {
			return sc
		}
	}
	return session.SystemClock{System: sys}
}

func satParticipates(rs []detect.SatResult, sys gnss.System, id int) bool {
	for _, r := range rs {
		if r.System == sys && r.ID == id {
			return true
		}
	}
	return false
}
