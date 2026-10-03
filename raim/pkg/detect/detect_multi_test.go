package detect_test

import (
	"math"
	"math/rand"
	"testing"

	"raim/internal/sim"
	"raim/pkg/detect"
	"raim/pkg/geo"
	"raim/pkg/gnss"
	"raim/pkg/lsq"
)

func multiSky9() *sim.Constellation {
	rec := sim.DefaultReceiver()
	return sim.MultiSky(rec, []gnss.System{gnss.GPS, gnss.GAL, gnss.BDS}, 9, 15, 2026)
}

func TestMultiSystemNoFault(t *testing.T) {
	c := multiSky9()
	rng := rand.New(rand.NewSource(77))
	for i := 0; i < 100; i++ {
		a, err := detect.Assess(c.ObserveMulti(sim.MultiObs{Rng: rng}), opts())
		if err != nil {
			t.Fatal(err)
		}
		if a.Mode == detect.ModeDetected {
			t.Fatalf("无故障历元 %d 检出: SSE=%.2f thr=%.2f", i, a.SSE, a.Threshold)
		}
		if len(a.Systems) != 3 {
			t.Fatalf("应三系统参与, got %v", a.Systems)
		}
	}
}

func TestMultiSystemGalileoShiftNotDetected(t *testing.T) {
	// Galileo 全部伪距同加 40 km：只被 Galileo 钟差吸收，不应检出，位置不动。
	c := multiSky9()
	base, err := detect.Assess(c.ObserveMulti(sim.MultiObs{
		Rng: rand.New(rand.NewSource(11)),
	}), opts())
	if err != nil {
		t.Fatal(err)
	}
	shifted, err := detect.Assess(c.ObserveMulti(sim.MultiObs{
		Rng:  rand.New(rand.NewSource(11)),
		GGTO: map[gnss.System]float64{gnss.GAL: 40_000},
	}), opts())
	if err != nil {
		t.Fatal(err)
	}
	if shifted.Mode == detect.ModeDetected {
		t.Fatalf("整系统常数偏差不应检出: SSE=%.2f thr=%.2f", shifted.SSE, shifted.Threshold)
	}
	if d := geo.Norm(geo.Sub(base.Sol.Pos, shifted.Sol.Pos)); d > 1e-6 {
		t.Fatalf("位置应不变, 偏差 %.9f m", d)
	}
	if math.Abs(shifted.Sol.ClockBySys[gnss.GAL]-base.Sol.ClockBySys[gnss.GAL]-40_000) > 1e-5 {
		t.Fatal("Galileo 钟差应整体移动 40 km")
	}
}

func TestMultiSystemExcludesOnlyFaultySystemSat(t *testing.T) {
	// GPS 3 号单星出偏差：只剔 GPS:3，北斗 3 号照常参与。
	c := multiSky9()
	a, err := detect.Assess(c.ObserveMulti(sim.MultiObs{
		Rng:  rand.New(rand.NewSource(5)),
		Bias: map[gnss.SatID]float64{gnss.Key(gnss.GPS, 3): 90},
	}), opts())
	if err != nil {
		t.Fatal(err)
	}
	if a.Mode != detect.ModeExcluded ||
		a.ExcludedSystem != gnss.GPS || a.ExcludedID != 3 {
		t.Fatalf("应排除 GPS:3, got mode=%s %s:%d trials=%+v",
			a.Mode, a.ExcludedSystem, a.ExcludedID, a.Trials)
	}
	// 北斗 3 号在报告解（剔星干净解）里仍然参与；GPS 3 号不在报告解中。
	// SatResults 是全解残差（含被剔星），是否真正参与解算以报告解 a.Sol 为准。
	solSys := solSatSystems(a.Sol)
	if !solSys[gnss.Key(gnss.BDS, 3)] {
		t.Fatal("北斗 3 号应在剔后干净解中照常参与")
	}
	if solSys[gnss.Key(gnss.GPS, 3)] {
		t.Fatal("GPS 3 号应已被剔出报告解")
	}
	// 全解残差表里两颗 3 号星必须按系统分得清（不重号混淆）
	var sawGPS3, sawBDS3 bool
	for _, sr := range a.SatResults {
		if sr.System == gnss.BDS && sr.ID == 3 {
			sawBDS3 = true
		}
		if sr.System == gnss.GPS && sr.ID == 3 {
			sawGPS3 = true
		}
	}
	if !sawGPS3 || !sawBDS3 {
		t.Fatalf("全解残差表应同时含 GPS:3 与 BDS:3, got gps3=%v bds3=%v", sawGPS3, sawBDS3)
	}
	// 报告解（剔后）中 BDS 仍贡献位置
	if !sysIn(a.Sol.Systems, gnss.BDS) {
		t.Fatal("剔后解应仍含北斗系统")
	}
	// 位置回到真值附近
	rec := sim.DefaultReceiver()
	if d := geo.Norm(geo.Sub(a.Sol.Pos, rec.Pos)); d > 10 {
		t.Fatalf("剔后定位偏差 %.3f m", d)
	}
}

func TestMultiSystemExcludesBeiDouNotGPS(t *testing.T) {
	// 反向：偏差在北斗 3 号，只能剔 BDS:3，不能误伤 GPS:3。
	c := multiSky9()
	a, err := detect.Assess(c.ObserveMulti(sim.MultiObs{
		Rng:  rand.New(rand.NewSource(9)),
		Bias: map[gnss.SatID]float64{gnss.Key(gnss.BDS, 3): 90},
	}), opts())
	if err != nil {
		t.Fatal(err)
	}
	if a.Mode != detect.ModeExcluded ||
		a.ExcludedSystem != gnss.BDS || a.ExcludedID != 3 {
		t.Fatalf("应排除 BDS:3, got %s %s:%d", a.Mode, a.ExcludedSystem, a.ExcludedID)
	}
}

func sysIn(ss []gnss.System, s gnss.System) bool {
	for _, x := range ss {
		if x == s {
			return true
		}
	}
	return false
}

func solSatSystems(sol *lsq.Solution) map[gnss.SatID]bool {
	out := map[gnss.SatID]bool{}
	for _, k := range sol.SatIDs {
		out[k] = true
	}
	return out
}
