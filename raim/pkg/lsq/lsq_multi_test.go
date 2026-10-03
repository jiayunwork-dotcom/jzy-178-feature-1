package lsq_test

import (
	"math"
	"testing"

	"raim/internal/sim"
	"raim/pkg/apierr"
	"raim/pkg/geo"
	"raim/pkg/gnss"
	"raim/pkg/lsq"
)

func multiSky() *sim.Constellation {
	rec := sim.DefaultReceiver()
	return sim.MultiSky(rec, []gnss.System{gnss.GPS, gnss.GAL, gnss.BDS}, 9, 15, 2026)
}

func TestMultiSystemNoiselessPositionAndClocks(t *testing.T) {
	c := multiSky()
	ggto := map[gnss.System]float64{gnss.GAL: 12_345.6, gnss.BDS: -23_456.7}
	ep := c.ObserveMulti(sim.MultiObs{GGTO: ggto})
	sol, err := lsq.Solve(ep)
	if err != nil {
		t.Fatal(err)
	}
	rec := sim.DefaultReceiver()
	if d := dist(sol.Pos, rec.Pos); d > 0.01 {
		t.Fatalf("多模无噪声定位偏差 %.4f m", d)
	}
	// 基准钟差（GPS）应为 0（接收机零钟差）
	if math.Abs(sol.ClockBias) > 1e-6 {
		t.Fatalf("GPS 基准钟差应为 0, got %v", sol.ClockBias)
	}
	for sys, want := range ggto {
		got := sol.ClockBySys[sys]
		if math.Abs(got-want) > 1e-6 {
			t.Fatalf("%s 系统间偏差估计 %.6f, 期望 %.6f", sys, got, want)
		}
	}
	// 三个系统都应在时钟列里，GPS 为基准
	if len(sol.Systems) != 3 || sol.RefSystem != gnss.GPS {
		t.Fatalf("参与系统=%v ref=%v", sol.Systems, sol.RefSystem)
	}
}

func TestMultiSystemConstantShiftIsPositionInvariant(t *testing.T) {
	// Galileo 全部伪距同加常数：位置不变，只移动 Galileo 钟差估计。
	c := multiSky()
	const step = 50_000.0
	base := c.ObserveMulti(sim.MultiObs{})
	shifted := c.ObserveMulti(sim.MultiObs{GGTO: map[gnss.System]float64{gnss.GAL: step}})
	s1, err := lsq.Solve(base)
	if err != nil {
		t.Fatal(err)
	}
	s2, err := lsq.Solve(shifted)
	if err != nil {
		t.Fatal(err)
	}
	if d := dist(s1.Pos, s2.Pos); d > 1e-6 {
		t.Fatalf("Galileo 整体加常数后位置变化 %.9f m", d)
	}
	if math.Abs((s2.ClockBySys[gnss.GAL]-s1.ClockBySys[gnss.GAL])-step) > 1e-6 {
		t.Fatalf("Galileo 钟差变化 %v, 期望 %v",
			s2.ClockBySys[gnss.GAL]-s1.ClockBySys[gnss.GAL], step)
	}
	// GPS/北斗钟差与残差不受影响
	for _, sys := range []gnss.System{gnss.GPS, gnss.BDS} {
		if math.Abs(s2.ClockBySys[sys]-s1.ClockBySys[sys]) > 1e-6 {
			t.Fatalf("%s 钟差不应随 Galileo 台阶移动", sys)
		}
	}
}

func TestMultiSystemDOF(t *testing.T) {
	c := multiSky()
	ep := c.ObserveMulti(sim.MultiObs{})
	sol, err := lsq.Solve(ep)
	if err != nil {
		t.Fatal(err)
	}
	_, dof := lsq.WeightedSSEDOF(sol.Resid, sol.Sigma, 3+len(sol.Systems))
	if dof != 27-6 {
		t.Fatalf("27 星 3 系统 dof 应为 21, got %d", dof)
	}
}

func TestMultiSystemValidation(t *testing.T) {
	c := multiSky()
	ep := c.ObserveMulti(sim.MultiObs{})

	// 未知系统标识
	e := cloneEpoch(ep)
	e.Sats[0].System = gnss.System("XXX")
	_, err := lsq.Solve(e)
	fe, ok := err.(*apierr.FieldError)
	if !ok || fe.Field != "satellites[0].system" {
		t.Fatalf("未知系统应指向 satellites[0].system, got %v", err)
	}

	// 同系统重号
	e = cloneEpoch(ep)
	e.Sats[1].System = e.Sats[0].System
	e.Sats[1].ID = e.Sats[0].ID
	_, err = lsq.Solve(e)
	if fe, ok = err.(*apierr.FieldError); !ok || fe.Field != "satellites[1].id" {
		t.Fatalf("同系统重号应指向 satellites[1].id, got %v", err)
	}

	// 跨系统同号合法：构造一个干净的小星座，三系统各有 3 号星，互不冲突
	rec := sim.DefaultReceiver()
	c3 := sim.BuildMultiConstellation(rec, []sim.MultiSatDef{
		{System: gnss.GPS, ID: 1, Azimuth: 10, Elev: 40},
		{System: gnss.GPS, ID: 2, Azimuth: 100, Elev: 30},
		{System: gnss.GPS, ID: 3, Azimuth: 200, Elev: 45},
		{System: gnss.GAL, ID: 1, Azimuth: 300, Elev: 50},
		{System: gnss.GAL, ID: 2, Azimuth: 50, Elev: 60},
		{System: gnss.GAL, ID: 3, Azimuth: 150, Elev: 35},
		{System: gnss.BDS, ID: 1, Azimuth: 250, Elev: 55},
		{System: gnss.BDS, ID: 2, Azimuth: 350, Elev: 42},
		{System: gnss.BDS, ID: 3, Azimuth: 70, Elev: 48},
	})
	if _, err := lsq.Solve(c3.ObserveMulti(sim.MultiObs{})); err != nil {
		t.Fatalf("三系统同有 3 号星不应报错: %v", err)
	}
}

func TestMultiSystemUnknownsNeedMoreSats(t *testing.T) {
	rec := sim.DefaultReceiver()
	// 4 颗星分属 2 个系统：3+2=5 个未知量，4<5，不能定位
	c := sim.BuildMultiConstellation(rec, []sim.MultiSatDef{
		{System: gnss.GPS, ID: 1, Azimuth: 0, Elev: 30},
		{System: gnss.GPS, ID: 2, Azimuth: 90, Elev: 40},
		{System: gnss.GPS, ID: 3, Azimuth: 180, Elev: 35},
		{System: gnss.GAL, ID: 1, Azimuth: 270, Elev: 45},
	})
	_, err := lsq.Solve(c.ObserveMulti(sim.MultiObs{}))
	if fe, ok := err.(*apierr.FieldError); !ok || fe.Field != "satellites" {
		t.Fatalf("未知量多于星数应指向 satellites, got %v", err)
	}
}

func dist(a, b geo.Vec) float64 {
	return geo.Norm(geo.Sub(a, b))
}

func cloneEpoch(ep *lsq.Epoch) *lsq.Epoch {
	sats := make([]lsq.Satellite, len(ep.Sats))
	copy(sats, ep.Sats)
	return &lsq.Epoch{Approx: ep.Approx, Sats: sats}
}
