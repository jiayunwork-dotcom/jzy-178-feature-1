package protect_test

import (
	"testing"

	"raim/internal/sim"
	"raim/pkg/chisq"
	"raim/pkg/detect"
	"raim/pkg/gnss"
	"raim/pkg/lsq"
	"raim/pkg/protect"
)

// hplFor 直接对一个几何解算 HPL（用全解、固定阈值）。
func hplFor(t *testing.T, ep *lsq.Epoch, pfa, pmd float64) float64 {
	t.Helper()
	sol, err := lsq.Solve(ep)
	if err != nil {
		t.Fatal(err)
	}
	_, dof := lsq.WeightedSSE(sol.Resid, sol.Sigma)
	thr := chisq.Threshold(dof, pfa)
	return protect.HPL(sol, dof, thr, pmd)
}

func TestHPLPositiveAndGeometric(t *testing.T) {
	rec := sim.DefaultReceiver()
	good := sim.BuildConstellation(rec, []sim.SatelliteDef{
		{ID: 1, Azimuth: 0, Elev: 25}, {ID: 2, Azimuth: 45, Elev: 70},
		{ID: 3, Azimuth: 90, Elev: 30}, {ID: 4, Azimuth: 135, Elev: 65},
		{ID: 5, Azimuth: 180, Elev: 35}, {ID: 6, Azimuth: 225, Elev: 72},
		{ID: 7, Azimuth: 270, Elev: 28}, {ID: 8, Azimuth: 315, Elev: 60},
	})
	bad := sim.BuildConstellation(rec, []sim.SatelliteDef{
		{ID: 1, Azimuth: 0, Elev: 25}, {ID: 2, Azimuth: 8, Elev: 70},
		{ID: 3, Azimuth: 16, Elev: 30}, {ID: 4, Azimuth: 24, Elev: 65},
		{ID: 5, Azimuth: 32, Elev: 35}, {ID: 6, Azimuth: 40, Elev: 72},
		{ID: 7, Azimuth: 48, Elev: 28}, {ID: 8, Azimuth: 56, Elev: 60},
	})
	hGood := hplFor(t, good.Observe(sim.Obs{}), 1e-5, 1e-3)
	hBad := hplFor(t, bad.Observe(sim.Obs{}), 1e-5, 1e-3)
	if !(hGood > 0 && hBad > hGood) {
		t.Fatalf("几何越差 HPL 应越大: good=%v bad=%v (HDOP 对比见 lsq 测试)", hGood, hBad)
	}
}

func TestHPLExclusionUsesTighterGeometry(t *testing.T) {
	// 排除后星少、几何略差，但 HPL 仍为正且有限
	rec := sim.DefaultReceiver()
	c := sim.EvenSky(rec, 9, 15, 5)
	a, err := detect.Assess(c.Observe(sim.Obs{Bias: map[int]float64{4: 60}}),
		detect.Options{Pfa: 1e-5})
	if err != nil {
		t.Fatal(err)
	}
	if a.Mode != detect.ModeExcluded {
		t.Fatalf("期望排除, got %v", a.Mode)
	}
	h := protect.AssessmentHPL(a, 1e-3)
	if h <= 0 {
		t.Fatalf("排除后 HPL 应为正: %v", h)
	}
}

func TestHPLZeroWhenNoRedundancy(t *testing.T) {
	rec := sim.DefaultReceiver()
	c := sim.EvenSky(rec, 4, 20, 1)
	a, err := detect.Assess(c.Observe(sim.Obs{}), detect.Options{Pfa: 1e-5})
	if err != nil {
		t.Fatal(err)
	}
	if h := protect.AssessmentHPL(a, 1e-3); h != 0 {
		t.Fatalf("无冗余时 HPL 应为 0（不可用）, got %v", h)
	}
}

func TestHPLMultiSystem(t *testing.T) {
	// 多模（3 系统、各 9 颗）：HPL 为正、有限；排除后仍为正。
	rec := sim.DefaultReceiver()
	c := sim.MultiSky(rec, []gnss.System{gnss.GPS, gnss.GAL, gnss.BDS}, 9, 15, 2026)
	ep := c.ObserveMulti(sim.MultiObs{Sigma: 1})
	sol, err := lsq.Solve(ep)
	if err != nil {
		t.Fatal(err)
	}
	sse, dof := lsq.WeightedSSEDOF(sol.Resid, sol.Sigma, 3+len(sol.Systems))
	_ = sse
	thr := chisq.Threshold(dof, 1e-5)
	h := protect.HPL(sol, dof, thr, 1e-3)
	if !(h > 0 && h < 1e6) {
		t.Fatalf("多模 HPL 应为正且有限, got %v (dof=%d)", h, dof)
	}

	// 单星故障排除后的 HPL 仍为正
	a, err := detect.Assess(c.ObserveMulti(sim.MultiObs{
		Bias: map[gnss.SatID]float64{gnss.Key(gnss.GPS, 4): 90},
	}), detect.Options{Pfa: 1e-5})
	if err != nil {
		t.Fatal(err)
	}
	if a.Mode != detect.ModeExcluded {
		t.Fatalf("应排除 GPS:4, got %v", a.Mode)
	}
	if he := protect.AssessmentHPL(a, 1e-3); !(he > 0) {
		t.Fatalf("多模排除后 HPL 应为正, got %v", he)
	}
}
