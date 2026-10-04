package detect_test

import (
	"math"
	"testing"

	"raim/internal/sim"
	"raim/pkg/detect"
	"raim/pkg/geo"
	"raim/pkg/gnss"
)

func multiSky() *sim.Constellation {
	rec := sim.DefaultReceiver()
	return sim.MultiSky(rec,
		map[gnss.System]int{gnss.GPS: 7, gnss.Galileo: 7, gnss.BeiDou: 7},
		15, 4242)
}

func TestMultiSystemNoFaultOK(t *testing.T) {
	c := multiSky()
	a, err := detect.Assess(c.Observe(sim.Obs{}), opts())
	if err != nil {
		t.Fatal(err)
	}
	if a.Mode == detect.ModeDetected {
		t.Fatalf("无故障不应检出: SSE=%.2f thr=%.2f", a.SSE, a.Threshold)
	}
	if a.DOF != 21-(3+3) {
		t.Fatalf("三系统 DOF 应为 15, got %d", a.DOF)
	}
}

func TestGalileoSystemWideConstantNotDetected(t *testing.T) {
	// Galileo 全部伪距同加常数：被 GAL 钟差列吸收，位置不变、不检出
	c := multiSky()
	before, err := detect.Assess(c.Observe(sim.Obs{}), opts())
	if err != nil {
		t.Fatal(err)
	}
	after, err := detect.Assess(c.Observe(sim.Obs{
		SysClock: map[gnss.System]float64{gnss.Galileo: 500_000},
	}), opts())
	if err != nil {
		t.Fatal(err)
	}
	if after.Mode == detect.ModeDetected || after.Mode == detect.ModeExcluded {
		t.Fatalf("整系统常数台阶不应判故障: mode=%s SSE=%.4f thr=%.2f",
			after.Mode, after.SSE, after.Threshold)
	}
	if d := geo.Norm(geo.Sub(before.Sol.Pos, after.Sol.Pos)); d > 1e-4 {
		t.Fatalf("位置变化 %.6f m，整系统偏差应完全被钟差吸收", d)
	}
	g1, _ := before.Sol.ClockOf(gnss.Galileo)
	g2, _ := after.Sol.ClockOf(gnss.Galileo)
	if math.Abs((g2-g1)-500_000) > 1e-2 {
		t.Fatalf("GAL 钟差台阶 %v, 期望 500000", g2-g1)
	}
}

func TestSingleGPS3BiasExcludesOnlyGPS3NotBDS3(t *testing.T) {
	c := multiSky()
	const bias = 120.0
	a, err := detect.Assess(c.Observe(sim.Obs{
		BiasK: map[gnss.SatKey]float64{gnss.Key(gnss.GPS, 3): bias},
	}), opts())
	if err != nil {
		t.Fatal(err)
	}
	if a.Mode != detect.ModeExcluded {
		t.Fatalf("应能唯一排除, got mode=%s SSE=%.1f thr=%.1f trials=%d",
			a.Mode, a.SSE, a.Threshold, len(a.Trials))
	}
	if a.ExcludedID != 3 || a.ExcludedSys != gnss.System("") {
		t.Fatalf("被剔的应是 GPS 3 号（JSON 省略 sys），got %s:%d",
			a.ExcludedSys, a.ExcludedID)
	}
	if k := a.ExcludedKey(); k != gnss.Key(gnss.GPS, 3) {
		t.Fatalf("排除主键应 GPS:3, got %s", k)
	}
	// 北斗 3 号必须仍在采用的干净解里、照常参与
	bds3InActive := false
	for _, sr := range a.SatResults {
		if sr.Key() == gnss.Key(gnss.BeiDou, 3) {
			bds3InActive = true
		}
	}
	if !bds3InActive {
		t.Fatal("北斗 3 号应照常参与解算，未在卫星结果中找到")
	}
	// 剔后位置回到真值附近
	if d := geo.Norm(geo.Sub(a.Sol.Pos, c.TruePos())); d > 10 {
		t.Fatalf("剔后定位偏差 %.3f m", d)
	}
}

func TestSingleBDS3BiasExcludesOnlyBDS3NotGPS3(t *testing.T) {
	// 反向验证：坏的是北斗 3 号，GPS 3 号不受牵连
	c := multiSky()
	a, err := detect.Assess(c.Observe(sim.Obs{
		BiasK: map[gnss.SatKey]float64{gnss.Key(gnss.BeiDou, 3): 120},
	}), opts())
	if err != nil {
		t.Fatal(err)
	}
	if a.Mode != detect.ModeExcluded {
		t.Fatalf("应能唯一排除 BDS:3, got mode=%s", a.Mode)
	}
	if a.ExcludedKey() != gnss.Key(gnss.BeiDou, 3) {
		t.Fatalf("被剔的应是 BDS:3, got %s:%d", a.ExcludedSys, a.ExcludedID)
	}
	if a.ExcludedSys != gnss.BeiDou {
		t.Fatalf("excluded_sys 应为 BDS, got %q", a.ExcludedSys)
	}
	gps3Active := false
	for _, sr := range a.SatResults {
		if sr.Key() == gnss.Key(gnss.GPS, 3) {
			gps3Active = true
		}
	}
	if !gps3Active {
		t.Fatal("GPS 3 号应照常参与解算")
	}
}

func TestSatResultsKeyedBySystem(t *testing.T) {
	c := multiSky()
	a, err := detect.Assess(c.Observe(sim.Obs{}), opts())
	if err != nil {
		t.Fatal(err)
	}
	count := map[gnss.System]int{}
	for _, sr := range a.SatResults {
		count[sr.Key().Sys]++
	}
	if count[gnss.GPS] != 7 || count[gnss.Galileo] != 7 || count[gnss.BeiDou] != 7 {
		t.Fatalf("每系统每星结果数: %v", count)
	}
	// GPS 星的 sys JSON 字段省略（升级前兼容）
	for _, sr := range a.SatResults {
		if sr.Sys == gnss.GPS && sr.Sys != "" {
			t.Fatal("GPS 星 sys 应留空")
		}
	}
}

func TestTwoSystemMinimumRedundancy(t *testing.T) {
	rec := sim.DefaultReceiver()
	// 2 系统各 3 颗 = 6 颗，p=5，dof=1：可检测；剔任一星后 dof=0，不能排除
	c := sim.MultiSky(rec, map[gnss.System]int{gnss.GPS: 3, gnss.Galileo: 3}, 20, 9)
	a, err := detect.Assess(c.Observe(sim.Obs{
		BiasK: map[gnss.SatKey]float64{gnss.Key(gnss.GPS, 1): 100},
	}), detect.Options{Pfa: 1e-3})
	if err != nil {
		t.Fatal(err)
	}
	if a.Mode != detect.ModeDetected {
		t.Fatalf("6 星 2 系统有 1 个冗余可检出, got %s (dof=%d)", a.Mode, a.DOF)
	}
	if a.ExcludedID != 0 {
		t.Fatal("剔后无冗余，不应排除")
	}
}
