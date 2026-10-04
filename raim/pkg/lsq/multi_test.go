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

// multiConstellation 构造三系统各 6 颗共视星（几何充分好）。
func multiConstellation() *sim.Constellation {
	rec := sim.DefaultReceiver()
	return sim.MultiSky(rec,
		map[gnss.System]int{gnss.GPS: 6, gnss.Galileo: 6, gnss.BeiDou: 6},
		15, 20261004)
}

func TestMultiSystemNoiselessPosition(t *testing.T) {
	c := multiConstellation()
	// 无噪声、系统时偏差各取不同常数
	ep := c.Observe(sim.Obs{
		SysClock: map[gnss.System]float64{
			gnss.GPS:     0,
			gnss.Galileo: 45_000, // GPST-GST 量级（任意常数）
			gnss.BeiDou:  -12_000,
		},
	})
	sol, err := lsq.Solve(ep)
	if err != nil {
		t.Fatal(err)
	}
	if d := geo.Norm(geo.Sub(sol.Pos, c.TruePos())); d > 0.01 {
		t.Fatalf("三系统无噪声定位偏差 %.4f m", d)
	}
	if len(sol.Systems) != 3 || sol.NPars != 6 {
		t.Fatalf("应有 3 系统 6 参数, got systems=%v p=%d", sol.Systems, sol.NPars)
	}
	if math.Abs(sol.Clocks[0]-0) > 1e-6 {
		t.Fatalf("GPS 钟差应=0, got %v", sol.Clocks[0])
	}
	if math.Abs(sol.Clocks[1]-45_000) > 1e-3 {
		t.Fatalf("GAL 钟差应=45000, got %v", sol.Clocks[1])
	}
	if math.Abs(sol.Clocks[2]-(-12_000)) > 1e-3 {
		t.Fatalf("BDS 钟差应=-12000, got %v", sol.Clocks[2])
	}
	// DOF = 18 - 6 = 12
	if sse, dof := sol.SSE(); math.Abs(sse) > 1e-12 || dof != 12 {
		t.Fatalf("无噪声 SSE/dof = %v/%d", sse, dof)
	}
}

func TestSystemWideConstantAbsorbedByClock(t *testing.T) {
	// Galileo 全部伪距同加常数：位置不变，只 GAL 钟差列平移
	c := multiConstellation()
	base := c.Observe(sim.Obs{})
	shift := 300_000.0
	shifted := c.Observe(sim.Obs{
		SysClock: map[gnss.System]float64{gnss.Galileo: shift},
	})
	s1, err := lsq.Solve(base)
	if err != nil {
		t.Fatal(err)
	}
	s2, err := lsq.Solve(shifted)
	if err != nil {
		t.Fatal(err)
	}
	if d := geo.Norm(geo.Sub(s1.Pos, s2.Pos)); d > 1e-5 {
		t.Fatalf("Galileo 整系统常数台阶后位置变化 %.6f m", d)
	}
	g1, _ := s1.ClockOf(gnss.Galileo)
	g2, _ := s2.ClockOf(gnss.Galileo)
	if math.Abs((g2-g1)-shift) > 1e-4 {
		t.Fatalf("GAL 钟差变化 %v, 期望 %v", g2-g1, shift)
	}
	// GPS/BDS 钟差不变
	gps1, _ := s1.ClockOf(gnss.GPS)
	gps2, _ := s2.ClockOf(gnss.GPS)
	b1, _ := s1.ClockOf(gnss.BeiDou)
	b2, _ := s2.ClockOf(gnss.BeiDou)
	if math.Abs(gps2-gps1) > 1e-4 || math.Abs(b2-b1) > 1e-4 {
		t.Fatalf("其他系统钟差不应改变: GPS %v->%v BDS %v->%v", gps1, gps2, b1, b2)
	}
}

func TestCrossSystemSameIDCoexist(t *testing.T) {
	// GPS 3 号与北斗 3 号必须同时进入解算
	c := multiConstellation()
	ep := c.Observe(sim.Obs{})
	sol, err := lsq.Solve(ep)
	if err != nil {
		t.Fatal(err)
	}
	seenGPS3, seenBDS3 := false, false
	for i, s := range ep.Sats {
		if s.Sys == gnss.GPS && s.ID == 3 {
			seenGPS3 = true
			if sol.RowSys[i] != gnss.GPS {
				t.Fatal("GPS3 行系统标记错")
			}
		}
		if s.Sys == gnss.BeiDou && s.ID == 3 {
			seenBDS3 = true
			if sol.RowSys[i] != gnss.BeiDou {
				t.Fatal("BDS3 行系统标记错")
			}
		}
	}
	if !seenGPS3 || !seenBDS3 {
		t.Fatalf("GPS3/BDS3 应共存于同一历元, seen=%v/%v", seenGPS3, seenBDS3)
	}
	if len(sol.Resid) != 18 {
		t.Fatalf("残差行数 %d", len(sol.Resid))
	}
}

func TestMultiSystemValidation(t *testing.T) {
	c := multiConstellation()
	good := c.Observe(sim.Obs{})

	cases := []struct {
		name   string
		mutate func(*lsq.Epoch)
		field  string
	}{
		{"未知系统", func(e *lsq.Epoch) { e.Sats[0].Sys = "GLONASS" }, "satellites[0].sys"},
		{"同系统重号", func(e *lsq.Epoch) { e.Sats[2].ID = e.Sats[1].ID; e.Sats[2].Sys = e.Sats[1].Sys }, "satellites[2].id"},
		{"跨系统同号允许", func(e *lsq.Epoch) {
			// 一颗 GPS、一颗 GAL 都改成 10（各系统内原本没有 10）：不报错
			e.Sats[0].ID = 10
			e.Sats[0].Sys = gnss.GPS
			e.Sats[7].ID = 10
			e.Sats[7].Sys = gnss.Galileo
		}, ""},
		{"多系统星数不足3+k", func(e *lsq.Epoch) {
			// 3 GPS + 2 GAL = 5 颗、2 系统：需要 3+2=5 颗才刚好能定位，
			// 这里再砍到 4 颗（3 GPS + 1 GAL），低于 3+k=5
			kept := append([]lsq.Satellite{}, e.Sats[0:3]...) // 3 颗 GPS
			for _, s := range e.Sats[6:] {
				if s.Sys == gnss.Galileo {
					kept = append(kept, s)
					break
				}
			}
			e.Sats = kept
		}, "satellites"},
	}
	for _, tc := range cases {
		e := *good
		sats := make([]lsq.Satellite, len(good.Sats))
		copy(sats, good.Sats)
		e.Sats = sats
		tc.mutate(&e)
		_, err := lsq.Solve(&e)
		if tc.name == "跨系统同号允许" {
			if err != nil {
				t.Fatalf("%s: 跨系统同号不应报错, got %v", tc.name, err)
			}
			continue
		}
		fe, ok := err.(*apierr.FieldError)
		if !ok {
			t.Fatalf("%s: 错误不是 FieldError: %v", tc.name, err)
		}
		if fe.Field != tc.field {
			t.Fatalf("%s: 错误字段=%s, 期望 %s (%v)", tc.name, fe.Field, tc.field, err)
		}
	}
}

func TestMultiSystemSSEMeanMatchesDOF(t *testing.T) {
	// 纯噪声下 SSE 均值应接近 dof = n-(3+k) = 18-6 = 12
	c := multiConstellation()
	// 用独立确定性噪声逐历元在伪距上加噪
	rng := newDeterministic(777)
	var sum float64
	const N = 800
	for i := 0; i < N; i++ {
		ep := c.Observe(sim.Obs{Sigma: 1})
		for j := range ep.Sats {
			ep.Sats[j].PR += rng.gauss()
		}
		sol, err := lsq.Solve(ep)
		if err != nil {
			t.Fatal(err)
		}
		sse, dof := sol.SSE()
		if dof != 12 {
			t.Fatalf("dof=%d", dof)
		}
		sum += sse
	}
	mean := sum / N
	if math.Abs(mean-12)/12 > 0.05 {
		t.Fatalf("SSE 均值 %v 与 dof 12 相对偏差超 5%%", mean)
	}
}

type rngWrap struct{ x uint64 }

func newDeterministic(seed uint64) *rngWrap { return &rngWrap{x: seed} }

// gauss 是一个确定性的近似高斯抽样（Box-Muller），仅测试用。
func (r *rngWrap) gauss() float64 {
	// xorshift64*
	r.x ^= r.x >> 12
	r.x ^= r.x << 25
	r.x ^= r.x >> 27
	u1 := float64(r.x%1000000)/1000000.0 + 1e-12
	r.x ^= r.x >> 12
	r.x ^= r.x << 25
	r.x ^= r.x >> 27
	u2 := float64(r.x%1000000) / 1000000.0
	if u2 < 1e-12 {
		u2 = 1e-12
	}
	return math.Sqrt(-2*math.Log(u1)) * math.Cos(2*math.Pi*u2)
}

func TestMissingSysNoISBColumn(t *testing.T) {
	// 只含 GPS+GAL 两套：5 个参数，DOF=n-5
	rec := sim.DefaultReceiver()
	c := sim.MultiSky(rec, map[gnss.System]int{gnss.GPS: 6, gnss.Galileo: 6}, 15, 11)
	sol, err := lsq.Solve(c.Observe(sim.Obs{}))
	if err != nil {
		t.Fatal(err)
	}
	if len(sol.Systems) != 2 || sol.NPars != 5 {
		t.Fatalf("systems=%v p=%d", sol.Systems, sol.NPars)
	}
	if _, dof := sol.SSE(); dof != 12-5 {
		t.Fatalf("dof=%d", dof)
	}
	if _, ok := sol.ClockOf(gnss.BeiDou); ok {
		t.Fatal("BDS 未参与，不应有钟差列")
	}
}
