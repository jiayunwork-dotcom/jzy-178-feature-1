package lsq_test

import (
	"math"
	"math/rand"
	"testing"

	"raim/internal/sim"
	"raim/pkg/gnss"
	"raim/pkg/lsq"
)

// refSolve4 是教科书式独立实现的 4 未知量（x,y,z,clock）加权最小二乘，
// 作为“升级前 GPS-only 模型”的参考实现，与生产代码不共享任何函数。
func refSolve4(ep *lsq.Epoch) (pos [3]float64, clock, sse float64) {
	p := ep.Approx
	clk := 0.0
	var resid []float64
	for iter := 0; iter < 10; iter++ {
		N := make([][]float64, 4)
		u := make([]float64, 4)
		for i := range N {
			N[i] = make([]float64, 4)
		}
		for _, s := range ep.Sats {
			dx := s.Pos.X - p.X
			dy := s.Pos.Y - p.Y
			dz := s.Pos.Z - p.Z
			r := math.Sqrt(dx*dx + dy*dy + dz*dz)
			g := []float64{-dx / r, -dy / r, -dz / r, 1}
			y := s.PR - (r + clk)
			w := 1 / (s.Sigma * s.Sigma)
			for i := 0; i < 4; i++ {
				u[i] += w * g[i] * y
				for j := 0; j < 4; j++ {
					N[i][j] += w * g[i] * g[j]
				}
			}
		}
		d := gauss4(N, u)
		p.X += d[0]
		p.Y += d[1]
		p.Z += d[2]
		clk += d[3]
		if math.Sqrt(d[0]*d[0]+d[1]*d[1]+d[2]*d[2]) < 0.001 {
			break
		}
	}
	resid = make([]float64, len(ep.Sats))
	for i, s := range ep.Sats {
		dx := s.Pos.X - p.X
		dy := s.Pos.Y - p.Y
		dz := s.Pos.Z - p.Z
		r := math.Sqrt(dx*dx + dy*dy + dz*dz)
		resid[i] = s.PR - (r + clk)
		z := resid[i] / s.Sigma
		sse += z * z
	}
	return [3]float64{p.X, p.Y, p.Z}, clk, sse
}

func gauss4(A [][]float64, b []float64) []float64 {
	// 带列归一化与部分选主元（与生产解算同思想的独立写法）
	sc := make([]float64, 4)
	for i := 0; i < 4; i++ {
		sc[i] = math.Sqrt(math.Abs(A[i][i]))
		if sc[i] == 0 {
			sc[i] = 1
		}
	}
	for i := 0; i < 4; i++ {
		for j := 0; j < 4; j++ {
			A[i][j] /= sc[i] * sc[j]
		}
		b[i] /= sc[i]
	}
	for col := 0; col < 4; col++ {
		piv := col
		for r := col + 1; r < 4; r++ {
			if math.Abs(A[r][col]) > math.Abs(A[piv][col]) {
				piv = r
			}
		}
		A[piv], A[col] = A[col], A[piv]
		b[piv], b[col] = b[col], b[piv]
		for r := 0; r < 4; r++ {
			if r == col {
				continue
			}
			f := A[r][col] / A[col][col]
			for j := col; j < 4; j++ {
				A[r][j] -= f * A[col][j]
			}
			b[r] -= f * b[col]
		}
	}
	x := make([]float64, 4)
	for i := 0; i < 4; i++ {
		x[i] = b[i] / A[i][i] / sc[i]
	}
	return x
}

// 升级兼容性核心：GPS-only 数据上，新多模管线（m=3+1）的位置、钟差、SSE
// 必须与独立的旧 4 未知量参考实现一致（200 个含噪历元、含偏差/钟差场景）。
func TestGPSOnlyPipelineEqualsIndependent4StateReference(t *testing.T) {
	rec := sim.DefaultReceiver()
	c := sim.EvenSky(rec, 9, 15, 5)
	rng := rand.New(rand.NewSource(20261003))
	clock := 73_000.0
	for i := 0; i < 200; i++ {
		bias := map[int]float64(nil)
		if i%37 == 0 {
			bias = map[int]float64{4: 60}
		}
		ep := c.Observe(sim.Obs{Rng: rng, Bias: bias, Clock: &clock})
		sol, err := lsq.Solve(ep)
		if err != nil {
			t.Fatal(err)
		}
		// 新管线确认只有 GPS 一个钟差列
		if len(sol.Systems) != 1 || sol.Systems[0] != gnss.GPS {
			t.Fatalf("GPS-only 应只有 GPS 钟差列, got %v", sol.Systems)
		}
		p, ck, sse := refSolve4(ep)
		if d := math.Sqrt((sol.Pos.X-p[0])*(sol.Pos.X-p[0]) +
			(sol.Pos.Y-p[1])*(sol.Pos.Y-p[1]) +
			(sol.Pos.Z-p[2])*(sol.Pos.Z-p[2])); d > 1e-7 {
			t.Fatalf("历元 %d 位置与旧 4 未知量参考不一致: %.2e m", i, d)
		}
		if math.Abs(sol.ClockBias-ck) > 1e-7 {
			t.Fatalf("历元 %d 钟差不一致: %.2e", i, sol.ClockBias-ck)
		}
		sseNew, dof := lsq.WeightedSSE(sol.Resid, sol.Sigma)
		if dof != 5 || math.Abs(sseNew-sse) > 1e-9 {
			t.Fatalf("历元 %d SSE/dof 不一致: %v(dof=%d) vs %v", i, sseNew, dof, sse)
		}
	}
}
