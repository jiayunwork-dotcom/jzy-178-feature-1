package server_test

import (
	"bytes"
	"encoding/json"
	"math/rand"
	"net/http"
	"path/filepath"
	"testing"

	"raim/internal/sim"
	"raim/pkg/gnss"
	"raim/pkg/session"
)

func multiEpochInputs(c *sim.Constellation, n int, seed int64,
	stepAt int, step float64) []session.EpochInput {
	rng := rand.New(rand.NewSource(seed))
	out := make([]session.EpochInput, n)
	for i := range out {
		ggto := map[gnss.System]float64{}
		if stepAt >= 0 && i >= stepAt {
			ggto[gnss.GAL] = step
		}
		ep := c.ObserveMulti(sim.MultiObs{Rng: rng, Sigma: 1, GGTO: ggto})
		sats := make([]session.SatInput, len(ep.Sats))
		for j, s := range ep.Sats {
			sats[j] = session.SatInput{
				System: s.System, ID: s.ID,
				Pos: [3]float64{s.Pos.X, s.Pos.Y, s.Pos.Z},
				PR:  s.PR, Sigma: s.Sigma,
			}
		}
		out[i] = session.EpochInput{Timestamp: int64(i + 1), Sats: sats}
	}
	return out
}

// 多模数据经 HTTP：批量与逐历元（含中途重启）逐项一致。
func TestHTTPMultiSystemBatchEqualsRestartIndividual(t *testing.T) {
	rec := sim.DefaultReceiver()
	c := sim.MultiSky(rec, []gnss.System{gnss.GPS, gnss.GAL, gnss.BDS}, 9, 15, 2026)
	epochs := multiEpochInputs(c, 30, 31, 10, 50_000)

	// A：整段批量
	dirA := filepath.Join(t.TempDir(), "dataA")
	_, tsA := start(t, dirA)
	postBody(t, tsA.URL+"/api/sessions",
		map[string]string{"id": "a", "profile_name": "enroute"}, http.StatusCreated)
	postBody(t, tsA.URL+"/api/sessions/a/epochs/batch",
		map[string]any{"epochs": epochs}, http.StatusOK)
	respA := mustGet(t, tsA.URL+"/api/sessions/a")
	tsA.Close()

	// B：逐历元 + 中途重启
	dirB := filepath.Join(t.TempDir(), "dataB")
	_, tsB := start(t, dirB)
	postBody(t, tsB.URL+"/api/sessions",
		map[string]string{"id": "b", "profile_name": "enroute"}, http.StatusCreated)
	for _, e := range epochs[:12] {
		postBody(t, tsB.URL+"/api/sessions/b/epochs", e, http.StatusOK)
	}
	tsB.Close()
	_, tsB2 := start(t, dirB)
	for _, e := range epochs[12:] {
		postBody(t, tsB2.URL+"/api/sessions/b/epochs", e, http.StatusOK)
	}
	respB := mustGet(t, tsB2.URL+"/api/sessions/b")
	defer tsB2.Close()

	ra := recsOf(respA)
	rb := recsOf(respB)
	if len(ra) != len(rb) {
		t.Fatalf("记录数 %d vs %d", len(ra), len(rb))
	}
	for i := range ra {
		ba, _ := json.Marshal(ra[i])
		bb, _ := json.Marshal(rb[i])
		if !bytes.Equal(ba, bb) {
			t.Fatalf("历元 %d 多模结果不一致:\nA=%s\nB=%s", i, ba, bb)
		}
	}
}

// HTTP 输入校验：未知系统 400 且指到具体字段。
func TestHTTPMultiSystemUnknownSystem(t *testing.T) {
	rec := sim.DefaultReceiver()
	c := sim.MultiSky(rec, []gnss.System{gnss.GPS, gnss.GAL, gnss.BDS}, 6, 15, 2026)
	epochs := multiEpochInputs(c, 2, 5, -1, 0)

	dir := filepath.Join(t.TempDir(), "data")
	_, ts := start(t, dir)
	defer ts.Close()
	postBody(t, ts.URL+"/api/sessions",
		map[string]string{"id": "m", "profile_name": "enroute"}, http.StatusCreated)

	bad := epochs[0]
	bad.Sats[0].System = gnss.System("QZS")
	b, _ := json.Marshal(bad)
	r, err := http.Post(ts.URL+"/api/sessions/m/epochs", "application/json", bytes.NewReader(b))
	if err != nil {
		t.Fatal(err)
	}
	defer r.Body.Close()
	if r.StatusCode != http.StatusBadRequest {
		t.Fatalf("未知系统应 400, got %d", r.StatusCode)
	}
	var m map[string]any
	_ = json.NewDecoder(r.Body).Decode(&m)
	if m["field"] != "satellites[0].system" {
		t.Fatalf("错误字段=%v", m["field"])
	}

	// 正常多模历元应 200
	ok, _ := json.Marshal(epochs[1])
	r2, err := http.Post(ts.URL+"/api/sessions/m/epochs", "application/json", bytes.NewReader(ok))
	if err != nil {
		t.Fatal(err)
	}
	defer r2.Body.Close()
	if r2.StatusCode != http.StatusOK {
		t.Fatalf("合法多模历元应 200, got %d", r2.StatusCode)
	}
}

// HTTP 旧客户端（卫星不带 system）继续可用，按 GPS 处理。
func TestHTTPLegacyClientNoSystem(t *testing.T) {
	dir := filepath.Join(t.TempDir(), "data")
	_, ts := start(t, dir)
	defer ts.Close()
	postBody(t, ts.URL+"/api/sessions",
		map[string]string{"id": "g", "profile_name": "enroute"}, http.StatusCreated)

	epochs := epochInputs(sim.EvenSky(sim.DefaultReceiver(), 9, 15, 5), 3, 5, nil)
	for _, e := range epochs {
		postBody(t, ts.URL+"/api/sessions/g/epochs", e, http.StatusOK)
	}
}
