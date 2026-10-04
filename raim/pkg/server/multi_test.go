package server_test

import (
	"math/rand"
	"path/filepath"
	"testing"

	"raim/internal/sim"
	"raim/pkg/gnss"
	"raim/pkg/session"
)

func multiEpochInputs(n, seed int) []session.EpochInput {
	rec := sim.DefaultReceiver()
	c := sim.MultiSky(rec,
		map[gnss.System]int{gnss.GPS: 7, gnss.Galileo: 7, gnss.BeiDou: 7},
		15, int64(seed))
	rng := rand.New(rand.NewSource(int64(seed)))
	out := make([]session.EpochInput, n)
	for i := range out {
		ep := c.Observe(sim.Obs{Rng: rng, Sigma: 1})
		sats := make([]session.SatInput, len(ep.Sats))
		for j, s := range ep.Sats {
			in := session.SatInput{
				ID:  s.ID,
				Pos: [3]float64{s.Pos.X, s.Pos.Y, s.Pos.Z},
				PR:  s.PR, Sigma: s.Sigma,
			}
			if s.Sys != gnss.GPS {
				in.Sys = string(s.Sys)
			}
			sats[j] = in
		}
		out[i] = session.EpochInput{Timestamp: int64(i + 1), Sats: sats}
	}
	return out
}

func TestHTTPMultiSystemEpoch(t *testing.T) {
	dir := filepath.Join(t.TempDir(), "data")
	_, ts := start(t, dir)
	defer ts.Close()

	postBody(t, ts.URL+"/api/sessions", map[string]string{"id": "m", "profile_name": "enroute"}, 201)
	epochs := multiEpochInputs(3, 7)
	for _, e := range epochs {
		resp := postBody(t, ts.URL+"/api/sessions/m/epochs", e, 200)
		systems, ok := resp["systems"].([]any)
		if !ok || len(systems) != 3 {
			t.Fatalf("应返回 3 套系统结果, got %v", resp["systems"])
		}
	}
}

func TestHTTPMultiValidationErrors(t *testing.T) {
	dir := filepath.Join(t.TempDir(), "data")
	_, ts := start(t, dir)
	defer ts.Close()

	postBody(t, ts.URL+"/api/sessions", map[string]string{"id": "m", "profile_name": "enroute"}, 201)
	epochs := multiEpochInputs(4, 9)

	// 未知系统标识 -> 400 + 字段
	bad := epochs[0]
	bad.Timestamp = 100
	bad.Sats[0].Sys = "GLONASS"
	resp := postBody(t, ts.URL+"/api/sessions/m/epochs", bad, 400)
	if resp["field"] != "satellites[0].sys" {
		t.Fatalf("field=%v", resp["field"])
	}

	// 同系统重号 -> 400 + 字段
	bad2 := epochs[1]
	bad2.Timestamp = 101
	bad2.Sats[2].ID = bad2.Sats[1].ID
	bad2.Sats[2].Sys = bad2.Sats[1].Sys
	resp = postBody(t, ts.URL+"/api/sessions/m/epochs", bad2, 400)
	if resp["field"] != "satellites[2].id" {
		t.Fatalf("field=%v", resp["field"])
	}

	// 跨系统同号合法
	ok3 := epochs[2]
	ok3.Timestamp = 102
	for i := range ok3.Sats {
		if ok3.Sats[i].Sys == "GAL" {
			ok3.Sats[i].ID = 1
			break
		}
	}
	postBody(t, ts.URL+"/api/sessions/m/epochs", ok3, 200)

	// 批量错误带历元下标
	batch := multiEpochInputs(5, 10)
	for i := range batch {
		batch[i].Timestamp = int64(200 + i)
	}
	batch[2].Sats[3].Sys = "QWERTY"
	batchResp := postBody(t, ts.URL+"/api/sessions/m/epochs/batch",
		map[string]any{"epochs": batch}, 400)
	if batchResp["field"] != "epochs[2].satellites[3].sys" {
		t.Fatalf("批量字段=%v", batchResp["field"])
	}
}

func TestHTTPLegacyClientWithoutSysIsGPS(t *testing.T) {
	// 升级前客户端不带 sys 字段：全部按 GPS，历元正常接受且不输出多系统字段
	dir := filepath.Join(t.TempDir(), "data")
	_, ts := start(t, dir)
	defer ts.Close()

	postBody(t, ts.URL+"/api/sessions", map[string]string{"id": "g", "profile_name": "enroute"}, 201)
	epochs := epochInputs(sim.EvenSky(sim.DefaultReceiver(), 9, 15, 5), 3, 5, nil)
	for _, e := range epochs {
		resp := postBody(t, ts.URL+"/api/sessions/g/epochs", e, 200)
		if s, present := resp["systems"]; present && s != nil {
			t.Fatalf("纯 GPS 历元不应有 systems: %v", s)
		}
	}
}
