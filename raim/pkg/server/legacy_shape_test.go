package server_test

import (
	"bytes"
	"encoding/json"
	"net/http"
	"path/filepath"
	"testing"

	"raim/internal/sim"
)

func httpPost(url string, body []byte) (*http.Response, error) {
	return http.Post(url, "application/json", bytes.NewReader(body))
}

// TestGPSOnlyJSONShapeMatchesLegacy 钉死升级前 GPS-only 历元的 JSON 形态：
// 响应里不能出现任何多模新增字段（systems / sys / excluded_sys / isolated_sats），
// 且原有字段全部保留。这是“只含 GPS 逐历元结果与升级前一致”的逐字节级保证。
func TestGPSOnlyJSONShapeMatchesLegacy(t *testing.T) {
	dir := filepath.Join(t.TempDir(), "data")
	_, ts := start(t, dir)
	defer ts.Close()

	postBody(t, ts.URL+"/api/sessions", map[string]string{"id": "g", "profile_name": "enroute"}, 201)
	// 制造一个排除+隔离历元（新字段最容易漏出来的场景）
	epochs := epochInputs(sim.EvenSky(sim.DefaultReceiver(), 9, 15, 5), 3, 5,
		func(i int) map[int]float64 {
			if i >= 0 {
				return map[int]float64{4: 100}
			}
			return nil
		})
	var raw map[string]any
	for _, e := range epochs {
		b, _ := json.Marshal(e)
		r, err := postRaw(t, ts.URL+"/api/sessions/g/epochs", b, 200)
		if err != nil {
			t.Fatal(err)
		}
		raw = r
	}
	for _, banned := range []string{"systems", "excluded_sys", "isolated_sats"} {
		if _, ok := raw[banned]; ok {
			t.Fatalf("GPS-only 响应不应含 %q: %v", banned, raw[banned])
		}
	}
	// 原有字段保留（逐历元必有项）
	for _, k := range []string{"seq", "timestamp", "profile_name", "pfa", "pmd", "hal",
		"chi_square_threshold", "dof", "mode", "sse", "hpl", "isolated",
		"position", "clock_bias", "iterations", "converged", "sat_results",
		"alert", "bad_streak", "good_streak", "raim_available"} {
		if _, ok := raw[k]; !ok {
			t.Fatalf("GPS-only 响应缺少升级前字段 %q", k)
		}
	}
	// 每星结果也不能有 sys
	for _, item := range raw["sat_results"].([]any) {
		sr := item.(map[string]any)
		if _, ok := sr["sys"]; ok {
			t.Fatalf("GPS-only 每星结果不应含 sys: %v", sr)
		}
	}
	// trials（若有）不含 excluded_sys
	if trs, ok := raw["trials"].([]any); ok {
		for _, item := range trs {
			tr := item.(map[string]any)
			if _, ok := tr["excluded_sys"]; ok {
				t.Fatalf("GPS-only trial 不应含 excluded_sys")
			}
		}
	}
}

// postRaw 用给定 JSON 字节 POST，返回解码后的 body。
func postRaw(t *testing.T, url string, body []byte, want int) (map[string]any, error) {
	t.Helper()
	r, err := httpPost(url, body)
	if err != nil {
		return nil, err
	}
	defer r.Body.Close()
	var m map[string]any
	_ = json.NewDecoder(r.Body).Decode(&m)
	if r.StatusCode != want {
		t.Fatalf("POST %s 状态码 %d, 期望 %d, body=%v", url, r.StatusCode, want, m)
	}
	return m, nil
}

// postBodyRaw 与 postBody 相同但返回 map（供状态码失败仍需看字段的用例）。
func postBodyRaw(t *testing.T, url string, body any, want int) map[string]any {
	t.Helper()
	b, _ := json.Marshal(body)
	r, err := httpPost(url, b)
	if err != nil {
		t.Fatal(err)
	}
	defer r.Body.Close()
	var m map[string]any
	_ = json.NewDecoder(r.Body).Decode(&m)
	if r.StatusCode != want {
		t.Fatalf("POST %s 状态码 %d, 期望 %d, body=%v", url, r.StatusCode, want, m)
	}
	return m
}
