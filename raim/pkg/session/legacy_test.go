package session_test

import (
	"encoding/json"
	"os"
	"path/filepath"
	"testing"

	"raim/internal/sim"
	"raim/pkg/gnss"
	"raim/pkg/profile"
	"raim/pkg/session"
)

// enrouteShape 复制内置 enroute 档（不依赖文件，直接用内存服务算旧记录数值）。
func enrouteShape() profile.Profile {
	p, _ := profile.Builtin("enroute")
	return p
}

// buildLegacySessionFile 按“升级前”的磁盘格式手写一个旧会话：
//   - 卫星没有 system 字段（EpochInput 不进文件，体现在记录里 GPS 星无 system）；
//   - state.isolated 用裸编号整数键；
//   - 历元记录没有 excluded_system / system_clocks；
//   - 隔离列表是裸整数数组。
//
// 内容：3 个历元，第 2 历元排除 4 号星（enroute 档，连续 5 历元才恢复）。
func buildLegacySessionFile(t *testing.T, dir string) {
	t.Helper()
	if err := os.MkdirAll(filepath.Join(dir, "sessions"), 0o755); err != nil {
		t.Fatal(err)
	}

	rec := sim.DefaultReceiver()
	c := sim.EvenSky(rec, 9, 15, 5)
	epochs := genEpochs(c, 3, 1, 2024, faultWindow(4, 1, 2, 80))

	// 用当前代码跑同样的数据得到自洽数值，再剥掉多模新增字段，模拟升级前形态。
	prof := enrouteShape()
	svc := session.NewService([]profile.Profile{prof})
	sess, _ := svc.NewSession("legacy", "enroute")
	var legacyRecs []map[string]any
	for _, e := range epochs {
		r, err := svc.Step(sess, e)
		if err != nil {
			t.Fatal(err)
		}
		b, err := json.Marshal(r)
		if err != nil {
			t.Fatal(err)
		}
		var m map[string]any
		if err := json.Unmarshal(b, &m); err != nil {
			t.Fatal(err)
		}
		delete(m, "excluded_system")
		delete(m, "system_clocks")
		legacyRecs = append(legacyRecs, m)
	}

	doc := map[string]any{
		"id":           "legacy",
		"profile_name": "enroute",
		"created_at":   "2026-09-30T08:00:00Z",
		"last_ts":      3,
		"state": map[string]any{
			"isolated": map[string]any{"4": map[string]any{
				"id": 4, "normal_streak": 0, "since_epoch_seq": 2,
			}},
			"alert": map[string]any{
				"active": false, "bad_streak": 0, "good_streak": 1,
			},
		},
		"stats": map[string]any{
			"epochs": 3, "raim_available_epochs": 3, "alert_epochs": 0,
			"detections": 1, "alert_episodes": 0, "false_alarm_episodes": 0,
			"raim_availability": 1, "service_availability": 1,
		},
		"records":         legacyRecs,
		"alert_open_risk": false,
	}
	b, err := json.MarshalIndent(doc, "", "  ")
	if err != nil {
		t.Fatal(err)
	}
	if err := os.WriteFile(filepath.Join(dir, "sessions", "legacy.json"), b, 0o644); err != nil {
		t.Fatal(err)
	}
}

// 验收 4：升级前生成的会话文件拿来接着提交——旧记录原样保留，新历元正常推进。
func TestLegacySessionResumeKeepsOldRecords(t *testing.T) {
	dir := filepath.Join(t.TempDir(), "data")
	buildLegacySessionFile(t, dir)

	path := filepath.Join(dir, "sessions", "legacy.json")
	before, err := os.ReadFile(path)
	if err != nil {
		t.Fatal(err)
	}
	var beforeDoc map[string]any
	if err := json.Unmarshal(before, &beforeDoc); err != nil {
		t.Fatal(err)
	}
	beforeRecs := beforeDoc["records"].([]any)

	// Store 会补齐内置 enroute 运行档，并从磁盘加载旧会话。
	st, err := session.NewStore(dir)
	if err != nil {
		t.Fatal(err)
	}
	sess, err := st.GetSession("legacy")
	if err != nil {
		t.Fatal(err)
	}
	if sess.Stats.Epochs != 3 {
		t.Fatalf("旧会话历元数应为 3, got %d", sess.Stats.Epochs)
	}
	// 旧隔离状态（裸编号 4）必须被识别为 GPS:4
	if _, ok := sess.State.Isolated[gnss.Key(gnss.GPS, 4)]; !ok {
		t.Fatalf("旧隔离星 4 应按 GPS:4 加载, got %v", sess.State.Isolated)
	}

	// 接着提交无系统标识的新历元（旧回放端继续推数），按 GPS 正常推进。
	rec := sim.DefaultReceiver()
	c := sim.EvenSky(rec, 9, 15, 5)
	newEpochs := genEpochs(c, 3, 1, 4321, nil)
	for _, e := range newEpochs {
		e.Timestamp += 100 // 避开 last_ts=3
		if _, err := st.AppendEpoch("legacy", e); err != nil {
			t.Fatal(err)
		}
	}

	after, err := os.ReadFile(path)
	if err != nil {
		t.Fatal(err)
	}
	var afterDoc map[string]any
	if err := json.Unmarshal(after, &afterDoc); err != nil {
		t.Fatal(err)
	}
	afterRecs := afterDoc["records"].([]any)
	if len(afterRecs) != 6 {
		t.Fatalf("应有 6 条记录, got %d", len(afterRecs))
	}
	for i := 0; i < 3; i++ {
		bo, _ := json.Marshal(beforeRecs[i])
		ao, _ := json.Marshal(afterRecs[i])
		if string(bo) != string(ao) {
			t.Fatalf("旧历元 %d 被改写:\n旧=%s\n新=%s", i+1, bo, ao)
		}
		m := afterRecs[i].(map[string]any)
		if _, has := m["system_clocks"]; has {
			t.Fatalf("旧历元 %d 不应被补写 system_clocks", i+1)
		}
		if _, has := m["excluded_system"]; has {
			t.Fatalf("旧历元 %d 不应被补写 excluded_system", i+1)
		}
	}
	// 新历元是升级后形态
	rec4 := afterRecs[3].(map[string]any)
	if _, has := rec4["system_clocks"]; !has {
		t.Fatal("新历元应含 system_clocks")
	}
	sess2, _ := st.GetSession("legacy")
	if sess2.Stats.Epochs != 6 {
		t.Fatalf("续跑后历元数应为 6, got %d", sess2.Stats.Epochs)
	}
}

// 验收 5（形态部分）：只含 GPS 的新会话记录保持升级前的紧凑形态。
func TestGPSOnlyRecordsMatchLegacyShape(t *testing.T) {
	rec := sim.DefaultReceiver()
	c := sim.EvenSky(rec, 9, 15, 5)
	epochs := genEpochs(c, 6, 1, 77, faultWindow(4, 2, 3, 80))
	st := newStore(t)
	if _, err := st.CreateSession("g", "enroute"); err != nil {
		t.Fatal(err)
	}
	var recs []*session.EpochRecord
	for _, e := range epochs {
		r, err := st.AppendEpoch("g", e)
		if err != nil {
			t.Fatal(err)
		}
		recs = append(recs, r)
	}
	for i, r := range recs {
		b, err := json.Marshal(r)
		if err != nil {
			t.Fatal(err)
		}
		var m map[string]any
		if err := json.Unmarshal(b, &m); err != nil {
			t.Fatal(err)
		}
		if _, has := m["excluded_system"]; has {
			t.Fatalf("GPS 历元 %d 不应出现 excluded_system", i+1)
		}
		if iso, ok := m["isolated"].([]any); ok {
			for _, x := range iso {
				if _, isNum := x.(float64); !isNum {
					t.Fatalf("GPS 隔离引用应为裸编号, got %v", x)
				}
			}
		}
		for _, sr := range asMaps(m["sat_results"]) {
			if _, has := sr["system"]; has {
				t.Fatalf("GPS sat_results 不应带 system: %v", sr)
			}
		}
		for _, tr := range asMaps(m["trials"]) {
			if _, has := tr["system"]; has {
				t.Fatalf("GPS trials 不应带 system: %v", tr)
			}
		}
		if len(asMaps(m["system_clocks"])) != 3 {
			t.Fatalf("应输出三系统钟差表, got %v", m["system_clocks"])
		}
	}
	if recs[2].ExcludedID != 4 || recs[2].ExcludedSystem != gnss.GPS {
		t.Fatalf("故障历元应排除 GPS:4, got %s:%d", recs[2].ExcludedSystem, recs[2].ExcludedID)
	}
}

func asMaps(v any) []map[string]any {
	xs, ok := v.([]any)
	if !ok {
		return nil
	}
	out := make([]map[string]any, 0, len(xs))
	for _, x := range xs {
		if m, ok := x.(map[string]any); ok {
			out = append(out, m)
		}
	}
	return out
}
