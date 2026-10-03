package main

import (
	"bytes"
	"encoding/json"
	"os"
	"path/filepath"
	"testing"
)

// 固定种子回放结果必须与 docs/replay_results.json 完全一致。
// 这是“只含 GPS 的数据升级前后逐历元结果相同”的端到端钉死：
// 多模改造若改动了 GPS-only 的任何数值（定位/检测/排除/保护级/隔离/告警/统计），
// 该黄金文件比对立即失败。
func TestReplayGolden(t *testing.T) {
	out := run(20260930)
	var got bytes.Buffer
	enc := json.NewEncoder(&got)
	enc.SetIndent("", "  ")
	if err := enc.Encode(out); err != nil {
		t.Fatal(err)
	}

	want, err := os.ReadFile(filepath.Join("..", "..", "docs", "replay_results.json"))
	if err != nil {
		t.Fatal(err)
	}
	if !bytes.Equal(got.Bytes(), want) {
		t.Fatalf("回放结果与 docs/replay_results.json 不一致（GPS-only 兼容性被破坏）\n got=%s\nwant=%s",
			got.Bytes(), want)
	}
}
