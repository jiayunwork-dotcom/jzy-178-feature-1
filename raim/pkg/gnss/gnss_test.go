package gnss_test

import (
	"encoding/json"
	"testing"

	"raim/pkg/gnss"
)

func TestParseSystem(t *testing.T) {
	cases := []struct {
		in   string
		want gnss.System
		ok   bool
	}{
		{"", gnss.GPS, true}, // 缺省按 GPS（升级前兼容）
		{"gps", gnss.GPS, true},
		{"GPS", gnss.GPS, true},
		{"G", gnss.GPS, true},
		{"gal", gnss.Galileo, true},
		{"GALILEO", gnss.Galileo, true},
		{"E", gnss.Galileo, true},
		{"bds", gnss.BeiDou, true},
		{"beidou", gnss.BeiDou, true},
		{"COMPASS", gnss.BeiDou, true},
		{"C", gnss.BeiDou, true},
		{"GLONASS", "", false}, // 不认识
		{"SBAS", "", false},
		{"xx", "", false},
	}
	for _, tc := range cases {
		got, ok := gnss.ParseSystem(tc.in)
		if ok != tc.ok || (ok && got != tc.want) {
			t.Fatalf("ParseSystem(%q)=(%q,%v), want (%q,%v)", tc.in, got, ok, tc.want, tc.ok)
		}
	}
}

func TestCrossSystemSameIDDifferentKey(t *testing.T) {
	gps3 := gnss.Key(gnss.GPS, 3)
	bds3 := gnss.Key(gnss.BeiDou, 3)
	gal3 := gnss.Key(gnss.Galileo, 3)
	if gps3 == bds3 || gps3 == gal3 || bds3 == gal3 {
		t.Fatal("跨系统同号必须是不同的主键")
	}
	if gps3.String() != "GPS:3" || bds3.String() != "BDS:3" || gal3.String() != "GAL:3" {
		t.Fatalf("主键字符串 %s %s %s", gps3, bds3, gal3)
	}
	m := map[gnss.SatKey]string{gps3: "a", bds3: "b", gal3: "c"}
	if len(m) != 3 || m[gps3] != "a" || m[bds3] != "b" || m[gal3] != "c" {
		t.Fatalf("三颗同号不同系统星应能同时作 map 键: %v", m)
	}
}

func TestKeyJSONRoundTrip(t *testing.T) {
	m := map[gnss.SatKey]int{
		gnss.Key(gnss.GPS, 3):      1,
		gnss.Key(gnss.Galileo, 12): 2,
		gnss.Key(gnss.BeiDou, 3):   3,
	}
	b, err := json.Marshal(m)
	if err != nil {
		t.Fatal(err)
	}
	var out map[gnss.SatKey]int
	if err := json.Unmarshal(b, &out); err != nil {
		t.Fatal(err)
	}
	if len(out) != 3 ||
		out[gnss.Key(gnss.GPS, 3)] != 1 ||
		out[gnss.Key(gnss.Galileo, 12)] != 2 ||
		out[gnss.Key(gnss.BeiDou, 3)] != 3 {
		t.Fatalf("主键 JSON 往返不一致: %v (json=%s)", out, b)
	}
}

func TestLegacyBareKeyUnmarshalsAsGPS(t *testing.T) {
	// 升级前落盘的隔离 map 键是裸编号 "3"，必须读成 GPS:3
	var k gnss.SatKey
	if err := json.Unmarshal([]byte(`"3"`), &k); err != nil {
		t.Fatal(err)
	}
	if k != (gnss.SatKey{Sys: gnss.GPS, ID: 3}) {
		t.Fatalf("裸编号应解释为 GPS:3, got %s", k)
	}
	// 新格式正常
	if err := json.Unmarshal([]byte(`"BDS:7"`), &k); err != nil {
		t.Fatal(err)
	}
	if k != (gnss.SatKey{Sys: gnss.BeiDou, ID: 7}) {
		t.Fatalf("got %s", k)
	}
	// 非法内容报错
	if err := json.Unmarshal([]byte(`"XYZ:1"`), &k); err == nil {
		t.Fatal("未知系统主键应反序列化失败")
	}
}

func TestLessOrdering(t *testing.T) {
	keys := []gnss.SatKey{
		gnss.Key(gnss.BeiDou, 1),
		gnss.Key(gnss.GPS, 9),
		gnss.Key(gnss.GPS, 2),
		gnss.Key(gnss.Galileo, 5),
		gnss.Key(gnss.BeiDou, 3),
		gnss.Key(gnss.Galileo, 2),
	}
	want := []gnss.SatKey{
		gnss.Key(gnss.GPS, 2), gnss.Key(gnss.GPS, 9),
		gnss.Key(gnss.Galileo, 2), gnss.Key(gnss.Galileo, 5),
		gnss.Key(gnss.BeiDou, 1), gnss.Key(gnss.BeiDou, 3),
	}
	for i := 1; i < len(keys); i++ {
		for j := i; j > 0 && gnss.Less(keys[j], keys[j-1]); j-- {
			keys[j], keys[j-1] = keys[j-1], keys[j]
		}
	}
	for i := range want {
		if keys[i] != want[i] {
			t.Fatalf("排序位置 %d: got %s want %s", i, keys[i], want[i])
		}
	}
}
