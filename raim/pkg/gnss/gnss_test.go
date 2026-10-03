package gnss_test

import (
	"encoding/json"
	"testing"

	"raim/pkg/gnss"
)

func TestParseAndValidate(t *testing.T) {
	if _, ok := gnss.ParseSystem("GPS"); !ok {
		t.Fatal("GPS 应受支持")
	}
	for _, bad := range []string{"gps", "GLO", ""} {
		if _, ok := gnss.ParseSystem(bad); ok {
			t.Fatalf("小写/未知/空标识不应接受: %q", bad)
		}
	}
}

func TestSameNumberDifferentSystemDistinct(t *testing.T) {
	gps3 := gnss.Key(gnss.GPS, 3)
	bds3 := gnss.Key(gnss.BDS, 3)
	gal3 := gnss.Key(gnss.GAL, 3)
	m := map[gnss.SatID]string{gps3: "g", bds3: "b", gal3: "e"}
	if len(m) != 3 {
		t.Fatalf("跨系统同号必须是不同的键, got %d", len(m))
	}
	if gnss.Compare(gps3, bds3) >= 0 || gnss.Compare(bds3, gal3) <= 0 || gnss.Compare(gps3, gps3) != 0 {
		t.Fatal("全序：GPS<GAL<BDS，再按编号")
	}
}

func TestSatIDTextRoundTrip(t *testing.T) {
	k := gnss.Key(gnss.BDS, 7)
	b, err := k.MarshalText()
	if err != nil || string(b) != "BDS:7" {
		t.Fatalf("MarshalText=%s err=%v", b, err)
	}
	// 旧形态裸编号按 GPS
	var legacy gnss.SatID
	if err := legacy.UnmarshalText([]byte("7")); err != nil {
		t.Fatal(err)
	}
	if legacy != gnss.Key(gnss.GPS, 7) {
		t.Fatalf("裸编号应解析为 GPS:7, got %v", legacy)
	}
	// map 键经 JSON 往返
	mp := map[gnss.SatID]int{k: 2}
	jb, _ := json.Marshal(mp)
	var back map[gnss.SatID]int
	if err := json.Unmarshal(jb, &back); err != nil {
		t.Fatal(err)
	}
	if back[k] != 2 {
		t.Fatalf("map JSON 往返失败: %s -> %v", jb, back)
	}
}

func TestSatRefJSON(t *testing.T) {
	// GPS 落盘为裸编号
	b, _ := json.Marshal(gnss.Ref(gnss.Key(gnss.GPS, 4)))
	if string(b) != "4" {
		t.Fatalf("GPS 引用应落盘为裸编号, got %s", b)
	}
	// 北斗落盘为带 system 的对象
	b, _ = json.Marshal(gnss.Ref(gnss.Key(gnss.BDS, 4)))
	if string(b) != `{"system":"BDS","id":4}` {
		t.Fatalf("BDS 引用形态不符, got %s", b)
	}
	// 旧记录裸编号反序列化
	var r gnss.SatRef
	if err := json.Unmarshal([]byte("4"), &r); err != nil {
		t.Fatal(err)
	}
	if r.Key() != gnss.Key(gnss.GPS, 4) {
		t.Fatalf("裸编号应按 GPS, got %v", r)
	}
	// 对象形态往返
	var r2 gnss.SatRef
	if err := json.Unmarshal([]byte(`{"system":"GAL","id":9}`), &r2); err != nil {
		t.Fatal(err)
	}
	if r2.Key() != gnss.Key(gnss.GAL, 9) {
		t.Fatalf("got %v", r2)
	}
}
