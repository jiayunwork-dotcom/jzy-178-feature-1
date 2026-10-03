// Package gnss 定义多模卫星导航的系统标识与“系统+编号”卫星身份。
//
// 三套系统（GPS / Galileo / 北斗）的卫星编号各自独立、会重号
// （GPS 3 号与北斗 3 号是两颗不同的星），因此本服务中所有按星索引的结构
// （隔离集合、排除结果、每星残差、报错字段等）一律使用 SatID{System, ID}，
// 不允许再用裸编号。
package gnss

import (
	"encoding/json"
	"strconv"
	"strings"
)

// System 是卫星导航系统标识（落盘/接口字符串，固定大写）。
type System string

const (
	// GPS 全球定位系统。升级前没有系统标识的旧数据一律按 GPS 处理。
	GPS System = "GPS"
	// GAL Galileo。
	GAL System = "GAL"
	// BDS 北斗（BeiDou）。
	BDS System = "BDS"
)

// Systems 按固定规范顺序返回全部受支持系统（GPS 为基准系统，排第一）。
// 输出顺序敏感处（每历元系统钟差列表、排序、仿真布星）都用它，保证确定性。
func Systems() []System { return []System{GPS, GAL, BDS} }

// Valid 报告系统标识是否受支持。空串也视为非法（缺省由上层兼容逻辑补成 GPS）。
func (s System) Valid() bool {
	switch s {
	case GPS, GAL, BDS:
		return true
	default:
		return false
	}
}

// ParseSystem 按精确（大小写敏感）匹配解析系统标识；不认识返回 false。
func ParseSystem(s string) (System, bool) {
	sys := System(strings.TrimSpace(s))
	if !sys.Valid() {
		return "", false
	}
	return sys, true
}

// SatID 是跨系统唯一的卫星身份。可作为 map 键。
type SatID struct {
	System System
	ID     int
}

// Key 便于构造 map 键的辅助函数。
func Key(sys System, id int) SatID { return SatID{System: sys, ID: id} }

// String 仅用于日志/报错，如 "GPS:4"、"BDS:3"。
func (s SatID) String() string { return string(s.System) + ":" + strconv.Itoa(s.ID) }

// Compare 对卫星身份做全序排序：先按系统规范顺序（GPS<GAL<BDS），再按编号。
// 返回 -1/0/1。
func Compare(a, b SatID) int {
	ia, ib := sysIndex(a.System), sysIndex(b.System)
	if ia != ib {
		if ia < ib {
			return -1
		}
		return 1
	}
	if a.ID < b.ID {
		return -1
	}
	if a.ID > b.ID {
		return 1
	}
	return 0
}

func sysIndex(s System) int {
	for i, x := range Systems() {
		if x == s {
			return i
		}
	}
	return len(Systems())
}

// MarshalText 把 SatID 编成紧凑文本（map 键落盘用），如 "GPS:4"。
func (s SatID) MarshalText() ([]byte, error) {
	return []byte(s.String()), nil
}

// UnmarshalText 兼容两种落盘形态：
//
//	"4"     —— 升级前的旧隔离状态，裸编号按 GPS 处理；
//	"GPS:4" —— 升级后的新形态。
func (s *SatID) UnmarshalText(b []byte) error {
	text := string(b)
	if i := strings.IndexByte(text, ':'); i >= 0 {
		s.System = System(text[:i])
		id, err := strconv.Atoi(text[i+1:])
		if err != nil {
			return err
		}
		s.ID = id
	} else {
		id, err := strconv.Atoi(text)
		if err != nil {
			return err
		}
		s.System = GPS
		s.ID = id
	}
	return nil
}

// SatRef 是面向记录/接口的卫星身份引用。JSON 形态刻意保持精简：
//
//	4                      —— GPS（裸编号，与升级前记录中的整数形态一致）
//	{"system":"GAL","id":4} —— 其余系统
type SatRef struct {
	System System `json:"-"`
	ID     int    `json:"id"`
}

// Ref 由 SatID 构造引用。
func Ref(k SatID) SatRef { return SatRef{System: k.System, ID: k.ID} }

// Key 转回 SatID。
func (r SatRef) Key() SatID { return SatID{System: r.System, ID: r.ID} }

// MarshalJSON：GPS 输出裸编号，旧消费方照读、旧记录形态逐字段一致。
func (r SatRef) MarshalJSON() ([]byte, error) {
	if r.System == GPS || r.System == "" {
		return []byte(strconv.Itoa(r.ID)), nil
	}
	return []byte(`{"system":"` + string(r.System) + `","id":` + strconv.Itoa(r.ID) + `}`), nil
}

// UnmarshalJSON：接受裸编号（旧记录，按 GPS）或 {"system":...,"id":...}。
func (r *SatRef) UnmarshalJSON(b []byte) error {
	if id, err := strconv.Atoi(strings.TrimSpace(string(b))); err == nil {
		r.System = GPS
		r.ID = id
		return nil
	}
	var v struct {
		System string `json:"system"`
		ID     int    `json:"id"`
	}
	if err := json.Unmarshal(b, &v); err != nil {
		return err
	}
	r.ID = v.ID
	if v.System == "" {
		r.System = GPS
	} else {
		r.System = System(v.System)
	}
	return nil
}
