// Package gnss 定义多模卫星导航的系统标识与“系统+编号”卫星主键。
//
// 三套系统的卫星编号空间互相重叠（GPS 3 号与北斗 3 号是两颗不同的星），
// 因此本服务一切按星索引的结构（隔离集合、排除结果、每星残差、错误定位）
// 都使用 SatKey{Sys, ID}，不能只用车载编号。
//
// 兼容升级前数据：系统标识缺省一律按 GPS 处理；落盘的旧状态文件里裸编号
// （如 "3"）在反序列化时解释为 GPS:3。
package gnss

import (
	"strconv"
	"strings"
)

// System 是卫星导航系统标识（规范大写码）。
type System string

const (
	// GPS 全球定位系统；也是缺省系统（升级前的星无系统标识）。
	GPS System = "GPS"
	// Galileo 伽利略，规范码 GAL（RINEX 前缀 E）。
	Galileo System = "GAL"
	// BeiDou 北斗卫星导航系统，规范码 BDS（RINEX 前缀 C）。
	BeiDou System = "BDS"
)

// All 按固定规范顺序列出全部支持的系统（GPS、GAL、BDS）。
// 顺序用于多系统解算的钟差列排列与输出，保证与卫星在输入中的次序无关。
var All = []System{GPS, Galileo, BeiDou}

// Valid 判断系统标识是否受支持。
func (s System) Valid() bool {
	switch s {
	case GPS, Galileo, BeiDou:
		return true
	}
	return false
}

// ParseSystem 解析调用方给的系统标识。空串按 GPS（升级前缺省）；
// 大小写不敏感，兼容常见别名（RINEX 单字母、全称等）。不认识的标识返回错误。
func ParseSystem(raw string) (System, bool) {
	t := strings.ToUpper(strings.TrimSpace(raw))
	switch t {
	case "":
		return GPS, true
	case "GPS", "G":
		return GPS, true
	case "GAL", "GALILEO", "E":
		return Galileo, true
	case "BDS", "BD", "BEIDOU", "COMPASS", "C":
		return BeiDou, true
	}
	return "", false
}

// SatKey 是一颗卫星的全局主键：所属系统 + 系统内编号。
type SatKey struct {
	Sys System `json:"sys"`
	ID  int    `json:"id"`
}

// Key 构造卫星主键。
func Key(sys System, id int) SatKey { return SatKey{Sys: sys, ID: id} }

// String 形如 "GPS:3"、"GAL:12"，用于日志与人类可读输出。
func (k SatKey) String() string { return string(k.Sys) + ":" + strconv.Itoa(k.ID) }

// MarshalText 使 SatKey 可作为 JSON map 键落盘。
func (k SatKey) MarshalText() ([]byte, error) { return []byte(k.String()), nil }

// UnmarshalText 兼容两种落盘形式：
//
//	"GAL:3" 新格式（系统:编号）
//	"3"     升级前旧格式（裸编号按 GPS）
func (k *SatKey) UnmarshalText(b []byte) error {
	s := strings.TrimSpace(string(b))
	if i := strings.IndexByte(s, ':'); i >= 0 {
		sys, ok := ParseSystem(s[:i])
		if !ok {
			return strconvErr(s)
		}
		id, err := strconv.Atoi(strings.TrimSpace(s[i+1:]))
		if err != nil {
			return strconvErr(s)
		}
		k.Sys, k.ID = sys, id
		return nil
	}
	id, err := strconv.Atoi(s)
	if err != nil {
		return strconvErr(s)
	}
	k.Sys, k.ID = GPS, id
	return nil
}

func strconvErr(s string) error {
	return &keyError{s: s}
}

type keyError struct{ s string }

func (e *keyError) Error() string { return "非法的卫星主键: " + e.s }

// Less 定义全序：先按 All 中的系统次序（GPS<GAL<BDS），再按编号。
// 用于任何需要稳定输出次序的场合（隔离列表、遍历解算）。
func Less(a, b SatKey) bool {
	ia, ib := sysIndex(a.Sys), sysIndex(b.Sys)
	if ia != ib {
		return ia < ib
	}
	return a.ID < b.ID
}

func sysIndex(s System) int {
	for i, x := range All {
		if x == s {
			return i
		}
	}
	return len(All)
}
