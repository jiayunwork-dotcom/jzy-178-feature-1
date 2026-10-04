// Package sim 为测试与回放演示提供确定性的 GNSS 仿真：
// 固定接收机真值、按方位角/仰角布置卫星、带固定随机种子的高斯噪声伪距，
// 支持给指定星注入伪距偏差与恒定钟差，并支持多系统（GPS/GAL/BDS）共视：
// 每套系统可有自己的系统时偏差（ISB），各系统按独立星编号布置。
package sim

import (
	"math"
	"math/rand"

	"raim/pkg/geo"
	"raim/pkg/gnss"
	"raim/pkg/lsq"
)

// Receiver 是仿真接收机真值。
type Receiver struct {
	LLA       geo.LLA
	Pos       geo.Vec // ECEF 真值
	ClockBias float64 // 米（相对 GPS 时）
}

// NewReceiver 给定经纬度高（度/米）与钟差（米）。
func NewReceiver(lon, lat, alt, clock float64) *Receiver {
	g := geo.LLA{Lon: lon, Lat: lat, Alt: alt}
	return &Receiver{LLA: g, Pos: geo.LLAToECEF(g), ClockBias: clock}
}

// DefaultReceiver：北京附近，零钟差。
func DefaultReceiver() *Receiver { return NewReceiver(116.39, 39.90, 50, 0) }

// SatelliteDef 描述一颗几何布置的卫星。
type SatelliteDef struct {
	ID      int
	Sys     gnss.System // 空按 GPS
	Azimuth float64     // 度（北为 0，顺时针）
	Elev    float64     // 度
	Range   float64     // 接收机到卫星距离（米），默认 21500km
}

// Constellation 是一组卫星的 ECEF 位置。
type Constellation struct {
	Rec  *Receiver
	Sats map[int]geo.Vec // GPS 星（升级前字段，仍为全部 GPS 编号）
	// All 按“系统+编号”保存全部卫星（含非 GPS）；Defs 为构造时的顺序定义。
	All  map[gnss.SatKey]geo.Vec
	Defs []SatelliteDef
}

// BuildConstellation 按方位角/仰角布星（升级前接口：全部按 GPS）。rangeKm<=0 取 21500 km。
func BuildConstellation(rec *Receiver, defs []SatelliteDef) *Constellation {
	c := &Constellation{
		Rec:  rec,
		Sats: map[int]geo.Vec{},
		All:  map[gnss.SatKey]geo.Vec{},
	}
	for _, d := range defs {
		sys := d.Sys
		if sys == "" {
			sys = gnss.GPS
		}
		dd := d
		dd.Sys = sys
		c.Defs = append(c.Defs, dd)
		p := place(rec, d)
		c.All[gnss.Key(sys, d.ID)] = p
		if sys == gnss.GPS {
			c.Sats[d.ID] = p
		}
	}
	return c
}

func place(rec *Receiver, d SatelliteDef) geo.Vec {
	az, el := d.Azimuth*geo.RadPerDeg, d.Elev*geo.RadPerDeg
	r := d.Range
	if r <= 0 {
		r = 21_500_000
	}
	// ENU 方向向量
	east := math.Cos(el) * math.Sin(az)
	north := math.Cos(el) * math.Cos(az)
	up := math.Sin(el)
	enu := geo.Vec{X: east * r, Y: north * r, Z: up * r}
	return geo.Add(rec.Pos, ENUToECEF(enu, rec.LLA.Lon, rec.LLA.Lat))
}

// ENUToECEF 把站心相对向量旋转回 ECEF。
func ENUToECEF(v geo.Vec, lonDeg, latDeg float64) geo.Vec {
	R := geo.ENURotation(lonDeg, latDeg)
	// ENU = R·ECEF ⇒ ECEF = Rᵀ·ENU（R 正交）
	var out geo.Vec
	out.X = R[0][0]*v.X + R[1][0]*v.Y + R[2][0]*v.Z
	out.Y = R[0][1]*v.X + R[1][1]*v.Y + R[2][1]*v.Z
	out.Z = R[0][2]*v.X + R[1][2]*v.Y + R[2][2]*v.Z
	return out
}

// EvenSky 在给定最低仰角下均布 n 颗星（低仰角星多 ⇒ 高 HDOP）。
// 同一 seed 每次结果一致（升级前接口：全部按 GPS）。
func EvenSky(rec *Receiver, n int, minElevDeg float64, seed int64) *Constellation {
	rng := rand.New(rand.NewSource(seed))
	defs := make([]SatelliteDef, n)
	for i := 0; i < n; i++ {
		az := math.Mod(float64(i)*360.0/float64(n)+rng.Float64()*30, 360)
		el := minElevDeg + rng.Float64()*(85-minElevDeg)
		defs[i] = SatelliteDef{ID: i + 1, Sys: gnss.GPS, Azimuth: az, Elev: el}
	}
	return BuildConstellation(rec, defs)
}

// MultiSky 为每套系统各均布 n 颗星，系统间方位错开，避免几何雷同。
// 同一 seed 每次结果一致。
func MultiSky(rec *Receiver, perSys map[gnss.System]int, minElevDeg float64, seed int64) *Constellation {
	rng := rand.New(rand.NewSource(seed))
	var defs []SatelliteDef
	sysIdx := 0
	for _, sys := range gnss.All {
		n := perSys[sys]
		if n <= 0 {
			continue
		}
		offset := float64(sysIdx) * (360.0 / 6.0) // 系统间方位错开 60° 量级
		for i := 0; i < n; i++ {
			az := math.Mod(float64(i)*360.0/float64(n)+offset+rng.Float64()*20, 360)
			el := minElevDeg + rng.Float64()*(85-minElevDeg)
			defs = append(defs, SatelliteDef{ID: i + 1, Sys: sys, Azimuth: az, Elev: el})
		}
		sysIdx++
	}
	return BuildConstellation(rec, defs)
}

// Keys 按系统次序（GPS<GAL<BDS）再按编号返回全部卫星主键，保证遍历确定。
func (c *Constellation) Keys() []gnss.SatKey {
	out := make([]gnss.SatKey, 0, len(c.All))
	for k := range c.All {
		out = append(out, k)
	}
	sortKeys(out)
	return out
}

func sortKeys(keys []gnss.SatKey) {
	for i := 1; i < len(keys); i++ {
		for j := i; j > 0 && gnss.Less(keys[j], keys[j-1]); j-- {
			keys[j], keys[j-1] = keys[j-1], keys[j]
		}
	}
}

// Obs 是一次带噪观测的全部参数。
type Obs struct {
	Rng      *rand.Rand
	Sigma    float64                 // 所有星统一误差标准差（米）
	Bias     map[int]float64         // （升级前）给指定 GPS 星注入的伪距偏差（米）
	BiasK    map[gnss.SatKey]float64 // 按“系统+编号”给指定星注入的伪距偏差（米）
	Clock    *float64                // 若非 nil，覆盖 GPS 接收机钟差（米）
	SysClock map[gnss.System]float64 // 各系统相对 GPS 时的系统时偏差（米）
	Hidden   []int                   // （升级前）本历元不可见的 GPS 星
	HiddenK  []gnss.SatKey           // 本历元不可见的星（任意系统）
}

func (o Obs) biasAt(k gnss.SatKey) (float64, bool) {
	if o.BiasK != nil {
		if b, ok := o.BiasK[k]; ok {
			return b, true
		}
	}
	if k.Sys == gnss.GPS && o.Bias != nil {
		if b, ok := o.Bias[k.ID]; ok {
			return b, true
		}
	}
	return 0, false
}

func (o Obs) hidden(k gnss.SatKey) bool {
	for _, h := range o.HiddenK {
		if h == k {
			return true
		}
	}
	if k.Sys == gnss.GPS {
		for _, h := range o.Hidden {
			if h == k.ID {
				return true
			}
		}
	}
	return false
}

// Observe 生成一个历元的 lsq 输入。
func (c *Constellation) Observe(o Obs) *lsq.Epoch {
	sigma := o.Sigma
	if sigma <= 0 {
		sigma = 1.0
	}
	gpsClock := c.Rec.ClockBias
	if o.Clock != nil {
		gpsClock = *o.Clock
	}
	var sats []lsq.Satellite
	for _, k := range c.Keys() {
		if o.hidden(k) {
			continue
		}
		sp := c.All[k]
		r := geo.Norm(geo.Sub(sp, c.Rec.Pos))
		clock := gpsClock + o.SysClock[k.Sys]
		pr := r + clock
		if b, ok := o.biasAt(k); ok {
			pr += b
		}
		if o.Rng != nil {
			pr += o.Rng.NormFloat64() * sigma
		}
		sats = append(sats, lsq.Satellite{ID: k.ID, Sys: k.Sys, Pos: sp, PR: pr, Sigma: sigma})
	}
	return &lsq.Epoch{Approx: approxOf(c.Rec.Pos), Sats: sats}
}

// approxOf 在真值附近给一个有偏的概略位置（~3 km），模拟冷启动初值。
func approxOf(true geo.Vec) geo.Vec {
	return geo.Add(true, geo.Vec{X: 2000, Y: -1500, Z: 1000})
}

// TruePos 返回真值 ECEF（供测试比对）。
func (c *Constellation) TruePos() geo.Vec { return c.Rec.Pos }

// IDs 返回 GPS 星座星编号（升级前接口）。
func (c *Constellation) IDs() []int {
	out := make([]int, 0, len(c.Sats))
	for id := range c.Sats {
		out = append(out, id)
	}
	return out
}
