// Package sim 为测试与回放演示提供确定性的 GNSS 仿真：
// 固定接收机真值、按方位角/仰角布置卫星、带固定随机种子的高斯噪声伪距，
// 支持给指定星注入伪距偏差与恒定钟差。
//
// 除原有的单模 GPS 星座（BuildConstellation/EvenSky/Observe，保持不变）外，
// 新增多模构造（BuildMultiConstellation/MultiSky）与 ObserveMulti：
// 每套系统各自布星，伪距里加入该系统时相对 GPS 时的偏差（GGTO）。
package sim

import (
	"math"
	"math/rand"
	"sort"

	"raim/pkg/geo"
	"raim/pkg/gnss"
	"raim/pkg/lsq"
)

// Receiver 是仿真接收机真值。
type Receiver struct {
	LLA       geo.LLA
	Pos       geo.Vec // ECEF 真值
	ClockBias float64 // 米
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
	Azimuth float64 // 度（北为 0，顺时针）
	Elev    float64 // 度
	Range   float64 // 接收机到卫星距离（米），默认 21500km
}

// Constellation 是一组卫星的 ECEF 位置。
type Constellation struct {
	Rec       *Receiver
	Sats      map[int]geo.Vec        // 单模（GPS）星座
	MultiSats map[gnss.SatID]geo.Vec // 多模星座（含系统归属）
}

// BuildConstellation 按方位角/仰角布星。rangeKm<=0 取 21500 km。
func BuildConstellation(rec *Receiver, defs []SatelliteDef) *Constellation {
	c := &Constellation{
		Rec:       rec,
		Sats:      map[int]geo.Vec{},
		MultiSats: map[gnss.SatID]geo.Vec{},
	}
	for _, d := range defs {
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
		pos := geo.Add(rec.Pos, ENUToECEF(enu, rec.LLA.Lon, rec.LLA.Lat))
		c.Sats[d.ID] = pos
		c.MultiSats[gnss.Key(gnss.GPS, d.ID)] = pos
	}
	return c
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
// 同一 seed 每次结果一致。
func EvenSky(rec *Receiver, n int, minElevDeg float64, seed int64) *Constellation {
	rng := rand.New(rand.NewSource(seed))
	defs := make([]SatelliteDef, n)
	for i := 0; i < n; i++ {
		az := math.Mod(float64(i)*360.0/float64(n)+rng.Float64()*30, 360)
		el := minElevDeg + rng.Float64()*(85-minElevDeg)
		defs[i] = SatelliteDef{ID: i + 1, Azimuth: az, Elev: el}
	}
	return BuildConstellation(rec, defs)
}

// MultiSatDef 是一颗带系统归属的布星定义。
type MultiSatDef struct {
	System  gnss.System
	ID      int
	Azimuth float64
	Elev    float64
	Range   float64
}

// BuildMultiConstellation 按系统布多模星座（允许跨系统同编号）。
func BuildMultiConstellation(rec *Receiver, defs []MultiSatDef) *Constellation {
	c := &Constellation{
		Rec:       rec,
		Sats:      map[int]geo.Vec{},
		MultiSats: map[gnss.SatID]geo.Vec{},
	}
	for _, d := range defs {
		az, el := d.Azimuth*geo.RadPerDeg, d.Elev*geo.RadPerDeg
		r := d.Range
		if r <= 0 {
			r = 21_500_000
		}
		east := math.Cos(el) * math.Sin(az)
		north := math.Cos(el) * math.Cos(az)
		up := math.Sin(el)
		enu := geo.Vec{X: east * r, Y: north * r, Z: up * r}
		pos := geo.Add(rec.Pos, ENUToECEF(enu, rec.LLA.Lon, rec.LLA.Lat))
		c.MultiSats[gnss.Key(d.System, d.ID)] = pos
		if d.System == gnss.GPS {
			c.Sats[d.ID] = pos // 同时填旧表，GPS 场景仍可走 Observe
		}
	}
	return c
}

// MultiSky 为给定系统集合分别均布 perSys 颗星。
// 每个系统用独立的确定性随机流（seed + 系统序号），因此跨系统同编号不会撞几何。
func MultiSky(rec *Receiver, systems []gnss.System, perSys int, minElevDeg float64, seed int64) *Constellation {
	var defs []MultiSatDef
	for si, sys := range systems {
		rng := rand.New(rand.NewSource(seed*1000 + int64(si+1)))
		for i := 0; i < perSys; i++ {
			az := math.Mod(float64(i)*360.0/float64(perSys)+rng.Float64()*30, 360)
			el := minElevDeg + rng.Float64()*(85-minElevDeg)
			defs = append(defs, MultiSatDef{
				System: sys, ID: i + 1, Azimuth: az, Elev: el,
			})
		}
	}
	return BuildMultiConstellation(rec, defs)
}

// MultiObs 是一次多模带噪观测的参数。
type MultiObs struct {
	Rng    *rand.Rand
	Sigma  float64                 // 所有星统一误差标准差（米）
	Clock  *float64                // 若非 nil，覆盖接收机钟差（GPS 时基准，米）
	GGTO   map[gnss.System]float64 // 非 GPS 系统时相对 GPS 时的偏差（米）
	Bias   map[gnss.SatID]float64  // 给指定（系统,编号）星注入的伪距偏差（米）
	Hidden []gnss.SatID            // 本历元不可见的星
}

// sortedMultiKeys 按系统规范顺序、再按编号返回多模星座的键（确定性）。
func sortedMultiKeys(m map[gnss.SatID]geo.Vec) []gnss.SatID {
	keys := make([]gnss.SatID, 0, len(m))
	for k := range m {
		keys = append(keys, k)
	}
	sort.Slice(keys, func(i, j int) bool { return gnss.Compare(keys[i], keys[j]) < 0 })
	return keys
}

// ObserveMulti 生成一个历元的多模 lsq 输入。
// 每颗星的伪距 = 几何距离 + 接收机钟差（GPS）+ GGTO（非 GPS）+ 注入偏差 + 噪声。
func (c *Constellation) ObserveMulti(o MultiObs) *lsq.Epoch {
	sigma := o.Sigma
	if sigma <= 0 {
		sigma = 1.0
	}
	clock := c.Rec.ClockBias
	if o.Clock != nil {
		clock = *o.Clock
	}
	hidden := map[gnss.SatID]bool{}
	for _, k := range o.Hidden {
		hidden[k] = true
	}
	var sats []lsq.Satellite
	for _, k := range sortedMultiKeys(c.MultiSats) {
		if hidden[k] {
			continue
		}
		sp := c.MultiSats[k]
		r := geo.Norm(geo.Sub(sp, c.Rec.Pos))
		sysOff := 0.0
		if k.System != gnss.GPS {
			sysOff = o.GGTO[k.System]
		}
		pr := r + clock + sysOff
		if b, ok := o.Bias[k]; ok {
			pr += b
		}
		if o.Rng != nil {
			pr += o.Rng.NormFloat64() * sigma
		}
		sats = append(sats, lsq.Satellite{System: k.System, ID: k.ID, Pos: sp, PR: pr, Sigma: sigma})
	}
	return &lsq.Epoch{Approx: approxOf(c.Rec.Pos), Sats: sats}
}

// MultiIDs 按规范顺序返回多模星座的全部（系统,编号）。
func (c *Constellation) MultiIDs() []gnss.SatID {
	return sortedMultiKeys(c.MultiSats)
}

// Obs 是一次带噪观测的全部参数。
type Obs struct {
	Rng    *rand.Rand
	Sigma  float64         // 所有星统一误差标准差（米）
	Bias   map[int]float64 // 给指定星注入的伪距偏差（米）
	Clock  *float64        // 若非 nil，覆盖接收机钟差（米）
	Hidden []int           // 本历元不可见的星
}

// Observe 生成一个历元的 lsq 输入。
func (c *Constellation) Observe(o Obs) *lsq.Epoch {
	sigma := o.Sigma
	if sigma <= 0 {
		sigma = 1.0
	}
	clock := c.Rec.ClockBias
	if o.Clock != nil {
		clock = *o.Clock
	}
	hidden := map[int]bool{}
	for _, id := range o.Hidden {
		hidden[id] = true
	}
	var sats []lsq.Satellite
	for id := 1; id <= len(c.Sats); id++ {
		if hidden[id] {
			continue
		}
		sp, ok := c.Sats[id]
		if !ok {
			continue
		}
		r := geo.Norm(geo.Sub(sp, c.Rec.Pos))
		pr := r + clock
		if b, ok := o.Bias[id]; ok {
			pr += b
		}
		if o.Rng != nil {
			pr += o.Rng.NormFloat64() * sigma
		}
		sats = append(sats, lsq.Satellite{ID: id, Pos: sp, PR: pr, Sigma: sigma})
	}
	return &lsq.Epoch{Approx: approxOf(c.Rec.Pos), Sats: sats}
}

// approxOf 在真值附近给一个有偏的概略位置（~3 km），模拟冷启动初值。
func approxOf(true geo.Vec) geo.Vec {
	return geo.Add(true, geo.Vec{X: 2000, Y: -1500, Z: 1000})
}

// TruePos 返回真值 ECEF（供测试比对）。
func (c *Constellation) TruePos() geo.Vec { return c.Rec.Pos }

// IDs 返回星座星编号。
func (c *Constellation) IDs() []int {
	out := make([]int, 0, len(c.Sats))
	for id := range c.Sats {
		out = append(out, id)
	}
	return out
}
