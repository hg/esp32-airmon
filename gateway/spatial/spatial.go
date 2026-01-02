package spatial

import (
	"math"
)

type Point struct {
	Lat, Lon float32
}

func (pt *Point) IsValid() bool {
	return pt.Lat >= -90 && pt.Lat <= 90 &&
		pt.Lon >= -180 && pt.Lon <= 180
}

const earthRadiusMeters = 6_371_000.0

func radians(deg float32) float64 {
	return float64(deg) * math.Pi / 180.0
}

func Haversine(one, two Point) float64 {
	lat1, lon1 := radians(one.Lat), radians(one.Lon)
	lat2, lon2 := radians(two.Lat), radians(two.Lon)

	dLat := lat2 - lat1
	dLon := lon2 - lon1

	sLat := math.Sin(dLat / 2)
	sLon := math.Sin(dLon / 2)

	a := sLat*sLat + math.Cos(lat1)*math.Cos(lat2)*sLon*sLon
	c := 2 * math.Atan2(math.Sqrt(a), math.Sqrt(1-a))

	return earthRadiusMeters * c
}
