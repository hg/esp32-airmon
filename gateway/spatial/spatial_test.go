package spatial

import (
	"math"
	"testing"
)

func near(a, b, tolerance float64) bool {
	return math.Abs(a-b) <= tolerance
}

func TestHaversine(t *testing.T) {
	tests := []struct {
		name      string
		one       Point
		two       Point
		want      float64
		tolerance float64
	}{
		{
			name:      "same point",
			one:       Point{Lat: 0, Lon: 0},
			two:       Point{Lat: 0, Lon: 0},
			want:      0,
			tolerance: 0.01,
		},
		{
			name:      "one degree latitude",
			one:       Point{Lat: 0, Lon: 0},
			two:       Point{Lat: 1, Lon: 0},
			want:      111_195,
			tolerance: 10,
		},
		{
			name:      "one degree longitude",
			one:       Point{Lat: 0, Lon: 0},
			two:       Point{Lat: 0, Lon: 1},
			want:      111_195,
			tolerance: 10,
		},
		{
			name:      "astana to almaty",
			one:       Point{Lat: 51.1605, Lon: 71.4265}, // Astana
			two:       Point{Lat: 43.2384, Lon: 76.8729}, // Almaty
			want:      971_610,
			tolerance: 10,
		},
		{
			name:      "antipodal points",
			one:       Point{Lat: 0, Lon: 0},
			two:       Point{Lat: 0, Lon: 180},
			want:      math.Pi * 6_371_000,
			tolerance: 10,
		},
	}

	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			d := Haversine(tt.one, tt.two)
			if !near(d, tt.want, tt.tolerance) {
				t.Errorf("Haversine(%v, %v) = %.2f, want %.2f",
					tt.one, tt.two, d, tt.want)
			}
		})
	}
}

func TestParsePoint(t *testing.T) {
	tests := []struct {
		raw string
		lat float64
		lon float64
		err bool
	}{
		{raw: "12.345,-42.1337", lat: 12.345, lon: -42.1337},
		{raw: "-90,180", lat: -90, lon: 180},
		{raw: "24, 12", lat: 24, lon: 12},
		{raw: "10,20,30", lat: 10, lon: 20},
		{raw: "-90,180p", err: true},
		{raw: "-91,0", err: true},
		{raw: "91,0", err: true},
		{raw: "0,-181", err: true},
		{raw: "0,181", err: true},
		{raw: "abc,def", err: true},
		{raw: "10 20", err: true},
		{raw: "", err: true},
	}

	for _, tt := range tests {
		t.Run(tt.raw, func(t *testing.T) {
			got, err := ParsePoint(tt.raw)
			if tt.err {
				if err == nil {
					t.Fatalf("want error, got nil (point=%+v)", got)
				}
				return
			}
			if err != nil {
				t.Fatalf("unexpected error: %v", err)
			}
			if got.Lat != tt.lat || got.Lon != tt.lon {
				t.Fatalf("want (%f,%f), got (%f,%f)",
					tt.lat, tt.lon, got.Lat, got.Lon)
			}
		})
	}
}
