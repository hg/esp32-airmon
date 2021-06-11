package ceb

import "testing"

const station = "fake-station"
const co2 = "co2"
const so2 = "so2"

func TestStaleDetector(t *testing.T) {
	d := newDetector()

	for i := 0; i < staleMaxUpdates; i++ {
		if d.isStale(station, co2, 100) {
			t.Fatal("value should not be stale")
		}
	}

	if d.isStale(station, so2, 50) {
		t.Fatal("value uses other pollutant and should not be stale")
	}

	for i := 0; i < staleMaxUpdates*2; i++ {
		if d.isStale(station, so2, float64(i)) {
			t.Fatal("value is changing and should not be stale")
		}
	}

	if !d.isStale(station, co2, 100) {
		t.Fatal("value should be stale")
	}

	if d.isStale(station, co2, 42) {
		t.Fatal("value has changed and should not be stale")
	}
}
