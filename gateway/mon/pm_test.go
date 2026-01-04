package mon

import (
	"reflect"
	"testing"
	"time"

	"github.com/hg/airmon/data"
)

var wantPM = []data.Measure{
	{
		Post: data.Post{Source: data.Custom, Name: "quux", Slug: "quux"},
		Rows: []data.Observation{
			{
				Date: time.Date(2020, 1, 1, 12, 34, 56, 0, time.UTC).Local(),
				Level: []data.Level{
					{Substance: "PM1", Unit: "µg/m³", Value: 42.3},
					{Substance: "PM2.5", Unit: "µg/m³", Value: 123.45},
					{Substance: "PM10", Unit: "µg/m³", Value: 200.33},
				},
			},
		},
	},
}

func TestParsePM(t *testing.T) {
	ms, err := ParseTemp(nil)
	if err == nil {
		t.Fatal("expected error, got nil")
	}
	b := []byte(`{"dev":"quux","time":1577882096,"sensor":"test","atm":{"pm1":42.3,"pm2.5":123.45,"pm10":200.33}}`)
	ms, err = ParsePM(b)
	if err != nil {
		t.Fatalf("expected nil error, got %v", err)
	}
	if !reflect.DeepEqual(ms, wantPM) {
		t.Fatalf("parsed data did not match: %+v", ms)
	}
}
