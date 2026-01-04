package mon

import (
	"reflect"
	"testing"
	"time"

	"github.com/hg/airmon/data"
)

var wantCO2 = []data.Measure{
	{
		Post: data.Post{Source: data.Custom, Name: "bar", Slug: "bar"},
		Rows: []data.Observation{
			{
				Date: time.Date(2020, 1, 1, 12, 34, 56, 0, time.UTC).Local(),
				Level: []data.Level{
					{Substance: "CO2", Unit: "ppm", Value: 530.42},
				},
			},
		},
	},
}

func TestParseCO2(t *testing.T) {
	ms, err := ParseTemp(nil)
	if err == nil {
		t.Fatal("expected error, got nil")
	}
	b := []byte(`{"dev":"bar","time":1577882096,"sensor":"quux","co2":530.42}`)
	ms, err = ParseCO2(b)
	if err != nil {
		t.Fatalf("expected nil error, got %v", err)
	}
	if !reflect.DeepEqual(ms, wantCO2) {
		t.Fatalf("parsed data did not match: %+v", ms)
	}
}
