package mon

import (
	"reflect"
	"testing"
	"time"

	"github.com/hg/airmon/data"
)

var wantTemp = []data.Measure{
	{
		Post: data.Post{Source: data.Custom, Name: "foo", Slug: "foo"},
		Rows: []data.Observation{
			{
				Date: time.Date(2000, 1, 1, 12, 34, 56, 0, time.UTC).Local(),
				Level: []data.Level{
					{Substance: "TEMP", Unit: "°C", Value: -12.34},
				},
			},
		},
	},
}

func TestParseTemp(t *testing.T) {
	ms, err := ParseTemp(nil)
	if err == nil {
		t.Fatal("expected error, got nil")
	}
	b := []byte(`{"dev":"foo","time":946730096,"sensor":"bar","temp":-12.34}`)
	ms, err = ParseTemp(b)
	if err != nil {
		t.Fatalf("expected nil error, got %v", err)
	}
	if !reflect.DeepEqual(ms, wantTemp) {
		t.Fatalf("parsed data did not match: %+v", ms)
	}
}
