package mon

import (
	"encoding/json"
	"time"

	"github.com/hg/airmon/data"
)

type co2 struct {
	Device string  `json:"dev"`
	Time   int64   `json:"time"`
	Sensor string  `json:"sens"`
	Co2    float32 `json:"co2"`
}

func (t *co2) Convert() []data.Measure {
	return []data.Measure{
		{
			Date: time.Unix(t.Time, 0),
			Post: &data.Post{
				Source: data.Custom,
				Name:   t.Device,
			},
			Level: []data.Level{
				{Substance: "CO2", Unit: "ppm", Value: t.Co2},
			},
		},
	}
}

func ParseCarbonDioxide(data []byte) (DataSource, error) {
	ms := &co2{}
	err := json.Unmarshal(data, ms)
	return ms, err
}
