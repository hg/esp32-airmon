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

func ParseCO2(raw []byte) ([]data.Measure, error) {
	var ms co2
	if err := json.Unmarshal(raw, &ms); err != nil {
		return nil, err
	}
	rows := []data.Measure{
		{
			Post: data.Post{
				Source: data.Custom,
				Name:   ms.Device,
				Slug:   ms.Device,
			},
			Rows: []data.Observation{
				{
					Date: time.Unix(ms.Time, 0),
					Level: []data.Level{
						{Substance: "CO2", Unit: "ppm", Value: ms.Co2},
					},
				},
			},
		},
	}
	return rows, nil
}
