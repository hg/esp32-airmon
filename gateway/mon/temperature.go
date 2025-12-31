package mon

import (
	"encoding/json"
	"time"

	"github.com/hg/airmon/data"
)

type temperature struct {
	Device      string  `json:"dev"`
	Time        int64   `json:"time"`
	Sensor      string  `json:"sens"`
	Temperature float32 `json:"temp"`
}

func ParseTemp(raw []byte) ([]data.Measure, error) {
	var ms temperature
	if err := json.Unmarshal(raw, &ms); err != nil {
		return nil, err
	}
	rows := []data.Measure{
		{
			Post: data.Post{
				Source: data.Custom,
				Name:   ms.Device,
			},
			Rows: []data.Observation{
				{
					Date: time.Unix(ms.Time, 0),
					Level: []data.Level{
						{Substance: "TEMP", Unit: "°C", Value: ms.Temperature},
					},
				},
			},
		},
	}
	return rows, nil
}
