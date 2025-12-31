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

func (t *temperature) Convert() []data.Measure {
	return []data.Measure{
		{
			Date: time.Unix(t.Time, 0),
			Post: &data.Post{
				Source: data.Custom,
				Name:   t.Device,
			},
			Level: []data.Level{
				{Substance: "TEMP", Unit: "°C", Value: t.Temperature},
			},
		},
	}
}

func ParseTemperature(data []byte) (DataSource, error) {
	ms := &temperature{}
	err := json.Unmarshal(data, ms)
	return ms, err
}
