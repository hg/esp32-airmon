package mon

import (
	"encoding/json"
	"time"

	"github.com/hg/airmon/data"
)

type density struct {
	Pm1  float32 `json:"pm1"`
	Pm25 float32 `json:"pm2.5"`
	Pm10 float32 `json:"pm10"`
}

type particulates struct {
	Device string  `json:"dev"`
	Time   int64   `json:"time"`
	Sensor string  `json:"sens"`
	Std    density `json:"std"`
	Atm    density `json:"atm"`
	Count  struct {
		Pm03 uint16 `json:"pm0.3"`
		Pm05 uint16 `json:"pm0.5"`
		Pm1  uint16 `json:"pm1"`
		Pm25 uint16 `json:"pm2.5"`
		Pm5  uint16 `json:"pm5"`
		Pm10 uint16 `json:"pm10"`
	} `json:"cnt"`
}

func (t *particulates) Convert() []data.Measure {
	return []data.Measure{
		{
			Date: time.Unix(t.Time, 0),
			Post: &data.Post{
				Source: data.Custom,
				Name:   t.Device,
			},
			Level: []data.Level{
				{Substance: "PM1", Unit: "µg/m³", Value: t.Atm.Pm1},
				{Substance: "PM2.5", Unit: "µg/m³", Value: t.Atm.Pm25},
				{Substance: "PM10", Unit: "µg/m³", Value: t.Atm.Pm10},
			},
		},
	}
}

func ParseParticulates(data []byte) (DataSource, error) {
	ms := &particulates{}
	err := json.Unmarshal(data, ms)
	return ms, err
}
