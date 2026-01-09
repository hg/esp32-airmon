package mon

import (
	"encoding/json"
	"time"

	"github.com/hg/airmon/data"
	"github.com/hg/airmon/db"
)

type density struct {
	Pm1  float32 `json:"pm1"`
	Pm25 float32 `json:"pm2.5"`
	Pm10 float32 `json:"pm10"`
}

type parts struct {
	Device string  `json:"dev"`
	Time   int64   `json:"time"`
	Sensor string  `json:"sens"`
	Atm    density `json:"atm"`
}

func ParsePM(st *db.Storage, raw []byte) ([]data.Measure, error) {
	var ms parts
	if err := json.Unmarshal(raw, &ms); err != nil {
		return nil, err
	}

	postID, err := st.GetPost(data.Post{
		Source: data.Custom,
		Name:   ms.Device,
		Slug:   ms.Device,
	})
	if err != nil {
		return nil, err
	}

	rows := []data.Measure{
		{
			PostID: postID,
			Rows: []data.Observation{
				{
					Date: time.Unix(ms.Time, 0),
					Level: []data.Level{
						{Substance: "PM1", Unit: "µg/m³", Value: ms.Atm.Pm1},
						{Substance: "PM2.5", Unit: "µg/m³", Value: ms.Atm.Pm25},
						{Substance: "PM10", Unit: "µg/m³", Value: ms.Atm.Pm10},
					},
				},
			},
		},
	}
	return rows, nil
}
