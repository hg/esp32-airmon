package mon

import (
	"encoding/json"
	influxdb2 "github.com/influxdata/influxdb-client-go/v2"
	influxdb2Write "github.com/influxdata/influxdb-client-go/v2/api/write"
	"log"
	"time"
)

type pmConcentration struct {
	Pm1   uint16 `json:"pm1"`
	Pm2_5 uint16 `json:"pm2.5"`
	Pm10  uint16 `json:"pm10"`
}

type particulates struct {
	Device      string          `json:"dev"`
	Time        int64           `json:"time"`
	Sensor      string          `json:"sens"`
	Standard    pmConcentration `json:"std"`
	Atmospheric pmConcentration `json:"atm"`
	Count       struct {
		Pm03  uint16 `json:"pm0.3"`
		Pm05  uint16 `json:"pm0.5"`
		Pm1   uint16 `json:"pm1"`
		Pm2_5 uint16 `json:"pm2.5"`
		Pm5   uint16 `json:"pm5"`
		Pm10  uint16 `json:"pm10"`
	} `json:"cnt"`
}

func (t *particulates) toPoint() *influxdb2Write.Point {
	return influxdb2.NewPoint("particulates",
		map[string]string{
			"device": t.Device,
			"sensor": t.Sensor,
		},
		map[string]interface{}{
			"ug_std_1":   t.Standard.Pm1,
			"ug_std_2_5": t.Standard.Pm2_5,
			"ug_std_10":  t.Standard.Pm10,
			"ug_atm_1":   t.Atmospheric.Pm1,
			"ug_atm_2_5": t.Atmospheric.Pm2_5,
			"ug_atm_10":  t.Atmospheric.Pm10,
			"cnt_0_3":    t.Count.Pm03,
			"cnt_0_5":    t.Count.Pm05,
			"cnt_1":      t.Count.Pm1,
			"cnt_2_5":    t.Count.Pm2_5,
			"cnt_5":      t.Count.Pm5,
			"cnt_10":     t.Count.Pm10,
		},
		time.Unix(t.Time, 0))
}

func ParseParticulates(data []byte) (*influxdb2Write.Point, error) {
	var part particulates
	if err := json.Unmarshal(data, &part); err != nil {
		log.Print("could not parse particulates json: ", err)
		return nil, err
	}
	return part.toPoint(), nil
}
