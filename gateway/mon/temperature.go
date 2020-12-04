package mon

import (
	"encoding/json"
	influxdb2 "github.com/influxdata/influxdb-client-go/v2"
	influxdb2Write "github.com/influxdata/influxdb-client-go/v2/api/write"
	"log"
	"time"
)

type temperature struct {
	Device      string  `json:"dev"`
	Time        int64   `json:"time"`
	Sensor      string  `json:"sens"`
	Temperature float64 `json:"temp"`
}

func (t *temperature) toPoint() *influxdb2Write.Point {
	return influxdb2.NewPoint("temperature",
		map[string]string{
			"device": t.Device,
			"sensor": t.Sensor,
		},
		map[string]interface{}{
			"temperature": t.Temperature,
		},
		time.Unix(t.Time, 0))
}

func ParseTemperature(data []byte) (*influxdb2Write.Point, error) {
	var temp temperature
	if err := json.Unmarshal(data, &temp); err != nil {
		log.Print("could not parse temp json: ", err)
		return nil, err
	}
	return temp.toPoint(), nil
}
