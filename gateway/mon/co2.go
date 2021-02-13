package mon

import (
	"encoding/json"
	influxdb2 "github.com/influxdata/influxdb-client-go/v2"
	influxdb2Write "github.com/influxdata/influxdb-client-go/v2/api/write"
	"log"
	"time"
)

type co2 struct {
	Device string `json:"dev"`
	Time   int64  `json:"time"`
	Sensor string `json:"sens"`
	Co2    uint16 `json:"co2"`
}

func (t *co2) toPoint() *influxdb2Write.Point {
	return influxdb2.NewPoint("co2",
		map[string]string{
			"device": t.Device,
			"sensor": t.Sensor,
		},
		map[string]interface{}{
			"co2": t.Co2,
		},
		time.Unix(t.Time, 0))
}

func ParseCarbonDioxide(data []byte) (*influxdb2Write.Point, error) {
	var co2 co2
	if err := json.Unmarshal(data, &co2); err != nil {
		log.Print("could not parse co2 json: ", err)
		return nil, err
	}
	return co2.toPoint(), nil
}
