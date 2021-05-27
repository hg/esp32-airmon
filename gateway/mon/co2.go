package mon

import (
	"encoding/json"
	"time"

	influxdb2 "github.com/influxdata/influxdb-client-go/v2"
	influxdb2Write "github.com/influxdata/influxdb-client-go/v2/api/write"
)

type co2 struct {
	Device string `json:"dev"`
	Time   int64  `json:"time"`
	Sensor string `json:"sens"`
	Co2    uint16 `json:"co2"`
}

func (t *co2) ToPoint() *influxdb2Write.Point {
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

func ParseCarbonDioxide(data []byte) (PointSource, error) {
	ms := &co2{}
	err := json.Unmarshal(data, ms)
	return ms, err
}
