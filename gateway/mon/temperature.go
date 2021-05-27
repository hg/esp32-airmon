package mon

import (
	"encoding/json"
	"time"

	influxdb2 "github.com/influxdata/influxdb-client-go/v2"
	influxdb2Write "github.com/influxdata/influxdb-client-go/v2/api/write"
)

type temperature struct {
	Device      string  `json:"dev"`
	Time        int64   `json:"time"`
	Sensor      string  `json:"sens"`
	Temperature float64 `json:"temp"`
}

func (t *temperature) ToPoint() *influxdb2Write.Point {
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

func ParseTemperature(data []byte) (PointSource, error) {
	ms := &temperature{}
	err := json.Unmarshal(data, ms)
	return ms, err
}
