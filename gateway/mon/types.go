package mon

import (
	influxdb2Write "github.com/influxdata/influxdb-client-go/v2/api/write"
)

type PointSource interface {
	ToPoint() *influxdb2Write.Point
}
