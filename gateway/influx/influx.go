package influx

import (
	"context"
	"errors"
	influxdb2 "github.com/influxdata/influxdb-client-go/v2"
	influxdb2Api "github.com/influxdata/influxdb-client-go/v2/api"
	influxdb2Write "github.com/influxdata/influxdb-client-go/v2/api/write"
	"log"
	"time"
)

type Settings struct {
	Uri    string
	Org    string
	Bucket string
	Token  string
}

func (s *Settings) validate() error {
	if s.Uri == "" {
		return errors.New("InfluxDB URI not set")
	}
	if s.Org == "" {
		return errors.New("InfluxDB organization not set")
	}
	if s.Bucket == "" {
		return errors.New("InfluxDB bucket is empty")
	}
	if s.Token == "" {
		log.Print("InfluxDB token is empty, set it if you see authentication errors")
	}
	return nil
}

func newClient(settings Settings) influxdb2Api.WriteAPIBlocking {
	client := influxdb2.NewClient(settings.Uri, settings.Token)
	return client.WriteAPIBlocking(settings.Org, settings.Bucket)
}

type MeasurementSender struct {
	api influxdb2Api.WriteAPIBlocking
	ch  chan *influxdb2Write.Point
}

func (ms *MeasurementSender) Send(point *influxdb2Write.Point) bool {
	for retry := 0; ; retry++ {
		select {
		case ms.ch <- point:
			return true

		case <-time.After(5 * time.Second):
			log.Print("timed out while trying to send measurement")

			if retry >= 3 {
				log.Print("could not send measurement, discarding point")
				return false
			}
			<-ms.ch // drop the oldest measurement and retry
		}
	}
}

func (ms *MeasurementSender) receive() {
	ctx := context.Background()

	for point := range ms.ch {
		if err := ms.api.WritePoint(ctx, point); err == nil {
			log.Print("point written")
		} else {
			log.Print("could not write point: ", err)
		}
	}
}

func NewWriter(settings Settings) (*MeasurementSender, error) {
	if err := settings.validate(); err != nil {
		return nil, err
	}

	sender := &MeasurementSender{
		ch:  make(chan *influxdb2Write.Point, 2000),
		api: newClient(settings),
	}
	go sender.receive()

	return sender, nil
}
