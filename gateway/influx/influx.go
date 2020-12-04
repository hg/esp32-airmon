package influx

import (
	"context"
	"errors"
	influxdb2 "github.com/influxdata/influxdb-client-go/v2"
	influxdb2Api "github.com/influxdata/influxdb-client-go/v2/api"
	influxdb2Write "github.com/influxdata/influxdb-client-go/v2/api/write"
	"log"
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
	writeApi := client.WriteAPIBlocking(settings.Org, settings.Bucket)
	return writeApi
}

func NewWriter(settings Settings) (chan<- *influxdb2Write.Point, error) {
	if err := settings.validate(); err != nil {
		return nil, err
	}

	writeApi := newClient(settings)
	writeCh := make(chan *influxdb2Write.Point, 2000)
	ctx := context.Background()

	go func() {
		for point := range writeCh {
			if err := writeApi.WritePoint(ctx, point); err == nil {
				log.Print("point written")
			} else {
				log.Print("could not write point: ", err)
			}
		}
	}()

	return writeCh, nil
}
