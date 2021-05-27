package influx

import (
	"context"
	"errors"
	"flag"
	"os"
	"time"

	"github.com/hg/airmon/logger"
	influxdb2 "github.com/influxdata/influxdb-client-go/v2"
	influxdb2Api "github.com/influxdata/influxdb-client-go/v2/api"
	influxdb2Write "github.com/influxdata/influxdb-client-go/v2/api/write"
	"go.uber.org/zap"
)

var log = logger.Get(logger.Influx)

type Settings struct {
	Uri    string
	Org    string
	Bucket string
	Token  string
}

func (s *Settings) AddFlags() {
	flag.StringVar(&s.Uri, "influx.uri", "http://localhost:8086", "InfluxDB server URI")
	flag.StringVar(&s.Org, "influx.org", "home", "InfluxDB organization name")
	flag.StringVar(&s.Bucket, "influx.bucket", "airmon", "InfluxDB server URI")
	flag.StringVar(&s.Token, "influx.token", "", "InfluxDB access token")
}

func (s *Settings) SetFromEnvironment() {
	if s.Uri == "" {
		s.Uri = os.Getenv("INFLUX_URI")
	}
	if s.Org == "" {
		s.Org = os.Getenv("INFLUX_ORG")
	}
	if s.Bucket == "" {
		s.Bucket = os.Getenv("INFLUX_BUCKET")
	}
	if s.Token == "" {
		s.Token = os.Getenv("INFLUX_TOKEN")
	}
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
		log.Warn("token is empty, set it if you see authentication errors")
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
	ctx context.Context
}

func (ms *MeasurementSender) Send(point *influxdb2Write.Point) bool {
	for retry := 0; ; retry++ {
		select {
		case ms.ch <- point:
			return true

		case <-time.After(5 * time.Second):
			log.Error("timed out while trying to send measurement")

			if retry >= 3 {
				log.Error("could not send measurement, discarding point")
				return false
			}
			<-ms.ch // drop the oldest measurement and retry
		}
	}
}

func (ms *MeasurementSender) receive() {
	for point := range ms.ch {
		if err := ms.api.WritePoint(ms.ctx, point); err == nil {
			log.Debug("point written")
		} else {
			log.Error("could not write point", zap.Error(err))
		}
	}
}

func NewSender(settings Settings) (*MeasurementSender, error) {
	if err := settings.validate(); err != nil {
		return nil, err
	}

	sender := &MeasurementSender{
		ch:  make(chan *influxdb2Write.Point, 2000),
		api: newClient(settings),
		ctx: context.Background(),
	}
	go sender.receive()

	return sender, nil
}
