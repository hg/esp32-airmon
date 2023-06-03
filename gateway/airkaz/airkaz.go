package airkaz

import (
	"encoding/json"
	"regexp"
	"time"

	"github.com/hg/airmon/influx"
	"github.com/hg/airmon/logger"
	"github.com/hg/airmon/net"
	"github.com/hg/airmon/storage"
	"github.com/hg/airmon/tm"
	influxdb2 "github.com/influxdata/influxdb-client-go/v2"
	"github.com/pkg/errors"
	"go.uber.org/zap"
)

var dataRe = regexp.MustCompile(`(?si)<script.*>.*sensors_data\s*=\s*(\[.+])</script`)

type measurement struct {
	Id       int64    `json:"id,string"`
	City     string   `json:"city"`
	Name     string   `json:"name"`
	Lat      float64  `json:"lat,string"`
	Lng      float64  `json:"lng,string"`
	Pm10Curr *float64 `json:"pm10,string"`
	Pm10Day  *float64 `json:"pm10ss,string"`
	Pm25Curr *float64 `json:"pm25,string"`
	Pm25Day  *float64 `json:"pm25ss,string"`
	TempCurr *float64 `json:"temp,string"`
	TempDay  *float64 `json:"tempss,string"`
	Humid    *float64 `json:"humid,string"`
	Press    *float64 `json:"press,string"`
	Error    int64    `json:"error,string"`
	Status   string   `json:"status"`
	Date     tm.Time  `json:"date"`
	Hour     string   `json:"hour"`
}

type lastUpdates map[int64]tm.Time

type collector struct {
	sender      *influx.MeasurementSender
	client      *net.Client
	lastUpdates lastUpdates
}

const lastUpdatesFilename = "airkaz-times.json"

var log = logger.Get(logger.Airkaz)

func Collect(sender *influx.MeasurementSender) {
	col := collector{
		sender:      sender,
		client:      net.NewProxiedClient(),
		lastUpdates: lastUpdates{},
	}

	if last := loadLastUpdates(); last != nil {
		col.lastUpdates = *last
	}

	for {
		if err := col.run(); err != nil {
			log.Error("could not save airkaz data", zap.Error(err))
		}
		time.Sleep(5 * time.Minute)
	}
}

func loadLastUpdates() *lastUpdates {
	var lu *lastUpdates

	err := storage.Load(lastUpdatesFilename, &lu)
	if err != nil {
		lu = nil
		log.Error("could not load airkaz update times", zap.Error(err))
	}

	return lu
}

func (c *collector) saveLastUpdates() {
	if err := storage.Save(lastUpdatesFilename, c.lastUpdates); err != nil {
		log.Error("could not save airkaz update times", zap.Error(err))
	}
}

func (c *collector) run() error {
	measurements, err := c.getResponse()
	if err != nil {
		return err
	}

	log.Info("measurements loaded", zap.Int("count", len(measurements)))

	toSave := make([]measurement, len(measurements))

	for _, meas := range measurements {
		if meas.Error != 0 || meas.Status != "active" || meas.Hour != "now" {
			continue
		}
		if last, ok := c.lastUpdates[meas.Id]; ok && !meas.Date.After(last.Time) {
			continue
		}
		c.lastUpdates[meas.Id] = meas.Date
		toSave = append(toSave, meas)
	}

	go c.saveLastUpdates()

	go func() {
		for _, ms := range toSave {
			c.saveMeasurement(&ms)
		}
	}()

	return nil
}

func (c *collector) getResponse() ([]measurement, error) {
	body, err := c.client.Get("https://airkaz.org/")
	if err != nil {
		return nil, err
	}

	matches := dataRe.FindSubmatch(body)
	if matches == nil {
		return nil, errors.Wrap(err, "measurement json not found in response")
	}

	var measurements []measurement
	if err = json.Unmarshal(matches[1], &measurements); err != nil {
		return nil, errors.Wrap(err, "could not parse response: ")
	}

	return measurements, nil
}

func (c *collector) saveMeasurement(meas *measurement) {
	tags := map[string]string{
		"city":    meas.City,
		"station": meas.Name,
	}

	save := func(kind string, fields map[string]interface{}) {
		c.sender.Send(influxdb2.NewPoint(kind, tags, fields, meas.Date.Time))
	}

	pmData := map[string]interface{}{
		"lat": meas.Lat,
		"lon": meas.Lng,
	}
	if meas.Pm25Curr != nil {
		pmData["pm25"] = uint16(*meas.Pm25Curr)
	}
	if meas.Pm10Curr != nil {
		pmData["pm10"] = uint16(*meas.Pm10Curr)
	}
	if len(pmData) > 0 {
		save("airkaz:particulates", pmData)
	}

	tempData := make(map[string]interface{})
	if meas.TempCurr != nil {
		tempData["temperature"] = *meas.TempCurr
	}
	if meas.Humid != nil {
		tempData["humidity"] = *meas.Humid
	}
	if meas.Press != nil {
		tempData["pressure"] = *meas.Press
	}
	if len(tempData) > 0 {
		save("airkaz:temperature", tempData)
	}
}
