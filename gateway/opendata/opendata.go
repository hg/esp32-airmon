package opendata

import (
	"fmt"
	"github.com/hg/airmon/influx"
	"github.com/hg/airmon/logger"
	"github.com/hg/airmon/net"
	"github.com/hg/airmon/tm"
	influxdb2 "github.com/influxdata/influxdb-client-go/v2"
	"github.com/pkg/errors"
	"go.uber.org/zap"
	"strconv"
	"time"
)

var log = logger.Get(logger.OpenData)

type worker struct {
	sender *influx.MeasurementSender
	client *net.Client
}

type city struct {
	ID    int    `json:"id"`
	Title string `json:"title"`
}

type cities struct {
	List []city `json:"list"`
}

func (w *worker) loadCities() []city {
	var res cities
	err := w.client.GetJSON("http://opendata.kz/api/city/list", &res)
	if err != nil {
		log.Error("error loading city list", zap.Error(err))
		return nil
	}
	log.Info("loaded cities", zap.Int("count", len(res.List)))
	return res.List
}

type sensor struct {
	SensorId  string   `json:"sensor_id"`
	Name      string   `json:"name"`
	Latitude  *float64 `json:"latitude,string"`
	Longitude *float64 `json:"longitude,string"`
	Height    *float64 `json:"sensor_height,string"`
	History   []struct {
		Data struct {
			CO2             float64 `json:"field1"`
			PM25            float64 `json:"field2"`
			TempC           float64 `json:"field3"`
			Humidity        float64 `json:"field5"`
			CO2Updated      tm.Time `json:"field1_created_at"`
			PM25Updated     tm.Time `json:"field2_created_at"`
			TempCUpdated    tm.Time `json:"field3_created_at"`
			HumidityUpdated tm.Time `json:"field5_created_at"`
		} `json:"data"`
	} `json:"history"`
}

type measurements struct {
	Sensors []sensor `json:"sensors"`
}

func (w *worker) loadMeasurements(cityID int) ([]sensor, error) {
	url := fmt.Sprintf("http://opendata.kz/api/sensor/getListWithLastHistory?cityId=%d", cityID)
	log.Debug("using url", zap.String("url", url))

	var res measurements

	if err := w.client.GetJSON(url, &res); err != nil {
		return nil, errors.WithMessage(err, "request failed")
	}

	return res.Sensors, nil
}

func (w *worker) saveMeasurements(sensors []sensor, city city) {
	for _, sens := range sensors {
		if len(sens.History) < 0 {
			log.Error("empty history", zap.String("sensor", sens.Name))
			continue
		}

		for _, hist := range sens.History {
			log.Info("saving measurement",
				zap.String("city", city.Title),
				zap.String("sensor", sens.Name))

			tags := map[string]string{
				"cityId":    strconv.Itoa(city.ID),
				"city":      city.Title,
				"stationId": sens.SensorId,
				"station":   sens.Name,
			}

			fields := map[string]interface{}{
				"co2":         hist.Data.CO2,
				"pm25":        hist.Data.PM25,
				"temperature": hist.Data.TempC,
				"humidity":    hist.Data.Humidity,
				"latitude":    sens.Latitude,
				"longitude":   sens.Longitude,
				"height":      sens.Height,
			}

			w.sender.Send(influxdb2.NewPoint("opendata", tags, fields, hist.Data.CO2Updated.Time))
		}
	}
}

func (w *worker) fetchAndSave() {
	for _, c := range w.loadCities() {
		sensors, err := w.loadMeasurements(c.ID)
		if err != nil {
			log.Error("error loading measurements", zap.Error(err),
				zap.Int("cityID", c.ID),
				zap.String("city", c.Title))
		} else {
			w.saveMeasurements(sensors, c)
		}
	}
}

func Collect(sender *influx.MeasurementSender) {
	wrk := &worker{
		sender: sender,
		client: net.NewProxiedClient(),
	}

	for {
		wrk.fetchAndSave()
		time.Sleep(5 * time.Minute)
	}
}
