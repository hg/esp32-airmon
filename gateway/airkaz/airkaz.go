package airkaz

import (
	"encoding/json"
	"github.com/hg/airmon/influx"
	"github.com/hg/airmon/net"
	"github.com/hg/airmon/tm"
	influxdb2 "github.com/influxdata/influxdb-client-go/v2"
	"github.com/pkg/errors"
	"log"
	"regexp"
	"time"
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

func Collect(sender *influx.MeasurementSender) {
	client := net.NewProxiedClient()
	lastUpdates := map[int64]tm.Time{}

	for {
		if measurements, err := getResponse(client); err == nil {
			log.Print("found ", len(measurements), " airkaz measurements")

			toSave := make([]*measurement, len(measurements))

			for _, meas := range measurements {
				if meas.Error != 0 || meas.Status != "active" || meas.Hour != "now" {
					continue
				}
				if last, ok := lastUpdates[meas.Id]; ok && last == meas.Date {
					continue
				}
				lastUpdates[meas.Id] = meas.Date
				toSave = append(toSave, meas)
			}

			go func() {
				for _, ms := range toSave {
					saveMeasurement(ms, sender)
				}
			}()
		} else {
			log.Print("could not get response from airkaz: ", err)
		}

		time.Sleep(5 * time.Minute)
	}
}

func getResponse(client *net.Client) ([]*measurement, error) {
	body, err := client.Get("https://airkaz.org/")
	if err != nil {
		return nil, err
	}

	matches := dataRe.FindSubmatch(body)
	if matches == nil {
		return nil, errors.Wrap(err, "measurement json not found in response")
	}

	var measurements []*measurement
	if err = json.Unmarshal(matches[1], &measurements); err != nil {
		return nil, errors.Wrap(err, "could not parse response: ")
	}

	return measurements, nil
}

func saveMeasurement(meas *measurement, sender *influx.MeasurementSender) {
	tags := map[string]string{
		"city":    meas.City,
		"station": meas.Name,
	}

	save := func(kind string, fields map[string]interface{}) {
		sender.Send(influxdb2.NewPoint(kind, tags, fields, meas.Date.Time))
	}

	if meas.Pm25Curr != nil && meas.Pm10Curr != nil {
		save("airkaz:particulates", map[string]interface{}{
			"pm25": uint16(*meas.Pm25Curr),
			"pm10": uint16(*meas.Pm10Curr),
		})
	}

	tempData := map[string]interface{}{}

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
