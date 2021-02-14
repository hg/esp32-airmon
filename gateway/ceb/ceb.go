package ceb

import (
	"encoding/json"
	"github.com/hg/airmon/influx"
	"github.com/hg/airmon/net"
	"github.com/hg/airmon/tm"
	influxdb2 "github.com/influxdata/influxdb-client-go/v2"
	"github.com/pkg/errors"
	"io/ioutil"
	"log"
	"net/http"
	"strings"
	"time"
)

type measurement struct {
	Id            string  `json:"id"`
	Title         string  `json:"post_title"`
	Address       string  `json:"address"`
	Lat           float64 `json:"lat,string"`
	Lon           float64 `json:"lon,string"`
	Pollutant     string  `json:"desc"`
	PollutantFull string  `json:"substance"`
	ValueMg       float64 `json:"value"`
	Date          tm.Time `json:"cdate"`
}

func Collect(sender *influx.MeasurementSender) {
	client := net.NewProxiedClient()
	lastAt := time.Time{}

	for {
		if measurements, err := getResponse(client); err == nil {
			latest := time.Time{}
			toSave := make([]measurement, len(measurements))

			log.Print("found ", len(measurements), " ceb measurements")

			for _, meas := range measurements {
				if meas.Date.After(lastAt) {
					latest = meas.Date.Time
					toSave = append(toSave, meas)
				}
			}

			if !latest.IsZero() {
				lastAt = latest
			}

			go func() {
				for _, ms := range toSave {
					saveMeasurement(ms, sender)
				}
			}()
		} else {
			log.Print("could not load data: ", err)
		}

		time.Sleep(5 * time.Minute)
	}
}

func saveMeasurement(ms measurement, sender *influx.MeasurementSender) {
	endOfFormula := strings.Index(ms.PollutantFull, "-")
	if endOfFormula <= 0 {
		return
	}

	tags := map[string]string{
		"station":   ms.Title,
		"address":   ms.Address,
		"formula":   ms.PollutantFull[:endOfFormula],
		"pollutant": ms.Pollutant,
	}

	fields := map[string]interface{}{
		"level_ug": ms.ValueMg * 1000,
		"lat":      ms.Lat,
		"lon":      ms.Lon,
	}

	sender.Send(influxdb2.NewPoint("ceb", tags, fields, ms.Date.Time))
}

func getResponse(client *http.Client) ([]measurement, error) {
	resp, err := client.Get("https://ceb-uk.kz/map/ajax.php?markers")
	if err != nil {
		return nil, errors.Wrap(err, "data fetch failed")
	}
	defer resp.Body.Close()

	body, err := ioutil.ReadAll(resp.Body)
	if err != nil {
		return nil, errors.Wrap(err, "could not read response")
	}

	var measurements []measurement
	err = json.Unmarshal(body, &measurements)
	return measurements, err
}
