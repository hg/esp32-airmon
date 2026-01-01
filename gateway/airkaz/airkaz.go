package airkaz

import (
	"encoding/json"
	"fmt"
	"log/slog"
	"regexp"
	"time"

	"github.com/hg/airmon/data"
	"github.com/hg/airmon/db"
	"github.com/hg/airmon/logger"
	"github.com/hg/airmon/net"
	"github.com/hg/airmon/tm"
)

var log = logger.Get(logger.Airkaz)

var dataRe = regexp.MustCompile(`(?si)<script.*>.*sensors_data\s*=\s*(\[.+])</script`)

type measurement struct {
	City     string   `json:"city"`
	Name     string   `json:"name"`
	Lat      float32  `json:"lat,string"`
	Lng      float32  `json:"lng,string"`
	Pm10Curr *float32 `json:"pm10,string"`
	Pm25Curr *float32 `json:"pm25,string"`
	TempCurr *float32 `json:"temp,string"`
	Humidity *float32 `json:"humid,string"`
	Pressure *float32 `json:"press,string"`
	Error    int64    `json:"error,string"`
	Status   string   `json:"status"`
	Date     tm.Time  `json:"date"`
	Hour     string   `json:"hour"`
}

type collector struct {
	sender *db.Storage
	client *net.Client
}

func (co *collector) load() ([]measurement, error) {
	page, err := co.client.Get("https://airkaz.org/")
	if err != nil {
		return nil, err
	}

	matches := dataRe.FindSubmatch(page)
	if matches == nil {
		return nil, fmt.Errorf("json not found in response: %w", err)
	}

	var mss []measurement
	err = json.Unmarshal(matches[1], &mss)
	return mss, err
}

func (ms *measurement) convert() data.Measure {
	var level []data.Level

	add := func(sub, unit string, val *float32) {
		if val != nil {
			level = append(level, data.Level{
				Substance: sub,
				Unit:      unit,
				Value:     *val,
			})
		}
	}

	add("PM2.5", "µg/m³", ms.Pm25Curr)
	add("PM10", "µg/m³", ms.Pm10Curr)
	add("TEMP", "°C", ms.TempCurr)
	add("RH", "%", ms.Humidity)
	add("BP", "mmHg", ms.Pressure)

	if len(level) == 0 {
		return data.Measure{}
	}

	return data.Measure{
		Post: data.Post{
			Source: data.Airkaz,
			Name:   ms.City + ":" + ms.Name, // TODO: legacy data
			City:   ms.City,
			Lon:    ms.Lng,
			Lat:    ms.Lat,
		},
		Rows: []data.Observation{
			{
				Date:  ms.Date.Time,
				Level: level,
			},
		},
	}
}

func (co *collector) update() error {
	mss, err := co.load()
	if err != nil {
		return err
	}

	log.Info("loaded measurements", "count", len(mss))

	var rows []data.Measure
	now := time.Now()

	for _, ms := range mss {
		if ms.Error != 0 || ms.Status != "active" || ms.Hour != "now" {
			continue
		}
		// adjust incorrect timezone
		if ms.Date.Time.After(now) {
			ms.Date.Time = ms.Date.Add(-time.Hour)
		}
		if row := ms.convert(); len(row.Rows) > 0 {
			rows = append(rows, row)
		}
	}

	go co.sender.Enqueue(rows)
	return nil
}

func (co *collector) collect() {
	for {
		if err := co.update(); err != nil {
			log.Error("unable to load airkaz", "error", err)
		} else {
			log.Info("airkaz updated")
		}
		time.Sleep(10 * time.Minute)
	}
}

func Start(sender *db.Storage) {
	co := collector{
		sender: sender,
		client: net.NewProxiedClient(),
	}
	go co.collect()
}
