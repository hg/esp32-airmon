package ceb

import (
	"strings"
	"sync"
	"time"

	"github.com/hg/airmon/influx"
	"github.com/hg/airmon/logger"
	"github.com/hg/airmon/net"
	"github.com/hg/airmon/storage"
	"github.com/hg/airmon/tm"
	influxdb2 "github.com/influxdata/influxdb-client-go/v2"
	"go.uber.org/zap"
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

const dataUrl = "https://ceb-uk.kz/map/ajax.php?markers"
const timeFilename = "ceb-times.json"
const staleMaxUpdates = 3

type repeat struct {
	level float64
	count int
}

type staleDetector struct {
	sync.Mutex
	values map[string]*repeat
}

func newDetector() *staleDetector {
	return &staleDetector{
		values: map[string]*repeat{},
	}
}

func (s *staleDetector) isStale(station string, pollutant string, level float64) bool {
	s.Lock()
	defer s.Unlock()

	key := station + ":" + pollutant

	rep, ok := s.values[key]
	if !ok {
		rep = &repeat{}
		s.values[key] = rep
	}

	if rep.level == level {
		rep.count++
		if rep.count >= staleMaxUpdates {
			return true
		}
	} else {
		rep.level = level
		rep.count = 0
	}

	return false
}

type collector struct {
	stale  *staleDetector
	sender *influx.MeasurementSender
	client *net.Client
	lastAt time.Time
}

var log = logger.Get(logger.Ceb)

func Collect(sender *influx.MeasurementSender) {
	col := collector{
		sender: sender,
		stale:  newDetector(),
		client: net.NewProxiedClient(),
		lastAt: loadLastAt(),
	}
	for {
		if err := col.run(); err != nil {
			log.Error("could not get ceb measurements", zap.Error(err))
		}
		time.Sleep(10 * time.Minute)
	}
}

func (c *collector) run() error {
	measurements, err := c.getResponse()
	if err != nil {
		return err
	}

	latest := time.Time{}
	toSave := make([]measurement, len(measurements))

	log.Info("loaded measurements", zap.Int("count", len(measurements)))

	for _, ms := range measurements {
		if ms.Date.After(c.lastAt) {
			latest = ms.Date.Time
			toSave = append(toSave, ms)
		}
	}

	if !latest.IsZero() {
		go saveLastAt(latest)
		c.lastAt = latest
	}

	go func() {
		for _, ms := range toSave {
			c.saveMeasurement(&ms)
		}
	}()

	return nil
}

func loadLastAt() time.Time {
	var t time.Time
	if err := storage.Load(timeFilename, &t); err != nil {
		log.Error("could not load last ceb time", zap.Error(err))
		t = time.Time{}
	}
	return t
}

func saveLastAt(tm time.Time) {
	if err := storage.Save(timeFilename, tm); err != nil {
		log.Error("could not save ceb last time", zap.Error(err))
	}
}

func (c *collector) saveMeasurement(ms *measurement) {
	endOfFormula := strings.Index(ms.PollutantFull, "-")
	if endOfFormula <= 0 {
		return
	}

	station := ms.Title
	pollutant := ms.Pollutant
	level := ms.ValueMg * 1000

	if c.stale.isStale(station, pollutant, level) {
		log.Error("value marked as stale and will not be saved",
			zap.String("station", ms.Title),
			zap.String("pollutant", ms.Pollutant),
			zap.Float64("level", ms.ValueMg))
		return
	}

	tags := map[string]string{
		"station":   station,
		"address":   ms.Address,
		"formula":   ms.PollutantFull[:endOfFormula],
		"pollutant": pollutant,
	}

	fields := map[string]interface{}{
		"level_ug": level,
		"lat":      ms.Lat,
		"lon":      ms.Lon,
	}

	c.sender.Send(influxdb2.NewPoint("ceb", tags, fields, ms.Date.Time))
}

func (c *collector) getResponse() (measurements []measurement, err error) {
	err = c.client.GetJSON(dataUrl, &measurements)
	return
}
