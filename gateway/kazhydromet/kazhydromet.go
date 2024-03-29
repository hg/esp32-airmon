package kazhydromet

import (
	"os"
	"strconv"
	"time"

	"github.com/hg/airmon/influx"
	"github.com/hg/airmon/logger"
	"github.com/hg/airmon/net"
	"github.com/hg/airmon/storage"
	influxdb2 "github.com/influxdata/influxdb-client-go/v2"
	"github.com/pkg/errors"
	"go.uber.org/zap"
)

var log = logger.Get(logger.Kazhydromet)

const (
	timeFilename = "kazhydromet-times.json"
)

type measurement struct {
	Id        string    `json:"id"`
	Value     float64   `json:"value"`
	Date      time.Time `json:"date"`
	StationId int64     `json:"stationId"`
	Code      string    `json:"code"`
	Unit      string    `json:"unit"`
}

type station struct {
	Id             int64    `json:"id"`
	IsDeleted      bool     `json:"isDeleted"`
	NameRu         string   `json:"nameRu"`
	NameKk         string   `json:"nameKk"`
	NameEn         string   `json:"nameEn"`
	Number         int64    `json:"number"`
	Longitude      float64  `json:"longitude"`
	Latitude       float64  `json:"latitude"`
	AddressRu      string   `json:"addressRu"`
	AddressKk      string   `json:"addressKk"`
	AddressEn      string   `json:"addressEn"`
	CityRu         string   `json:"cityRu"`
	CityKk         string   `json:"cityKk"`
	CityEn         string   `json:"cityEn"`
	Limit          float64  `json:"pdk"`
	HideComponents []string `json:"hideComponents"`
	CommentRu      string   `json:"commentRu"`
	CommentKk      string   `json:"commentKk"`
	CommentEn      string   `json:"commentEn"`
}

type entry struct {
	ms *measurement
	st *station
}

type measurementTimes map[string]time.Time

type collector struct {
	client   *net.Client
	sender   *influx.MeasurementSender
	stations map[int64]*station
	times    measurementTimes
}

func lastMapKey(ms *measurement) string {
	return strconv.FormatInt(ms.StationId, 10) + "/" + ms.Code
}

func Collect(sender *influx.MeasurementSender) {
	c := collector{
		client:   net.NewProxiedClient(),
		sender:   sender,
		stations: make(map[int64]*station),
		times:    make(measurementTimes),
	}

	if savedTime := loadSavedTime(); savedTime != nil {
		c.times = savedTime
	}

	for {
		entries, err := c.loadData()
		if err == nil {
			go saveTime(c.times)
			err = c.saveData(entries)
		}
		if err != nil {
			log.Error("could not save kazhydromet data", zap.Error(err))
		}
		time.Sleep(20 * time.Minute)
	}
}

func saveTime(ms measurementTimes) {
	err := storage.Save(timeFilename, ms)
	if err != nil {
		log.Error("could not save kazhydromet measurement times", zap.Error(err))
	}
}

func loadSavedTime() measurementTimes {
	var mt measurementTimes

	err := storage.Load(timeFilename, &mt)
	if err == nil {
		return mt
	}

	log.Error("could not parse kazhydromet measurement times", zap.Error(err))
	return nil
}

func (c *collector) saveData(entries []entry) error {
	failed := 0

	for _, ent := range entries {
		tags := map[string]string{
			"city":    ent.st.CityRu,
			"station": ent.st.NameRu,
			"address": ent.st.AddressRu,
			"code":    ent.ms.Code,
			"unit":    ent.ms.Unit,
		}

		fields := map[string]interface{}{
			"lat":   ent.st.Latitude,
			"lon":   ent.st.Longitude,
			"value": ent.ms.Value,
		}

		point := influxdb2.NewPoint("kazhydromet", tags, fields, ent.ms.Date)

		if !c.sender.Send(point) {
			failed++
		}
	}

	return errors.New("could not save " + strconv.Itoa(failed) + " points to db")
}

func (c *collector) loadData() ([]entry, error) {
	refreshedStations := false

	measurements, err := c.loadMeasurements()
	if err != nil {
		return nil, err
	}

	log.Info("loaded measurements", zap.Int("count", len(measurements)))

	var entries []entry
	for _, meas := range measurements {
		stat := c.stations[meas.StationId]

		if stat == nil {
			if refreshedStations {
				// We have already refreshed station data in this iteration.
				// It's an invalid station id, there's nothing we can do.
				log.Error("station not found", zap.Int64("id", meas.StationId))
				continue
			}

			refreshedStations = true

			if err = c.loadStations(); err != nil {
				log.Error("could not load station data", zap.Error(err))
				continue
			}

			stat = c.stations[meas.StationId]
			if stat == nil {
				log.Error("station not found", zap.Int64("id", meas.StationId))
				continue
			}
		}

		key := lastMapKey(meas)
		if lastAt, ok := c.times[key]; ok && !meas.Date.After(lastAt) {
			continue
		}
		c.times[key] = meas.Date

		entries = append(entries, entry{ms: meas, st: stat})
	}

	return entries, nil
}

func (c *collector) loadMeasurements() (measurements []*measurement, err error) {
	key := os.Getenv("KAZHYDROMET_KEY")
	url := "http://atmosphera.kz:4003/simple/averages/last?key=" + key
	err = c.client.GetJSON(url, &measurements)
	return
}

func (c *collector) loadStations() error {
	var stations []*station

	err := c.client.GetJSON("http://atmosphera.kz:4004/stations", &stations)
	if err != nil {
		return err
	}

	c.stations = make(map[int64]*station)

	for _, stat := range stations {
		c.stations[stat.Id] = stat
	}

	return nil
}
