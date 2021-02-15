package kazhydromet

import (
	"encoding/json"
	"github.com/hg/airmon/influx"
	"github.com/hg/airmon/net"
	influxdb2 "github.com/influxdata/influxdb-client-go/v2"
	"github.com/pkg/errors"
	"io/ioutil"
	"log"
	"net/http"
	"strconv"
	"time"
)

type measurement struct {
	Id        string    `json:"id"`
	Value     float64   `json:"value"`
	Date      time.Time `json:"date"`
	StationId int64     `json:"stationId"`
	Code      string    `json:"code"`
	Unit      string    `json:"unit"`
	Limit     *float64  `json:"pdk"`
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

type collector struct {
	client    *http.Client
	sender    *influx.MeasurementSender
	stations  map[int64]*station
	lastAt    map[string]time.Time
	lastRunAt time.Time
}

func lastMapKey(ms *measurement) string {
	return strconv.FormatInt(ms.StationId, 10) + "/" + ms.Code
}

func (c *collector) run() error {
	entries, err := c.loadData()
	if err != nil {
		return err
	}
	return c.saveData(entries)
}

func Collect(sender *influx.MeasurementSender) {
	c := collector{
		client:   net.NewProxiedClient(),
		sender:   sender,
		stations: make(map[int64]*station),
		lastAt:   make(map[string]time.Time),
	}

	for {
		if err := c.run(); err != nil {
			log.Println("could not save kazhydromet data: ", err)
		}
		time.Sleep(20 * time.Minute)
	}
}

func (c *collector) saveData(entries []entry) error {
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
			"limit": ent.ms.Limit,
		}

		point := influxdb2.NewPoint("kazhydromet", tags, fields, ent.ms.Date)

		if ok := c.sender.Send(point); !ok {
			return errors.New("could not save point to db")
		}
	}

	return nil
}

func (c *collector) loadData() ([]entry, error) {
	refreshedStations := false

	measurements, err := c.loadMeasurements()
	if err != nil {
		return nil, err
	}

	var entries []entry
	for _, meas := range measurements {
		stat := c.stations[meas.StationId]

		if stat == nil {
			if refreshedStations {
				// We have already refreshed station data in this iteration.
				// It's an invalid station id, there's nothing we can do.
				log.Print("station id ", meas.StationId, " not found")
				continue
			}

			refreshedStations = true

			if err = c.loadStations(); err != nil {
				log.Print("could not load station data: ", err)
				continue
			}

			stat = c.stations[meas.StationId]
			if stat == nil {
				log.Print("station id ", meas.StationId, " not found")
				continue
			}
		}

		key := lastMapKey(meas)

		if lastAt, ok := c.lastAt[key]; ok && !meas.Date.After(lastAt) {
			continue
		}
		c.lastAt[key] = meas.Date

		entries = append(entries, entry{ms: meas, st: stat})
	}

	return entries, nil
}

func (c *collector) loadMeasurements() ([]*measurement, error) {
	startedAt := time.Now()

	url := "http://atmosphera.kz:4004/averages"
	if !c.lastRunAt.IsZero() {
		url += "?after=" + c.lastRunAt.UTC().Format(time.RFC3339)
	}

	resp, err := c.client.Get(url)
	if err != nil {
		return nil, err
	}
	defer resp.Body.Close()

	body, err := ioutil.ReadAll(resp.Body)
	if err != nil {
		return nil, err
	}

	var measurements []*measurement
	if err = json.Unmarshal(body, &measurements); err == nil {
		c.lastRunAt = startedAt
	}

	return measurements, err
}

func (c *collector) loadStations() error {
	resp, err := c.client.Get("http://atmosphera.kz:4004/stations")
	if err != nil {
		return err
	}
	defer resp.Body.Close()

	body, err := ioutil.ReadAll(resp.Body)
	if err != nil {
		return err
	}

	var stations []*station
	if err = json.Unmarshal(body, &stations); err != nil {
		return err
	}

	c.stations = make(map[int64]*station)

	for _, stat := range stations {
		c.stations[stat.Id] = stat
	}

	return nil
}
