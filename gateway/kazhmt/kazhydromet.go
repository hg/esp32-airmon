package kazhmt

import (
	"fmt"
	"time"

	"github.com/hg/airmon/client"
	"github.com/hg/airmon/data"
	"github.com/hg/airmon/db"
	"github.com/hg/airmon/logger"
	"github.com/hg/airmon/spatial"
)

var log = logger.Get(logger.Kazhydromet)

type Settings struct {
	Token string `json:"token"`
}

type average struct {
	ID        string    `json:"id"`
	Value     float32   `json:"value"`
	Date      time.Time `json:"date"`
	StationID int64     `json:"stationId"`
	Code      string    `json:"code"`
	Unit      string    `json:"unit"`
}

type station struct {
	ID        int64   `json:"id"`
	NameRu    string  `json:"nameRu"`
	Longitude float32 `json:"longitude"`
	Latitude  float32 `json:"latitude"`
	AddressRu string  `json:"addressRu"`
	CityRu    string  `json:"cityRu"`
}

type collector struct {
	client   *client.Client
	sender   *db.Storage
	token    string
	stations map[int64]station
	nextFull time.Time
}

func (co *collector) scheduleFull() {
	co.nextFull = time.Now().Add(6 * time.Hour)
}

func (co *collector) loadAvgs() (map[int64][]average, error) {
	if err := co.loadStations(); err != nil {
		log.Error("could not load stations", "error", err)
		return nil, err
	}

	avgs, err := co.loadLatest()
	if err != nil {
		return nil, err
	}

	if co.nextFull.Before(time.Now()) {
		log.Info("starting kazhydromet history refresh")

		for _, st := range co.stations {
			if rows, err := co.loadHistory(st.ID); err == nil {
				avgs = append(avgs, rows...)
			} else {
				log.Error("unable to load history", "id", st.ID)
			}
		}

		co.scheduleFull()
	}

	byStation := make(map[int64][]average)

	for _, avg := range avgs {
		id := avg.StationID
		byStation[id] = append(byStation[id], avg)
	}

	return byStation, nil
}

func (co *collector) update() ([]data.Measure, error) {
	byStation, err := co.loadAvgs()
	if err != nil {
		return nil, err
	}

	var result []data.Measure

	for statID, statAvgs := range byStation {
		stat, ok := co.stations[statID]
		if !ok {
			log.Error("station not found", "id", statID)
			continue
		}

		postID, err := co.sender.GetPost(data.Post{
			Source: data.Kazhydromet,
			Name:   stat.NameRu,
			// source likes reusing IDs for unrelated posts
			Slug:    fmt.Sprintf("%s_%d", stat.NameRu, stat.ID),
			City:    stat.CityRu,
			Address: stat.AddressRu,
			Geo: spatial.Point{
				Lat: stat.Latitude,
				Lon: stat.Longitude,
			},
		})

		if err != nil {
			log.Error("unable to find post", "error", err)
			continue
		}

		var rows []data.Observation

		for _, avg := range statAvgs {
			rows = append(rows, data.Observation{
				Date: avg.Date,
				Level: []data.Level{
					{
						Substance: avg.Code,
						Unit:      avg.Unit,
						Value:     avg.Value,
					},
				},
			})
		}

		if len(rows) == 0 {
			continue
		}

		result = append(result, data.Measure{
			PostID: postID,
			Rows:   rows,
		})
	}

	return result, nil
}

func (co *collector) loadData(uri string) ([]average, error) {
	var ms []average
	err := co.client.GetJSON(uri, &ms)
	return ms, err
}

func (co *collector) loadHistory(stationID int64) ([]average, error) {
	since := time.Now().Add(-24 * time.Hour)
	uri := fmt.Sprintf("http://atmosphera.kz:4003/simple/averages?key=%s&after=%s&stationNumber=%d",
		co.token, since.Format("2006-01-02"), stationID)
	return co.loadData(uri)
}

func (co *collector) loadLatest() ([]average, error) {
	uri := "http://atmosphera.kz:4003/simple/averages/last?key=" + co.token
	return co.loadData(uri)
}

func (co *collector) loadStations() error {
	var stations []station

	err := co.client.GetJSON("http://atmosphera.kz:4004/stations", &stations)
	if err != nil {
		return err
	}
	log.Info("loaded stations", "count", len(stations))

	co.stations = make(map[int64]station)

	for _, stat := range stations {
		co.stations[stat.ID] = stat
	}

	return nil
}

func (co *collector) collect() {
	for {
		if mss, err := co.update(); err == nil {
			go co.sender.Enqueue(mss)
			log.Info("kazhydromet updated")
		} else {
			log.Error("ukazhydromet update failed", "error", err)
		}
		now := time.Now()
		next := now.
			Add(time.Hour).
			Truncate(time.Hour).
			Add(15 * time.Minute)
		time.Sleep(next.Sub(now))
	}
}

func Start(sender *db.Storage, set Settings) {
	co := collector{
		client: client.NewProxied(),
		sender: sender,
		token:  set.Token,
	}
	co.scheduleFull()

	go co.collect()
}
