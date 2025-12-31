package kazhmt

import (
	"fmt"
	"time"

	"github.com/hg/airmon/data"
	"github.com/hg/airmon/db"
	"github.com/hg/airmon/logger"
	"github.com/hg/airmon/net"
	"go.uber.org/zap"
)

var log = logger.Get(logger.Kazhydromet)

type average struct {
	Id        string    `json:"id"`
	Value     float32   `json:"value"`
	Date      time.Time `json:"date"`
	StationId int64     `json:"stationId"`
	Code      string    `json:"code"`
	Unit      string    `json:"unit"`
}

type station struct {
	Id        int64   `json:"id"`
	NameRu    string  `json:"nameRu"`
	Longitude float32 `json:"longitude"`
	Latitude  float32 `json:"latitude"`
	AddressRu string  `json:"addressRu"`
	CityRu    string  `json:"cityRu"`
}

type collector struct {
	client   *net.Client
	sender   *db.Storage
	token    string
	stations map[int64]station
	full     time.Time
}

func (co *collector) loadAvgs() (map[int64][]average, error) {
	if err := co.loadStations(); err != nil {
		log.Error("could not load stations", zap.Error(err))
		return nil, err
	}

	avgs, err := co.loadLatest()
	if err != nil {
		return nil, err
	}

	if co.full.Before(time.Now()) {
		log.Info("starting kazhydromet history refresh")

		for _, st := range co.stations {
			if rows, err := co.loadHistory(st.Id); err == nil {
				avgs = append(avgs, rows...)
			} else {
				log.Error("unable to load history", zap.Int64("id", st.Id))
			}
		}

		co.full = time.Now().Add(6 * time.Hour)
	}

	byStation := make(map[int64][]average)

	for _, avg := range avgs {
		id := avg.StationId
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

	for statId, statAvgs := range byStation {
		stat, ok := co.stations[statId]
		if !ok {
			log.Error("station not found", zap.Int64("id", statId))
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
			Post: data.Post{
				Source:  data.Kazhydromet,
				Name:    fmt.Sprintf("(КГМ) %s (%d)", stat.NameRu, stat.Id), // TODO: legacy data
				City:    stat.CityRu,
				Address: stat.AddressRu,
				Lon:     stat.Longitude,
				Lat:     stat.Latitude,
			},
			Rows: rows,
		})
	}

	return result, nil
}

func (co *collector) loadData(uri string) ([]average, error) {
	var ms []average
	err := co.client.GetJSON(uri, &ms)
	return ms, err
}

func (co *collector) loadHistory(stationId int64) ([]average, error) {
	since := time.Now().Add(-24 * time.Hour)
	uri := fmt.Sprintf("http://atmosphera.kz:4003/simple/averages?key=%s&after=%s&stationNumber=%d",
		co.token, since.Format("2006-01-02"), stationId)
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
	log.Info("loaded stations", zap.Int("count", len(stations)))

	co.stations = make(map[int64]station)

	for _, stat := range stations {
		co.stations[stat.Id] = stat
	}

	return nil
}

func (co *collector) collect() {
	for {
		if mss, err := co.update(); err == nil {
			go co.sender.Enqueue(mss)
			log.Info("kazhydromet updated")
		} else {
			log.Error("ukazhydromet update failed", zap.Error(err))
		}
		time.Sleep(15 * time.Minute)
	}
}

func Start(sender *db.Storage, token string) {
	co := collector{
		client: net.NewProxiedClient(),
		sender: sender,
		token:  token,
	}

	go co.collect()
}
