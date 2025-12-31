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
	Id             int64    `json:"id"`
	IsDeleted      bool     `json:"isDeleted"`
	NameRu         string   `json:"nameRu"`
	NameKk         string   `json:"nameKk"`
	NameEn         string   `json:"nameEn"`
	Number         int64    `json:"number"`
	Longitude      float32  `json:"longitude"`
	Latitude       float32  `json:"latitude"`
	AddressRu      string   `json:"addressRu"`
	AddressKk      string   `json:"addressKk"`
	AddressEn      string   `json:"addressEn"`
	CityRu         string   `json:"cityRu"`
	CityKk         string   `json:"cityKk"`
	CityEn         string   `json:"cityEn"`
	Limit          float32  `json:"pdk"`
	HideComponents []string `json:"hideComponents"`
	CommentRu      string   `json:"commentRu"`
	CommentKk      string   `json:"commentKk"`
	CommentEn      string   `json:"commentEn"`
}

type entry struct {
	ms *average
	st *station
}

type collector struct {
	client   *net.Client
	sender   *db.Storage
	token    string
	stations map[int64]*station
}

func (co *collector) loadData() ([]data.Measure, error) {
	if err := co.loadStations(); err != nil {
		log.Error("could not load stations", zap.Error(err))
		return nil, err
	}

	avgs, err := co.loadAverages()
	if err != nil {
		return nil, err
	}

	log.Info("loaded avgs", zap.Int("count", len(avgs)))

	var meas []data.Measure

	for _, avg := range avgs {
		st := co.stations[avg.StationId]
		if st == nil {
			log.Error("station not found", zap.Int64("id", avg.StationId))
			continue
		}

		meas = append(meas, data.Measure{
			Date: avg.Date,
			Post: &data.Post{
				Source:  data.Kazhydromet,
				Name:    fmt.Sprintf("(КГМ) %s (%d)", st.NameRu, st.Id), // TODO: legacy data
				City:    st.CityRu,
				Address: st.AddressRu,
				Lon:     st.Longitude,
				Lat:     st.Latitude,
			},
			Level: []data.Level{
				{
					Substance: avg.Code,
					Unit:      avg.Unit,
					Value:     avg.Value,
				},
			},
		})
	}

	return meas, nil
}

func (co *collector) loadAverages() ([]*average, error) {
	var ms []*average
	uri := "http://atmosphera.kz:4003/simple/averages/last?key=" + co.token
	err := co.client.GetJSON(uri, &ms)
	return ms, err
}

func (co *collector) loadStations() error {
	var stations []*station

	err := co.client.GetJSON("http://atmosphera.kz:4004/stations", &stations)
	if err != nil {
		return err
	}
	log.Info("loaded stations", zap.Int("count", len(stations)))

	co.stations = make(map[int64]*station)

	for _, stat := range stations {
		co.stations[stat.Id] = stat
	}

	return nil
}

func (co *collector) collect() {
	for {
		if meas, err := co.loadData(); err == nil {
			go co.sender.Enqueue(meas)
			log.Info("kazhydromet updated")
		} else {
			log.Error("could not save kazhydromet data", zap.Error(err))
		}
		time.Sleep(15 * time.Minute)
	}
}

func Start(sender *db.Storage, token string) {
	co := collector{
		client:   net.NewProxiedClient(),
		sender:   sender,
		stations: make(map[int64]*station),
		token:    token,
	}

	go co.collect()
}
