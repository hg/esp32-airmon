package main

import (
	"bytes"
	"encoding/json"
	"flag"
	"os"

	"github.com/hg/airmon/airkaz"
	"github.com/hg/airmon/cityair"
	"github.com/hg/airmon/db"
	"github.com/hg/airmon/kazhmt"
	"github.com/hg/airmon/logger"
	"github.com/hg/airmon/mqtt"
)

var log = logger.Get(logger.Main)

type sources struct {
	CityAir cityair.Settings `json:"cityair"`
	Kazhmt  kazhmt.Settings  `json:"kazhmt"`
}

type settings struct {
	MQTT    mqtt.Settings `json:"mqtt"`
	DB      db.Settings   `json:"db"`
	Sources sources       `json:"sources"`
}

func loadSettings(path string) (settings, error) {
	var set settings
	b, err := os.ReadFile(path)
	if err != nil {
		return set, err
	}
	dec := json.NewDecoder(bytes.NewReader(b))
	dec.DisallowUnknownFields()
	err = dec.Decode(&set)
	return set, err
}

func main() {
	var conf string

	flag.StringVar(&conf, "config", "/etc/airmon.json", "path to config file")
	flag.Parse()

	set, err := loadSettings(conf)
	if err != nil {
		log.Error("unable to load config", "error", err)
		os.Exit(1)
	}

	save, err := db.NewStorage(set.DB)
	if err != nil {
		log.Error("unable to setup db", "error", err)
		os.Exit(1)
	}

	if set.MQTT.Broker != "" {
		if err = mqtt.Start(set.MQTT, save); err != nil {
			log.Error("unable to setup MQTT client", "error", err)
			os.Exit(1)
		}
	}

	if set.Sources.CityAir.Token != "" {
		cityair.Start(save, set.Sources.CityAir)
	}
	if set.Sources.Kazhmt.Token != "" {
		kazhmt.Start(save, set.Sources.Kazhmt)
	}
	airkaz.Start(save)

	log.Info("application started")

	select {}
}
