package main

import (
	"flag"
	"os"

	"github.com/hg/airmon/airkaz"
	"github.com/hg/airmon/cityair"
	"github.com/hg/airmon/db"
	"github.com/hg/airmon/kazhmt"
	"github.com/hg/airmon/logger"
	"github.com/hg/airmon/mqtt"
	"go.uber.org/zap"
)

var log = logger.Get(logger.Main)

func env(key string, fallback string) string {
	if val := os.Getenv(key); val != "" {
		log.Info("using env var", zap.String("var", key))
		return val
	}
	return fallback
}

const (
	defaultBroker = "tcp://localhost:1883"
	defaultDB     = "postgres://air:pass@localhost:5432/air"
)

type tokens struct {
	kazhmt  string
	cityair string
}

func main() {
	var mqs mqtt.Settings
	var dbs db.Settings
	var cas cityair.Settings
	var khs kazhmt.Settings

	flag.StringVar(&mqs.Broker, "mqtt.broker", env("MQTT_BROKER", defaultBroker), "MQTT broker URI")
	flag.StringVar(&mqs.User, "mqtt.user", env("MQTT_USER", ""), "MQTT username")
	flag.StringVar(&mqs.Pass, "mqtt.pass", env("MQTT_PASS", ""), "MQTT password")
	flag.StringVar(&dbs.Uri, "db", env("POSTGRES_URI", defaultDB), "PostgreSQL connection URI")
	flag.StringVar(&khs.Token, "kazhmt.token", env("KAZHYDROMET_TOKEN", ""), "kazhydromet auth token")
	flag.StringVar(&cas.Token, "cityair.token", env("CITYAIR_TOKEN", ""), "cityair auth token")
	flag.StringVar(&cas.LatLon, "cityair.latlon", env("CITYAIR_CENTER", ""), "frequently poll stations in this location, e.g. 12.34,-34.56 ")
	flag.Parse()

	send, err := db.NewStorage(dbs)
	if err != nil {
		log.Fatal("unable to setup db", zap.Error(err))
		return
	}

	if mqs.Broker != "" {
		if err = mqtt.Start(&mqs, send); err != nil {
			log.Fatal("unable to setup MQTT client", zap.Error(err))
			return
		}
	}

	if cas.Token != "" {
		cityair.Start(send, cas)
	}
	if khs.Token != "" {
		kazhmt.Start(send, khs)
	}
	airkaz.Start(send)

	log.Info("application started")

	select {}
}
