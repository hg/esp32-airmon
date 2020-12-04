package main

import (
	"flag"
	"github.com/hg/airmon/airkaz"
	"github.com/hg/airmon/influx"
	"github.com/hg/airmon/mqtt"
	"log"
)

func main() {
	var mqts mqtt.Settings
	flag.StringVar(&mqts.Broker, "mqtt.broker", "tcp://localhost:1883", "the broker URI")
	flag.StringVar(&mqts.User, "mqtt.user", "", "MQTT username")
	flag.StringVar(&mqts.Pass, "mqtt.pass", "", "MQTT password")

	var infs influx.Settings
	flag.StringVar(&infs.Uri, "influx.uri", "http://localhost:8086", "InfluxDB server URI")
	flag.StringVar(&infs.Org, "influx.org", "home", "InfluxDB organization name")
	flag.StringVar(&infs.Bucket, "influx.bucket", "airmon", "InfluxDB server URI")
	flag.StringVar(&infs.Token, "influx.token", "", "InfluxDB access token")

	flag.Parse()

	writeCh, err := influx.NewWriter(infs)
	if err != nil {
		log.Fatal("could not prepare InfluxDB writer: ", err)
	}

	go airkaz.Collect(writeCh)

	if err = mqtt.StartMqtt(&mqts, writeCh); err != nil {
		log.Fatal("could not create MQTT client: ", err)
	}

	log.Print("started, press Ctrl+C to terminate")

	select {}
}
