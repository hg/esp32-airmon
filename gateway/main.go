package main

import (
	"flag"
	"github.com/hg/airmon/airkaz"
	"github.com/hg/airmon/ceb"
	"github.com/hg/airmon/influx"
	"github.com/hg/airmon/mqtt"
	"log"
	"os"
	"runtime/pprof"
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

	profile := flag.String("profile", "", "write cpu profile to this file")

	flag.Parse()
	mqts.SetFromEnvironment()
	infs.SetFromEnvironment()

	if *profile != "" {
		f, err := os.Create(*profile)
		if err != nil {
			log.Fatal("could not create cpu profile file: ", err)
		}
		defer f.Close()

		_ = pprof.StartCPUProfile(f)
		defer pprof.StopCPUProfile()
	}

	sender, err := influx.NewWriter(infs)
	if err != nil {
		log.Fatal("could not prepare InfluxDB writer: ", err)
	}

	go airkaz.Collect(sender)
	go ceb.Collect(sender)

	if err = mqtt.StartMqtt(&mqts, sender); err != nil {
		log.Fatal("could not create MQTT client: ", err)
	}

	log.Print("started, press Ctrl+C to terminate")

	select {}
}
