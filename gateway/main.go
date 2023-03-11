package main

import (
	"flag"
	"fmt"
	"github.com/hg/airmon/airkaz"
	"github.com/hg/airmon/ceb"
	"github.com/hg/airmon/kazhydromet"
	"github.com/hg/airmon/opendata"
	"os"
	"runtime/pprof"

	"github.com/hg/airmon/influx"
	"github.com/hg/airmon/logger"
	"github.com/hg/airmon/mqtt"
	"go.uber.org/zap"
)

func main() {
	log := logger.Get(logger.Main)

	mqts := mqtt.Settings{}
	mqts.AddFlags()

	infs := influx.Settings{}
	infs.AddFlags()

	profile := flag.String("profile", "", "write cpu profile to this file")

	flag.Parse()
	mqts.SetFromEnvironment()
	infs.SetFromEnvironment()

	if *profile != "" {
		f, err := os.Create(*profile)
		if err != nil {
			log.Fatal("could not create cpu profile file", zap.Error(err))
		}
		defer f.Close()

		_ = pprof.StartCPUProfile(f)
		defer pprof.StopCPUProfile()
	}

	sender, err := influx.NewSender(infs)
	if err != nil {
		log.Fatal("could not prepare InfluxDB writer", zap.Error(err))
	}

	go kazhydromet.Collect(sender)
	go airkaz.Collect(sender)
	go ceb.Collect(sender)
	go opendata.Collect(sender)

	if err = mqtt.StartMqtt(&mqts, sender); err != nil {
		log.Fatal("could not create MQTT client", zap.Error(err))
	}

	fmt.Fprint(os.Stderr, "started, press Ctrl+C to terminate")

	select {}
}
