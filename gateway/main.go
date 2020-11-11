package main

import (
	"bufio"
	"context"
	"encoding/json"
	"flag"
	"log"
	"os"
	"time"

	mqtt "github.com/eclipse/paho.mqtt.golang"
	influxdb2 "github.com/influxdata/influxdb-client-go/v2"
	influxdb2Api "github.com/influxdata/influxdb-client-go/v2/api"
	influxdb2Write "github.com/influxdata/influxdb-client-go/v2/api/write"
)

type temperature struct {
	Device      string  `json:"dev"`
	Time        int64   `json:"time"`
	Sensor      string  `json:"sens"`
	Temperature float64 `json:"temp"`
}

func (t *temperature) toPoint() *influxdb2Write.Point {
	return influxdb2.NewPoint("temperature",
		map[string]string{
			"device": t.Device,
			"sensor": t.Sensor,
		},
		map[string]interface{}{
			"temperature": t.Temperature,
		},
		time.Unix(t.Time, 0))
}

func isStdinPiped() bool {
	stat, err := os.Stdin.Stat()
	return err == nil && (stat.Mode()&os.ModeCharDevice) == 0
}

func saveTemperature(write influxdb2Api.WriteAPIBlocking, data []byte) error {
	var temp temperature
	if err := json.Unmarshal(data, &temp); err != nil {
		log.Print("could not parse temp json: ", err)
		return err
	}
	point := temp.toPoint()
	if err := write.WritePoint(context.Background(), point); err != nil {
		log.Print("could not write point: ", err)
		return err
	}
	log.Print("temp point written: ", temp)
	return nil
}

func main() {
	mqttBroker := flag.String("mqtt.broker", "tcp://localhost:1883", "the broker URI")
	mqttUser := flag.String("mqtt.user", "", "MQTT username")
	mqttPassword := flag.String("mqtt.pass", "", "MQTT password")

	influxUri := flag.String("influx.uri", "http://localhost:8086", "InfluxDB server URI")
	influxOrg := flag.String("influx.org", "home", "InfluxDB organization name")
	influxBucket := flag.String("influx.bucket", "airmon", "InfluxDB server URI")
	influxToken := flag.String("influx.token", "", "InfluxDB access token")

	flag.Parse()

	if *mqttBroker == "" {
		log.Fatal("broker URI not set")
	}
	if *influxUri == "" {
		log.Fatal("InfluxDB URI not set")
	}

	if *influxToken == "" {
		log.Print("InfluxDB token is empty, set it if you see authentication errors")
	}

	influx := influxdb2.NewClient(*influxUri, *influxToken)
	write := influx.WriteAPIBlocking(*influxOrg, *influxBucket)

	if isStdinPiped() {
		failed := false
		for scanner := bufio.NewScanner(os.Stdin); scanner.Scan(); {
			line := scanner.Text()
			log.Print("parsing", line)
			if err := saveTemperature(write, []byte(line)); err != nil {
				failed = true
			}
		}
		if failed {
			os.Exit(1)
		} else {
			os.Exit(0)
		}
	}

	if *mqttBroker == "" {
		log.Fatal("empty broker URI")
	}

	opts := mqtt.NewClientOptions()
	opts.SetResumeSubs(true)
	opts.AddBroker(*mqttBroker)

	if *mqttUser != "" {
		opts.SetUsername(*mqttUser)
		log.Print("using MQTT username ", *mqttUser)
	}
	if *mqttPassword != "" {
		opts.SetPassword(*mqttPassword)
		log.Print("using MQTT password")
	}

	opts.SetDefaultPublishHandler(func(client mqtt.Client, msg mqtt.Message) {
		log.Print("mqtt msg received:", string(msg.Payload()))
	})

	mqttClient := mqtt.NewClient(opts)
	mqttToken := mqttClient.Connect()

	if mqttToken.Wait() && mqttToken.Error() != nil {
		log.Fatal("mqtt connect failed: ", mqttToken.Error())
	}

	mqttClient.Subscribe("meas/temp", 0, func(client mqtt.Client, msg mqtt.Message) {
		go saveTemperature(write, msg.Payload())
	})
	if mqttToken.Wait() && mqttToken.Error() != nil {
		log.Fatal("could not subscribe to mqtt topic: ", mqttToken.Error())
	}

	log.Print("started, press Ctrl+C to terminate")

	select {}
}
