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

type pmConcentration struct {
	Pm1   uint16 `json:"pm1"`
	Pm2_5 uint16 `json:"pm2.5"`
	Pm10  uint16 `json:"pm10"`
}

type particulates struct {
	Device      string          `json:"dev"`
	Time        int64           `json:"time"`
	Sensor      string          `json:"sens"`
	Standard    pmConcentration `json:"std"`
	Atmospheric pmConcentration `json:"atm"`
	Count       struct {
		Pm03  uint16 `json:"pm0.3"`
		Pm05  uint16 `json:"pm0.5"`
		Pm1   uint16 `json:"pm1"`
		Pm2_5 uint16 `json:"pm2.5"`
		Pm5   uint16 `json:"pm5"`
		Pm10  uint16 `json:"pm10"`
	} `json:"cnt"`
}

func (t *particulates) toPoint() *influxdb2Write.Point {
	return influxdb2.NewPoint("particulates",
		map[string]string{
			"device": t.Device,
			"sensor": t.Sensor,
		},
		map[string]interface{}{
			"ug_std_1":   t.Standard.Pm1,
			"ug_std_2_5": t.Standard.Pm2_5,
			"ug_std_10":  t.Standard.Pm10,
			"ug_atm_1":   t.Atmospheric.Pm1,
			"ug_atm_2_5": t.Atmospheric.Pm2_5,
			"ug_atm_10":  t.Atmospheric.Pm10,
			"cnt_0_3":    t.Count.Pm03,
			"cnt_0_5":    t.Count.Pm05,
			"cnt_1":      t.Count.Pm1,
			"cnt_2_5":    t.Count.Pm2_5,
			"cnt_5":      t.Count.Pm5,
			"cnt_10":     t.Count.Pm10,
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
		log.Print("could not temp write point: ", err)
		return err
	}
	log.Print("temp point written: ", temp)
	return nil
}

func saveParticulates(write influxdb2Api.WriteAPIBlocking, data []byte) error {
	var part particulates
	if err := json.Unmarshal(data, &part); err != nil {
		log.Print("could not parse particulates json: ", err)
		return err
	}
	point := part.toPoint()
	if err := write.WritePoint(context.Background(), point); err != nil {
		log.Print("could not write part point: ", err)
		return err
	}
	log.Print("part point written: ", part)
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

	mqttClient.Subscribe("meas/part", 0, func(client mqtt.Client, msg mqtt.Message) {
		go saveParticulates(write, msg.Payload())
	})
	if mqttToken.Wait() && mqttToken.Error() != nil {
		log.Fatal("could not subscribe to mqtt topic: ", mqttToken.Error())
	}

	log.Print("started, press Ctrl+C to terminate")

	select {}
}
