package main

import (
	"bufio"
	"context"
	"encoding/json"
	"flag"
	"io/ioutil"
	"log"
	"net"
	"net/http"
	"os"
	"regexp"
	"strings"
	"time"

	mqtt "github.com/eclipse/paho.mqtt.golang"
	influxdb2 "github.com/influxdata/influxdb-client-go/v2"
	influxdb2Api "github.com/influxdata/influxdb-client-go/v2/api"
	influxdb2Write "github.com/influxdata/influxdb-client-go/v2/api/write"
	"golang.org/x/net/proxy"
)

var reAirkaz = regexp.MustCompile(`<script>.*sensors_data\s*=\s*(\[.+\])</script>`)

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

const airkazTimeFormat = "2006-01-02 15:04:05"

type AirkazTime struct {
	time.Time
}

var airkazLocation *time.Location

func (t *AirkazTime) UnmarshalJSON(b []byte) (err error) {
	s := strings.Trim(string(b), `"`)
	if s == "null" {
		t.Time = time.Time{}
		return nil
	}
	t.Time, err = time.ParseInLocation(airkazTimeFormat, s, airkazLocation)
	return err
}

type AirkazInfo struct {
	Id       int64      `json:"id,string"`
	City     string     `json:"city"`
	Name     string     `json:"name"`
	Lat      float64    `json:"lat,string"`
	Lng      float64    `json:"lng,string"`
	Pm10Curr *float64   `json:"pm10,string"`
	Pm10Day  *float64   `json:"pm10ss,string"`
	Pm25Curr *float64   `json:"pm25,string"`
	Pm25Day  *float64   `json:"pm25ss,string"`
	TempCurr *float64   `json:"temp,string"`
	TempDay  *float64   `json:"tempss,string"`
	Humid    *float64   `json:"humid,string"`
	Press    *float64   `json:"press,string"`
	Error    int64      `json:"error,string"`
	Status   string     `json:"status"`
	Date     AirkazTime `json:"date"`
	Hour     string     `json:"hour"`
}

func newProxiedClient() *http.Client {
	proxyDialer := proxy.FromEnvironmentUsing(&net.Dialer{
		Timeout:   30 * time.Second,
		KeepAlive: 30 * time.Second,
	})
	return &http.Client{
		Transport: &http.Transport{
			DialContext: func(ctx context.Context, network, addr string) (net.Conn, error) {
				return proxyDialer.Dial(network, addr)
			},
			MaxIdleConns:        1,
			IdleConnTimeout:     1 * time.Minute,
			TLSHandshakeTimeout: 10 * time.Second,
		},
	}
}

func saveAirkazPoints(meas AirkazInfo, write influxdb2Api.WriteAPIBlocking) {
	tags := map[string]string{
		"city":    meas.City,
		"station": meas.Name,
	}

	save := func(kind string, fields map[string]interface{}) {
		point := influxdb2.NewPoint(kind, tags, fields, meas.Date.Time)
		err := write.WritePoint(context.Background(), point)

		if err == nil {
			log.Print("airkaz ", kind, " point written: ", meas.City, " -> ", meas.Name)
		} else {
			log.Print("could not save airkaz ", kind, " point: ", err)
		}
	}

	if meas.Pm25Curr != nil && meas.Pm10Curr != nil {
		save("airkaz:particulates", map[string]interface{}{
			"pm25": uint16(*meas.Pm25Curr),
			"pm10": uint16(*meas.Pm10Curr),
		})
	}

	tempData := map[string]interface{}{}

	if meas.TempCurr != nil {
		tempData["temperature"] = *meas.TempCurr
	}
	if meas.Humid != nil {
		tempData["humidity"] = *meas.Humid
	}
	if meas.Press != nil {
		tempData["pressure"] = *meas.Press
	}

	if len(tempData) > 0 {
		save("airkaz:temperature", tempData)
	}
}

func collectAirkaz(write influxdb2Api.WriteAPIBlocking) {
	client := newProxiedClient()

	lastUpdates := map[int64]AirkazTime{}

	for {
		time.Sleep(5 * time.Second)

		resp, err := client.Get("https://airkaz.org/")
		if err != nil {
			log.Print("airkaz get failed: ", err)
			continue
		}
		defer resp.Body.Close()

		body, err := ioutil.ReadAll(resp.Body)
		if err != nil {
			log.Print("could not read response: ", err)
			continue
		}

		matches := reAirkaz.FindSubmatch(body)
		if matches == nil {
			log.Print("measurement json not found in response")
			continue
		}

		var measurements []AirkazInfo
		if err = json.Unmarshal(matches[1], &measurements); err != nil {
			log.Print("could not parse response: ", err)
			continue
		}

		log.Print("found ", len(measurements), " airkaz measurements")

		for _, meas := range measurements {
			if meas.Error != 0 || meas.Status != "active" || meas.Hour != "now" {
				continue
			}
			if last, ok := lastUpdates[meas.Id]; ok && last == meas.Date {
				continue
			}
			lastUpdates[meas.Id] = meas.Date

			go saveAirkazPoints(meas, write)
		}

		time.Sleep(5 * time.Minute)
	}
}

func main() {
	var err error
	airkazLocation, err = time.LoadLocation("Asia/Almaty")
	if err != nil {
		log.Fatal("could not find airkaz timezone")
	}

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

	go collectAirkaz(write)

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
