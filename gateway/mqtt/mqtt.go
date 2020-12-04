package mqtt

import (
	mqtt "github.com/eclipse/paho.mqtt.golang"
	"github.com/hg/airmon/mon"
	influxdb2Write "github.com/influxdata/influxdb-client-go/v2/api/write"
	"github.com/pkg/errors"
	"log"
	"strings"
)

type Settings struct {
	Broker string
	User   string
	Pass   string
}

func (s *Settings) validate() error {
	if s.Broker == "" {
		return errors.New("MQTT broker is empty")
	}
	if s.User != "" {
		log.Print("using MQTT username ", s.User)
	}
	if s.Pass != "" {
		log.Print("using MQTT password ", strings.Repeat("*", len(s.Pass)))
	}
	return nil
}

func newMqttClient(settings *Settings) mqtt.Client {
	opts := mqtt.NewClientOptions()
	opts.SetResumeSubs(true)
	opts.AddBroker(settings.Broker)

	if settings.User != "" {
		opts.SetUsername(settings.User)
	}
	if settings.Pass != "" {
		opts.SetPassword(settings.Pass)
	}

	opts.SetDefaultPublishHandler(func(client mqtt.Client, msg mqtt.Message) {
		log.Print("mqtt msg received:", string(msg.Payload()))
	})

	mqttClient := mqtt.NewClient(opts)
	return mqttClient
}

func StartMqtt(settings *Settings, write chan<- *influxdb2Write.Point) error {
	if err := settings.validate(); err != nil {
		return err
	}

	client := newMqttClient(settings)

	token := client.Connect()
	if token.Wait() && token.Error() != nil {
		return errors.Wrap(token.Error(), "mqtt connect failed")
	}

	client.Subscribe("meas/temp", 0, func(client mqtt.Client, msg mqtt.Message) {
		if point, err := mon.ParseTemperature(msg.Payload()); err == nil {
			write <- point
		} else {
			log.Print("could not parse temperature: ", err)
		}
	})
	if token.Wait() && token.Error() != nil {
		return errors.Wrap(token.Error(), "could not subscribe to mqtt topic")
	}

	client.Subscribe("meas/part", 0, func(client mqtt.Client, msg mqtt.Message) {
		if point, err := mon.ParseParticulates(msg.Payload()); err == nil {
			write <- point
		} else {
			log.Print("could not parse particulates: ", err)
		}
	})
	if token.Wait() && token.Error() != nil {
		return errors.Wrap(token.Error(), "could not subscribe to mqtt topic")
	}

	return nil
}
