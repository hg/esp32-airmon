package mqtt

import (
	"errors"
	"fmt"
	"strings"

	mqtt "github.com/eclipse/paho.mqtt.golang"
	"github.com/hg/airmon/data"
	"github.com/hg/airmon/db"
	"github.com/hg/airmon/logger"
	"github.com/hg/airmon/mon"
)

var log = logger.Get(logger.Mqtt)

type Settings struct {
	Broker string `json:"broker"`
	User   string `json:"user"`
	Pass   string `json:"pass"`
}

func newMqttClient(set Settings, onConn mqtt.OnConnectHandler) mqtt.Client {
	opts := mqtt.NewClientOptions()
	opts.SetResumeSubs(true)
	opts.AddBroker(set.Broker)

	if set.User != "" {
		log.Info("using username", "user", set.User)
		opts.SetUsername(set.User)
	}
	if set.Pass != "" {
		mask := strings.Repeat("*", len(set.Pass))
		log.Info("using password", "pass", mask)
		opts.SetPassword(set.Pass)
	}

	opts.SetOnConnectHandler(onConn)

	opts.SetDefaultPublishHandler(func(_ mqtt.Client, msg mqtt.Message) {
		log.Info("message received", "message", string(msg.Payload()))
	})

	mqttClient := mqtt.NewClient(opts)
	return mqttClient
}

type connHandler struct {
	sender *db.Storage
}

type mapper func(data []byte) ([]data.Measure, error)

type topic struct {
	topic  string
	mapper mapper
}

var topics = []*topic{
	{"meas/part", mon.ParsePM},
	{"meas/temp", mon.ParseTemp},
	{"meas/co2", mon.ParseCO2},
}

func subscribe(t *topic, client mqtt.Client, sender *db.Storage) {
	token := client.Subscribe(t.topic, 0, func(_ mqtt.Client, msg mqtt.Message) {
		raw := msg.Payload()
		if len(raw) == 0 {
			log.Error("empty message", "id", msg.MessageID())
			return
		}
		if mss, err := t.mapper(raw); err == nil {
			go sender.Enqueue(mss)
		} else {
			log.Error("could not parse data",
				"topic", t.topic,
				"error", err)
		}
	})

	if token.Wait() && token.Error() != nil {
		log.Error("could not subscribe to mqtt",
			"topic", t.topic,
			"error", token.Error())
	}
}

func (h *connHandler) onConnect(client mqtt.Client) {
	for _, t := range topics {
		log.Info("subscribing to topic", "topic", t.topic)
		go subscribe(t, client, h.sender)
	}
}

func Start(set Settings, save *db.Storage) error {
	if set.Broker == "" {
		return errors.New("MQTT broker is empty")
	}

	handler := connHandler{save}
	client := newMqttClient(set, handler.onConnect)

	token := client.Connect()
	if token.Wait() && token.Error() != nil {
		return fmt.Errorf("mqtt connect failed: %w", token.Error())
	}

	return nil
}
