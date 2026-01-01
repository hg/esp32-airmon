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
	"go.uber.org/zap"
)

var log = logger.Get(logger.Mqtt)

type Settings struct {
	Broker string
	User   string
	Pass   string
}

func (se *Settings) validate() error {
	if se.Broker == "" {
		return errors.New("MQTT broker is empty")
	}
	if se.User != "" {
		log.Info("using username", zap.String("username", se.User))
	}
	if se.Pass != "" {
		log.Info("using MQTT password",
			zap.String("password", strings.Repeat("*", len(se.Pass))))
	}
	return nil
}

func newMqttClient(settings *Settings, onConn mqtt.OnConnectHandler) mqtt.Client {
	opts := mqtt.NewClientOptions()
	opts.SetResumeSubs(true)
	opts.AddBroker(settings.Broker)

	if settings.User != "" {
		opts.SetUsername(settings.User)
	}
	if settings.Pass != "" {
		opts.SetPassword(settings.Pass)
	}

	opts.SetOnConnectHandler(onConn)

	opts.SetDefaultPublishHandler(func(client mqtt.Client, msg mqtt.Message) {
		log.Info("message received", zap.String("message", string(msg.Payload())))
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
	token := client.Subscribe(t.topic, 0, func(client mqtt.Client, msg mqtt.Message) {
		raw := msg.Payload()
		if len(raw) == 0 {
			log.Error("empty message", zap.Uint16("id", msg.MessageID()))
			return
		}
		if mss, err := t.mapper(raw); err == nil {
			go sender.Enqueue(mss)
		} else {
			log.Error("could not parse data",
				zap.String("topic", t.topic),
				zap.Error(err))
		}
	})

	if token.Wait() && token.Error() != nil {
		log.Error("could not subscribe to mqtt",
			zap.String("topic", t.topic),
			zap.Error(token.Error()))
	}
}

func (h *connHandler) onConnect(client mqtt.Client) {
	for _, t := range topics {
		log.Info("subscribing to topic", zap.String("topic", t.topic))
		go subscribe(t, client, h.sender)
	}
}

func Start(settings *Settings, sender *db.Storage) error {
	if err := settings.validate(); err != nil {
		return err
	}

	handler := connHandler{sender}
	client := newMqttClient(settings, handler.onConnect)

	token := client.Connect()
	if token.Wait() && token.Error() != nil {
		return fmt.Errorf("mqtt connect failed: %w", token.Error())
	}

	return nil
}
