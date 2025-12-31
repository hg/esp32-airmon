package mqtt

import (
	"strings"

	mqtt "github.com/eclipse/paho.mqtt.golang"
	"github.com/hg/airmon/db"
	"github.com/hg/airmon/logger"
	"github.com/hg/airmon/mon"
	"github.com/pkg/errors"
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

type topic struct {
	topic  string
	mapper func(data []byte) (mon.DataSource, error)
}

var topics = []*topic{
	{"meas/part", mon.ParseParticulates},
	{"meas/temp", mon.ParseTemperature},
	{"meas/co2", mon.ParseCarbonDioxide},
}

func subscribe(t *topic, client mqtt.Client, sender *db.Storage) {
	token := client.Subscribe(t.topic, 0, func(client mqtt.Client, msg mqtt.Message) {
		if ms, err := t.mapper(msg.Payload()); err == nil {
			go sender.Enqueue(ms.Convert())
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
		return errors.Wrap(token.Error(), "mqtt connect failed")
	}

	return nil
}
