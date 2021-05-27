package mqtt

import (
	"flag"
	"os"
	"strings"

	mqtt "github.com/eclipse/paho.mqtt.golang"
	"github.com/hg/airmon/influx"
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

func (s *Settings) AddFlags() {
	flag.StringVar(&s.Broker, "mqtt.broker", "tcp://localhost:1883", "the broker URI")
	flag.StringVar(&s.User, "mqtt.user", "", "MQTT username")
	flag.StringVar(&s.Pass, "mqtt.pass", "", "MQTT password")
}

func (s *Settings) SetFromEnvironment() {
	if s.Broker == "" {
		s.Broker = os.Getenv("MQTT_BROKER")
	}
	if s.User == "" {
		s.User = os.Getenv("MQTT_USER")
	}
	if s.Pass == "" {
		s.Pass = os.Getenv("MQTT_PASS")
	}
}

func (s *Settings) validate() error {
	if s.Broker == "" {
		return errors.New("MQTT broker is empty")
	}
	if s.User != "" {
		log.Info("using username", zap.String("username", s.User))
	}
	if s.Pass != "" {
		log.Info("using MQTT password",
			zap.String("password", strings.Repeat("*", len(s.Pass))))
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
	sender *influx.MeasurementSender
}

type topic struct {
	topic  string
	mapper func(data []byte) (mon.PointSource, error)
}

var topics = []*topic{
	{"meas/part", mon.ParseParticulates},
	{"meas/temp", mon.ParseTemperature},
	{"meas/co2", mon.ParseCarbonDioxide},
}

func subscribe(t *topic, client mqtt.Client, sender *influx.MeasurementSender) {
	token := client.Subscribe(t.topic, 0, func(client mqtt.Client, msg mqtt.Message) {
		if ms, err := t.mapper(msg.Payload()); err == nil {
			sender.Send(ms.ToPoint())
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

func StartMqtt(settings *Settings, sender *influx.MeasurementSender) error {
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
