package mqtt

import (
	mqtt "github.com/eclipse/paho.mqtt.golang"
	"github.com/hg/airmon/influx"
	"github.com/hg/airmon/mon"
	"github.com/pkg/errors"
	"log"
	"os"
	"strings"
)

type Settings struct {
	Broker string
	User   string
	Pass   string
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
		log.Print("using MQTT username ", s.User)
	}
	if s.Pass != "" {
		log.Print("using MQTT password ", strings.Repeat("*", len(s.Pass)))
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
		log.Print("mqtt msg received:", string(msg.Payload()))
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
			log.Print("could not parse ", t.topic, " data: ", err)
		}
	})

	if token.Wait() && token.Error() != nil {
		log.Print(errors.Wrap(token.Error(), "could not subscribe to mqtt topic "+t.topic))
	}
}

func (h *connHandler) onConnect(client mqtt.Client) {
	for _, t := range topics {
		log.Print("subscribing to topic ", t.topic)
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
