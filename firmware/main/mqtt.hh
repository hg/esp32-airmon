#pragma once

#include "queue.hh"
#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>
#include <mqtt_client.h>
#include <string>
#include <string_view>
#include <vector>

namespace mqtt {

enum class MqttState : EventBits_t {
  READY = BIT0,
};

struct Message {
  const int id;
  const std::string topic;
  const std::string respTopic;
  const std::string data;
};

class Client {
public:
  Client(std::string_view brokerUri, std::string_view caCert,
         std::string_view username, std::string_view password);

  Client(const Client &) = delete;

  ~Client();

  Client &operator=(const Client &) = delete;

  void waitReady() const;

  bool send(std::string_view topic, std::string_view data);

  [[nodiscard]] Message receive();

  bool subscribe(std::string_view topic, int qos);

private:
  void clearState(MqttState bits);

  void setState(MqttState bits);

  static esp_err_t handleEvent(esp_mqtt_event_handle_t evt);

  Queue<Message *> msgQueue{10};
  esp_mqtt_client_handle_t handle;
  const std::string cmdTopic;
  const std::string respTopic;
  const std::string_view cert;
  const EventGroupHandle_t event;
};

} // namespace mqtt
