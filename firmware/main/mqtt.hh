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
  Client(const char *brokerUri, const char *hint, const char *psk);
  Client(const Client &) = delete;
  ~Client();

  Client &operator=(const Client &) = delete;

  void waitReady() const;
  bool send(std::string_view topic, std::string_view data) const;
  [[nodiscard]] Message receive();
  bool subscribe(std::string_view topic, int qos) const;

private:
  void clearState(MqttState bits) const;
  void setState(MqttState bits) const;
  static esp_err_t handleEvent(esp_mqtt_event_handle_t evt);

  Queue<Message *> msgQueue{10};
  esp_mqtt_client_handle_t handle;
  EventGroupHandle_t event;
};

} // namespace mqtt
