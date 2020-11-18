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
  int id;
  std::string topic;
  std::string respTopic;
  std::string data;

  [[nodiscard]] bool isBroadcast() const { return topic == "cmd/*"; }
};

class Client {
public:
  Client(const char *brokerUri, const char *caCert, const char *username,
         const char *password);

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
  EventGroupHandle_t event;
  const char *cert;
  std::string cmdTopic;
  std::string respTopic;
};

} // namespace mqtt
