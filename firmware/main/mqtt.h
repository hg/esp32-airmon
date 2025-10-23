#pragma once

#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>
#include <mqtt_client.h>
#include <string>
#include <string_view>

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

private:
  void clearState(MqttState bits) const;
  void setState(MqttState bits) const;
  static void handleEvent(void *event_handler_arg,
                          esp_event_base_t event_bse,
                          int32_t event_id,
                          void *event_data);
  esp_mqtt_client_handle_t handle;
  EventGroupHandle_t event;
};

} // namespace mqtt