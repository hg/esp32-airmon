#pragma once

#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>
#include <mqtt_client.h>
#include <string>
#include <vector>

namespace mqtt {

enum class MqttState : EventBits_t {
  Ready = BIT0,
};

class Client {
public:
  Client(const char *brokerUri, const char *caCert, const char *username,
         const char *password);

  Client(const Client &) = delete;

  ~Client();

  Client &operator=(const Client &) = delete;

  void waitReady() const;

  bool send(const char *topic, const char *data);

private:
  void clearState(MqttState bits);

  void setState(MqttState bits);

  void subscribeToCommands();

  bool handleMessage(esp_mqtt_event_handle_t evt);

  bool handleOta(const char *respTopic, const std::vector<std::string> &args);

  bool handleWriteSetting(const char *respTopic,
                          const std::vector<std::string> &args);

  [[noreturn]] bool handleRestart(const char *respTopic);

  bool handleUnknown(const char *respTopic);

  bool handlePing(esp_mqtt_event_handle_t evt, const char *respTopic);

  static esp_err_t handleEvent(esp_mqtt_event_handle_t evt);

  esp_mqtt_client_handle_t handle;
  EventGroupHandle_t event;
  const char *cert;
  std::string cmdTopic;
  std::string respTopic;
};

} // namespace mqtt
