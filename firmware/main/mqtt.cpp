#include "mqtt.h"
#include "common.h"
#include "esp_tls.h"
#include "utils.h"

#include <esp_log.h>

namespace mqtt {

void Client::handleEvent(void *event_handler_arg,
                         esp_event_base_t event_base,
                         int32_t event_id,
                         void *event_data) {
  Client &client = *static_cast<Client *>(event_handler_arg);
  auto evt = static_cast<esp_mqtt_event_handle_t>(event_data);

  switch (event_id) {
  case MQTT_EVENT_CONNECTED:
    client.setState(MqttState::READY);
    ESP_LOGI(logTag, "mqtt connected");
    break;

  case MQTT_EVENT_DISCONNECTED:
    client.clearState(MqttState::READY);
    ESP_LOGI(logTag, "mqtt disconnected");
    break;

  case MQTT_EVENT_PUBLISHED:
    ESP_LOGD(logTag, "mqtt broker received message %d", evt->msg_id);
    break;

  case MQTT_EVENT_DATA: {
    ESP_LOGI(logTag, "mqtt message ignored (id %d)", evt->msg_id);
    break;
  }

  case MQTT_EVENT_SUBSCRIBED:
    ESP_LOGI(logTag, "mqtt subscription successful (%d)", evt->msg_id);
    break;

  case MQTT_EVENT_ERROR:
    ESP_LOGE(logTag, "mqtt error %d", evt->error_handle->error_type);
    break;

  default:
    break;
  }
}

// convert hex string to array of bytes and return how many bytes were stored
static size_t hex_to_bytes(const char *str, uint8_t *buf) {
  const size_t hex_len = strlen(str);
  configASSERT(hex_len % 2 == 0);

  const size_t psk_len = hex_len / 2;

  for (size_t i = 0; i < psk_len; ++i) {
    char b[3] = {0, 0, 0};
    b[0] = str[i * 2];
    b[1] = str[i * 2 + 1];
    buf[i] = strtol(b, nullptr, 16);
  }

  return psk_len;
}

Client::Client(const char *brokerUri, const char *hint, const char *psk)
  : event{xEventGroupCreate()} {

  uint8_t *buf = new uint8_t[strlen(psk) / 2];
  size_t psk_len = hex_to_bytes(psk, buf);

  auto key = new psk_hint_key_t{
      .key = buf,
      .key_size = psk_len,
      .hint = hint,
  };

  const esp_mqtt_client_config_t conf{
      .broker = {
          .address = {.uri = brokerUri},
          .verification = {.psk_hint_key = key},
      },
  };
  handle = esp_mqtt_client_init(&conf);
  configASSERT(handle);

  esp_mqtt_client_register_event(handle, MQTT_EVENT_ANY, handleEvent, this);

  ESP_ERROR_CHECK(esp_mqtt_client_start(handle));
}

Client::~Client() {
  esp_mqtt_client_stop(handle);
  esp_mqtt_client_destroy(handle);
  vEventGroupDelete(event);
}

void Client::waitReady() const {
  xEventGroupWaitBits(event, static_cast<EventBits_t>(MqttState::READY), false,
                      false, portMAX_DELAY);
}

bool Client::send(const std::string_view topic,
                  const std::string_view data) const {
  for (int i = 0; i < 10; ++i) {
    int msgId = esp_mqtt_client_publish(handle, topic.data(), data.data(),
                                        data.length(), 1, 1);
    if (msgId < 0) {
      ESP_LOGI(logTag, "mqtt publish failed, retrying");
      vTaskDelay(seconds(1));
    } else {
      return true;
    }
  }

  return false;
}

void Client::setState(const MqttState bits) const {
  xEventGroupSetBits(event, static_cast<EventBits_t>(bits));
}

void Client::clearState(const MqttState bits) const {
  xEventGroupClearBits(event, static_cast<EventBits_t>(bits));
}

} // namespace mqtt