#include "mqtt.hh"
#include "common.hh"
#include "settings.hh"
#include "utils.hh"
#include <esp_log.h>

namespace mqtt {

esp_err_t Client::handleEvent(const esp_mqtt_event_handle_t evt) {
  Client &client = *reinterpret_cast<Client *>(evt->user_context);

  switch (evt->event_id) {
  case MQTT_EVENT_CONNECTED:
    client.setState(MqttState::Ready);
    client.subscribe("cmd/*", 2);
    client.subscribe(client.cmdTopic.c_str(), 2);
    ESP_LOGI(logTag, "mqtt connected");
    break;

  case MQTT_EVENT_DISCONNECTED:
    client.clearState(MqttState::Ready);
    ESP_LOGI(logTag, "mqtt disconnected");
    break;

  case MQTT_EVENT_PUBLISHED:
    ESP_LOGD(logTag, "mqtt broker received message %d", evt->msg_id);
    break;

  case MQTT_EVENT_DATA: {
    ESP_LOGI(logTag, "mqtt message received (id %d)", evt->msg_id);
    auto *const msg = new Message{
        .id = evt->msg_id,
        .topic = std::string{evt->topic, static_cast<unsigned>(evt->topic_len)},
        .data = std::string{evt->data, static_cast<unsigned>(evt->data_len)}};
    msg->respTopic = msg->isBroadcast() ? "response/*" : client.respTopic;
    client.msgQueue.put(msg);
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

  return ESP_OK;
}

Client::Client(const char *const brokerUri, const char *const caCert,
               const char *const username, const char *const password) {
  cmdTopic = std::string{"cmd/"} + username;
  respTopic = std::string{"response/"} + username;
  cert = caCert;
  event = xEventGroupCreate();

  const esp_mqtt_client_config_t conf{
      .event_handle = handleEvent,
      .uri = brokerUri,
      .username = username,
      .password = password,
      .keepalive = 30,
      .user_context = this,
      .cert_pem = caCert,
  };
  handle = esp_mqtt_client_init(&conf);
  configASSERT(handle);

  ESP_ERROR_CHECK(esp_mqtt_client_start(handle));
}

Client::~Client() {
  esp_mqtt_client_stop(handle);
  esp_mqtt_client_destroy(handle);
  vEventGroupDelete(event);
}

void Client::waitReady() const {
  xEventGroupWaitBits(event, static_cast<EventBits_t>(MqttState::Ready), false,
                      false, portMAX_DELAY);
}

bool Client::send(const std::string &topic, const char *const data) {
  for (int result = -1; result < 0;) {
    result = esp_mqtt_client_publish(handle, topic.c_str(), data, 0, 1, 1);
    if (result < 0) {
      ESP_LOGI(logTag, "mqtt publish failed, retrying");
      vTaskDelay(secToTicks(5));
    }
  }

  return true;
}

void Client::setState(const MqttState bits) {
  xEventGroupSetBits(event, static_cast<EventBits_t>(bits));
}

void Client::clearState(const MqttState bits) {
  xEventGroupClearBits(event, static_cast<EventBits_t>(bits));
}

bool Client::subscribe(const char *topic, int qos) {
  return esp_mqtt_client_subscribe(handle, topic, qos) != ESP_OK;
}

Message Client::receive() { return *msgQueue.take(); }

} // namespace mqtt
