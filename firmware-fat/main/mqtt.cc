#include "mqtt.hh"
#include "common.hh"
#include "settings.hh"
#include "utils.hh"
#include <esp_log.h>

namespace mqtt {

esp_err_t Client::handleEvent(esp_mqtt_event_handle_t evt) {
  Client &client = *reinterpret_cast<Client *>(evt->user_context);

  switch (evt->event_id) {
  case MQTT_EVENT_CONNECTED:
    client.setState(MqttState::READY);
    client.subscribe("cmd/*", 2);
    client.subscribe(client.cmdTopic, 2);
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
    ESP_LOGI(logTag, "mqtt message received (id %d)", evt->msg_id);
    std::string topic{evt->topic, static_cast<unsigned>(evt->topic_len)};
    Message *const msg = new Message{
        .id = evt->msg_id,
        .topic = std::move(topic),
        .respTopic = topic == "cmd/*" ? "response/*" : client.respTopic,
        .data = std::string{evt->data, static_cast<unsigned>(evt->data_len)},
    };

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

Client::Client(std::string_view brokerUri, std::string_view caCert,
               std::string_view username, std::string_view password)
    : cmdTopic{std::string{"cmd/"}.append(username)},
      respTopic{std::string{"response/"}.append(username)}, cert{caCert},
      event{xEventGroupCreate()} {

  const esp_mqtt_client_config_t conf{
      .event_handle = handleEvent,
      .uri = brokerUri.data(),
      .username = username.data(),
      .password = password.data(),
      .keepalive = 30,
      .user_context = this,
      .cert_pem = caCert.data(),
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
  xEventGroupWaitBits(event, static_cast<EventBits_t>(MqttState::READY), false,
                      false, portMAX_DELAY);
}

bool Client::send(std::string_view topic, std::string_view data) {
  for (int result = -1; result < 0;) {
    result = esp_mqtt_client_publish(handle, topic.data(), data.data(),
                                     data.length(), 1, 1);
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

bool Client::subscribe(std::string_view topic, int qos) {
  return esp_mqtt_client_subscribe(handle, topic.data(), qos) != ESP_OK;
}

Message Client::receive() { return *msgQueue.take(); }

} // namespace mqtt
