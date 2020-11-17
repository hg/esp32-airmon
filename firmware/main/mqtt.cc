#include "mqtt.hh"
#include "common.hh"
#include "settings.hh"
#include "utils.hh"
#include <esp_http_client.h>
#include <esp_https_ota.h>
#include <esp_log.h>

static bool isBroadcastCmd(const esp_mqtt_event_handle_t evt) {
  return strncmp(evt->topic, "cmd/*", evt->topic_len) == 0;
}

namespace mqtt {

esp_err_t Client::handleEvent(const esp_mqtt_event_handle_t evt) {
  Client &client = *reinterpret_cast<Client *>(evt->user_context);

  switch (evt->event_id) {
  case MQTT_EVENT_CONNECTED:
    client.setState(MqttState::Ready);
    client.subscribeToCommands();
    ESP_LOGI(logTag, "mqtt connected");
    break;

  case MQTT_EVENT_DISCONNECTED:
    client.clearState(MqttState::Ready);
    ESP_LOGI(logTag, "mqtt disconnected");
    break;

  case MQTT_EVENT_PUBLISHED:
    ESP_LOGD(logTag, "mqtt broker received message %d", evt->msg_id);
    break;

  case MQTT_EVENT_DATA:
    ESP_LOGI(logTag, "mqtt message received (id %d)", evt->msg_id);
    if (!client.handleMessage(evt)) {
      ESP_LOGI(logTag, "could not handle message (id %d)", evt->msg_id);
    }
    break;

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

bool Client::send(const char *const topic, const char *const data) {
  for (int result = -1; result < 0;) {
    result = esp_mqtt_client_publish(handle, topic, data, 0, 1, 1);
    if (result < 0) {
      ESP_LOGI(logTag, "mqtt publish failed, retrying");
      vTaskDelay(secToTicks(5));
    }
  }

  return true;
}

bool Client::handlePing(const esp_mqtt_event_handle_t evt,
                        const char *const respTopic) {
  const char *data;
  char buf[64];

  if (isBroadcastCmd(evt)) {
    snprintf(buf, sizeof(buf), "pong: %s", appSettings.devName);
    data = buf;
  } else {
    data = "pong";
  }

  return send(respTopic, data);
}

bool Client::handleUnknown(const char *const respTopic) {
  send(respTopic, "unknown command");
  return false;
}

[[noreturn]] bool Client::handleRestart(const char *const respTopic) {
  send(respTopic, "restarting");
  esp_restart();
}

bool Client::handleWriteSetting(const char *const respTopic,
                                const std::vector<std::string> &args) {
  if (args.size() != 3) {
    send(respTopic, "usage: setting/set name_without_spaces value_too");
    return false;
  }
  const esp_err_t err = appSettings.write(args[1].c_str(), args[2].c_str());
  if (err == ESP_OK) {
    send(respTopic, "setting set");
  } else {
    send(respTopic, "setting write failed");
  }
  return true;
}

bool Client::handleOta(const char *const respTopic,
                       const std::vector<std::string> &args) {
  if (args.size() != 2) {
    send(respTopic, "usage: ota https://server/path.bin");
    return false;
  }

  const esp_http_client_config_t config{
      .url = args[1].c_str(),
      .cert_pem = cert,
  };

  const esp_err_t ret = esp_https_ota(&config);

  if (ret == ESP_OK) {
    ESP_LOGI(logTag, "OTA update finished");
    send(respTopic, "OTA success");
    esp_restart();
  }

  ESP_LOGE(logTag, "could not perform OTA update: 0x%x", ret);
  send(respTopic, "OTA failed");

  return false;
}

bool Client::handleMessage(const esp_mqtt_event_handle_t evt) {
  const char *const topic =
      isBroadcastCmd(evt) ? "response/*" : respTopic.c_str();

  char cmd[evt->data_len + 1];
  memcpy(cmd, evt->data, evt->data_len);
  cmd[sizeof(cmd) - 1] = '\0';

  std::vector<std::string> tokens;

  for (const char *tok = strtok(cmd, " \t"); tok != nullptr;
       tok = strtok(nullptr, " \t")) {
    tokens.push_back(tok);
  }

  if (tokens.size() < 1) {
    send(topic, "no command specified");
    return false;
  }

  const std::string &command = tokens[0];

  if (command == "ota") {
    return handleOta(topic, tokens);
  }
  if (command == "ping") {
    return handlePing(evt, topic);
  }
  if (command == "restart") {
    return handleRestart(topic);
  }
  return handleUnknown(topic);
}

void Client::subscribeToCommands() {
  esp_mqtt_client_subscribe(handle, "cmd/*", 2);
  esp_mqtt_client_subscribe(handle, cmdTopic.c_str(), 2);
}

void Client::setState(const MqttState bits) {
  xEventGroupSetBits(event, static_cast<EventBits_t>(bits));
}

void Client::clearState(const MqttState bits) {
  xEventGroupClearBits(event, static_cast<EventBits_t>(bits));
}

} // namespace mqtt
