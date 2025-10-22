#include "ca.hh"
#include "common.hh"
#include "mqtt.hh"
#include "settings.hh"
#include "utils.hh"
#include <algorithm>
#include <esp_http_client.h>
#include <esp_https_ota.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

using CommandArgs = std::vector<std::string_view>;

static bool handlePing(mqtt::Client &client, const mqtt::Message &msg) {
  return client.send(msg.respTopic, appSettings.devName);
}

[[noreturn]] static bool handleRestart(mqtt::Client &client,
                                       const mqtt::Message &msg) {
  client.send(msg.respTopic, "restarting");
  esp_restart();
}

static bool handleReadSettings(mqtt::Client &client, const mqtt::Message &msg) {
  const std::string json = appSettings.format();
  client.send(msg.respTopic, json);
  return true;
}

static bool handleWriteSettings(mqtt::Client &client,
                                const mqtt::Message &msg) {
  const esp_err_t err = appSettings.write();
  const char *const resp =
      err == ESP_OK ? "settings saved" : "could not write settings";
  return client.send(msg.respTopic, resp);
}

static bool handleSetSetting(mqtt::Client &client, const mqtt::Message &msg,
                             const CommandArgs &args) {
  if (args.size() != 3) {
    client.send(msg.respTopic, "usage: setting/set name_no_spaces value_also");
    return false;
  }
  const esp_err_t err = appSettings.set(args[1], args[2]);
  if (err == ESP_OK) {
    client.send(msg.respTopic, "setting set");
  } else {
    client.send(msg.respTopic, "setting write failed");
  }
  return true;
}

static bool handleOta(mqtt::Client &client, const mqtt::Message &msg,
                      const CommandArgs &args) {
  if (args.size() != 2) {
    client.send(msg.respTopic, "usage: ota https://server/path.bin");
    return false;
  }

  const esp_http_client_config_t config{
      .url = args[1].data(),
      .cert_pem = caPemStart,
  };

  const esp_err_t ret = esp_https_ota(&config);

  if (ret == ESP_OK) {
    ESP_LOGI(logTag, "OTA update finished");
    client.send(msg.respTopic, "OTA success");
    esp_restart();
  }

  ESP_LOGE(logTag, "could not perform OTA update: 0x%x", ret);
  client.send(msg.respTopic, "OTA failed");

  return false;
}

static bool handleUnknown(mqtt::Client &client, const mqtt::Message &msg) {
  client.send(msg.respTopic, "unknown command");
  return false;
}

static bool handleMessage(mqtt::Client &client, const mqtt::Message &msg) {
  CommandArgs tokens;

  const auto end = msg.data.cend();
  for (auto begin = msg.data.cbegin(); begin != end;) {
    const auto nextSpace = std::find_if(begin, end, isspace);
    if (nextSpace != begin) {
      const std::string_view::size_type len = nextSpace - begin;
      tokens.emplace_back(begin.base(), len);
      if (nextSpace == end) {
        break;
      }
    }
    begin = std::find_if(nextSpace, end, isgraph);
  }

  if (tokens.empty()) {
    client.send(msg.respTopic, "no command specified");
    return false;
  }

  const std::string_view &command = tokens.front();

  if (command == "ota") {
    return handleOta(client, msg, tokens);
  }
  if (command == "ping") {
    return handlePing(client, msg);
  }
  if (command == "restart") {
    return handleRestart(client, msg);
  }
  if (command == "setting/set") {
    return handleSetSetting(client, msg, tokens);
  }
  if (command == "setting/get") {
    return handleReadSettings(client, msg);
  }
  if (command == "setting/write") {
    return handleWriteSettings(client, msg);
  }
  return handleUnknown(client, msg);
}

[[noreturn]] void taskCommandHandler(void *const arg) {
  mqtt::Client &client = *reinterpret_cast<mqtt::Client *>(arg);

  while (true) {
    const mqtt::Message msg = client.receive();
    if (!handleMessage(client, msg)) {
      ESP_LOGE(logTag, "could not handle message");
    }
  }
}

void initCommandHandler(mqtt::Client &client) {
  xTaskCreate(taskCommandHandler, "mqtt_cmd", KiB(4), &client, 4, nullptr);
}
