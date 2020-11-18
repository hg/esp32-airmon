#include "settings.hh"
#include "common.hh"
#include <cstring>

AppSettings appSettings;

esp_err_t AppSettings::readString(nvs::NVSHandle &nvs, std::string_view name,
                                  const char *&dst) {
  size_t len;
  esp_err_t err = nvs.get_item_size(nvs::ItemType::SZ, name.data(), len);

  if (err != ESP_OK) {
    LOG_ERR(err, "could not get setting size");
    return err;
  }

  char *const value = new char[len];
  err = nvs.get_string(name.data(), value, len);

  if (err == ESP_OK) {
    dst = value;
    ESP_LOGI(logTag, "read setting %s: [%s]", name.data(), value);
  } else {
    delete[] value;
    LOG_ERR(err, "could not read setting %s", name.data());
  }

  return err;
}

esp_err_t AppSettings::read() {
  esp_err_t err;
  std::unique_ptr<nvs::NVSHandle> nvs =
      nvs::open_nvs_handle("storage", NVS_READONLY, &err);

  if (err != ESP_OK) {
    LOG_ERR(err, "could not open NVS for reading");
    return err;
  }

  readString(*nvs, "dev/name", devName);
  readString(*nvs, "wifi/ssid", wifi.ssid);
  readString(*nvs, "wifi/pass", wifi.pass);
  readString(*nvs, "mqtt/broker", mqtt.broker);
  readString(*nvs, "mqtt/username", mqtt.username);
  readString(*nvs, "mqtt/password", mqtt.password);

  return ESP_OK;
}

esp_err_t AppSettings::write(std::string_view name, std::string_view value) {
  esp_err_t err;
  const std::unique_ptr<nvs::NVSHandle> nvs =
      nvs::open_nvs_handle("storage", NVS_READWRITE, &err);

  if (err != ESP_OK) {
    LOG_ERR(err, "could not open NVS");
    return err;
  }
  err = nvs->set_string(name.data(), value.data());
  if (err != ESP_OK) {
    LOG_ERR(err, "could not write to NVS");
    return err;
  }
  err = nvs->commit();
  if (err != ESP_OK) {
    LOG_ERR(err, "NVS commit failed");
  }
  return err;
}

std::string AppSettings::format() const {
  constexpr auto tpl =
      R"({"dev":"%s","wifi":{"ssid":"%s","pass":"%s"},"mqtt":{"broker":"%s","user":"%s","pass":"%s"}})";
  // this is pretty slow, but we rarely call this command, and it will prevent
  // using more heap than necessary
  const size_t size = strlen(tpl) + strlen(devName) + strlen(wifi.ssid) +
                      strlen(wifi.pass) + strlen(mqtt.broker) +
                      strlen(mqtt.username) + strlen(mqtt.password);
  std::string json;
  json.reserve(size);
  snprintf(&json[0], json.capacity(), tpl, devName, wifi.ssid, wifi.pass,
           mqtt.broker, mqtt.username, mqtt.password);
  return json;
}
