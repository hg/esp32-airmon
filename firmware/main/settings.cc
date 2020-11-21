#include "settings.hh"
#include "common.hh"
#include <charconv>
#include <cstring>

AppSettings appSettings;

static esp_err_t readString(nvs::NVSHandle &nvs, std::string_view name,
                            std::string &dst) {
  size_t len;
  esp_err_t err = nvs.get_item_size(nvs::ItemType::SZ, name.data(), len);

  if (err != ESP_OK) {
    LOG_ERR(err, "could not get setting size");
    return err;
  }

  dst.reserve(len);
  err = nvs.get_string(name.data(), &dst[0], len);

  if (err == ESP_OK) {
    ESP_LOGI(logTag, "read setting %s: [%s]", name.data(), dst.c_str());
  } else {
    LOG_ERR(err, "could not read setting %s", name.data());
  }

  return err;
}

esp_err_t AppSettings::read() {
  esp_err_t err;
  const std::unique_ptr<nvs::NVSHandle> nvs =
      nvs::open_nvs_handle("storage", NVS_READONLY, &err);

  if (err != ESP_OK) {
    LOG_ERR(err, "could not open NVS for reading");
    return err;
  }

  bool anyError = (readString(*nvs, "devName", devName) != ESP_OK) |
                  (readString(*nvs, "wifi.ssid", wifi.ssid) != ESP_OK) |
                  (readString(*nvs, "wifi.pass", wifi.pass) != ESP_OK) |
                  (readString(*nvs, "mqtt.broker", mqtt.broker) != ESP_OK) |
                  (readString(*nvs, "mqtt.username", mqtt.username) != ESP_OK) |
                  (readString(*nvs, "mqtt.password", mqtt.password) != ESP_OK) |
                  (nvs->get_item("period.pm", period.pm) != ESP_OK) |
                  (nvs->get_item("period.temp", period.temp) != ESP_OK);

  return anyError ? ESP_FAIL : ESP_OK;
}

esp_err_t AppSettings::write() {
  esp_err_t err;
  const std::unique_ptr<nvs::NVSHandle> nvs =
      nvs::open_nvs_handle("storage", NVS_READWRITE, &err);

  if (err != ESP_OK) {
    LOG_ERR(err, "could not open NVS for writing");
    return err;
  }

  bool anyError =
      (nvs->set_string("devName", devName.c_str()) != ESP_OK) |
      (nvs->set_string("wifi.ssid", wifi.ssid.c_str()) != ESP_OK) |
      (nvs->set_string("wifi.pass", wifi.pass.c_str()) != ESP_OK) |
      (nvs->set_string("mqtt.broker", mqtt.broker.c_str()) != ESP_OK) |
      (nvs->set_string("mqtt.username", mqtt.username.c_str()) != ESP_OK) |
      (nvs->set_string("mqtt.password", mqtt.password.c_str()) != ESP_OK) |
      (nvs->set_item("period.pm", period.pm) != ESP_OK) |
      (nvs->set_item("period.temp", period.temp) != ESP_OK);

  return anyError ? ESP_FAIL : ESP_OK;
}

bool AppSettings::set(std::string_view name, std::string_view value) {
  const auto setInt = [&value](int &out) -> bool {
    const auto result =
        std::from_chars(value.data(), value.data() + value.size(), out);
    return result.ec != std::errc::invalid_argument;
  };
  if (name == "devName") {
    devName = value;
  } else if (name == "wifi.ssid") {
    wifi.ssid = (value);
  } else if (name == "wifi.pass") {
    wifi.pass = value;
  } else if (name == "mqtt.broker") {
    mqtt.broker = value;
  } else if (name == "mqtt.username") {
    mqtt.username = value;
  } else if (name == "mqtt.password") {
    mqtt.password = value;
  } else if (name == "period.pm") {
    return setInt(period.pm);
  } else if (name == "period.temp") {
    return setInt(period.temp);
  } else {
    ESP_LOGE(logTag, "unknown setting %s", name.data());
    return false;
  }
  return true;
}

std::string AppSettings::format() const {
  constexpr auto tpl =
      R"({"dev":"%s","wifi":{"ssid":"%s","pass":"%s"},"mqtt":{"broker":"%s","user":"%s","pass":"%s"},{"period":{"pm":%d,"temp":%d}}})";
  const size_t size = strlen(tpl) + devName.size() + wifi.ssid.size() +
                      wifi.pass.size() + mqtt.broker.size() +
                      mqtt.username.size() + mqtt.password.size() +
                      10 * 2 + // space for each int
                      1;       // '\0'
  std::string json;
  json.reserve(size);
  snprintf(&json[0], json.capacity(), tpl, devName.c_str(), wifi.ssid.c_str(),
           wifi.pass.c_str(), mqtt.broker.c_str(), mqtt.username.c_str(),
           mqtt.password.c_str(), period.pm, period.temp);
  return json;
}
