#pragma once

#include <esp_err.h>
#include <nvs_handle.hpp>

struct AppSettings {
  const char *devName = CONFIG_DEV_NAME;
  struct {
    const char *ssid = CONFIG_WIFI_SSID;
    const char *pass = CONFIG_WIFI_PASSWORD;
  } wifi;
  struct {
    const char *broker = CONFIG_MQTT_BROKER_URI;
    const char *username = CONFIG_MQTT_USERNAME;
    const char *password = CONFIG_MQTT_PASSWORD;
  } mqtt;

  esp_err_t read();

  esp_err_t write(const char *name, const char *value);

  std::string format() const;
private:
  static esp_err_t readString(nvs::NVSHandle &nvs, const char *name,
                              const char *&dst);
};

extern AppSettings appSettings;
