#pragma once

#include <esp_err.h>
#include <nvs_handle.hpp>

struct AppSettings {
  std::string devName{CONFIG_DEV_NAME};

  struct {
    std::string ssid{CONFIG_WIFI_SSID};
    std::string pass{CONFIG_WIFI_PASSWORD};
  } wifi;

  struct {
    std::string broker{CONFIG_MQTT_BROKER_URI};
    std::string username{CONFIG_MQTT_USERNAME};
    std::string password{CONFIG_MQTT_PASSWORD};
  } mqtt;

  struct {
    int pm{CONFIG_PARTICULATE_PERIOD_SECONDS};
    int temp{CONFIG_TEMPERATURE_PERIOD_SECONDS};
  } period;

  esp_err_t read();

  esp_err_t write() const;

  bool set(std::string_view out, std::string_view value);

  [[nodiscard]] std::string format() const;
};

extern AppSettings appSettings;
