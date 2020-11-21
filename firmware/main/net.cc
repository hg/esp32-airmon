#include "net.hh"
#include "common.hh"
#include "settings.hh"
#include "state.hh"
#include "utils.hh"
#include <esp_event.h>
#include <esp_tls.h>
#include <esp_wifi.h>

static void handleIpEvent([[maybe_unused]] void *const arg,
                          const esp_event_base_t event_base,
                          const int32_t event_id, void *const event_data) {
  configASSERT(event_base == IP_EVENT);

  switch (event_id) {
  case IP_EVENT_STA_GOT_IP: {
    appState->set(AppState::STATE_NET_CONNECTED);
    appState->reset(AppState::STATE_NET_DISCONNECTED);

    const ip_event_got_ip_t *const evt =
        reinterpret_cast<ip_event_got_ip_t *>(event_data);
    ESP_LOGI(logTag, "got ip %d.%d.%d.%d", IP2STR(&evt->ip_info.ip));
    break;
  }

  case IP_EVENT_STA_LOST_IP:
    appState->reset(AppState::STATE_NET_CONNECTED);
    appState->set(AppState::STATE_NET_DISCONNECTED);
    ESP_LOGI(logTag, "lost ip");
    break;

  default:
    ESP_LOGI(logTag, "unexpected ip event %d", event_id);
    break;
  }
}

static void handleWifiEvent([[maybe_unused]] void *const arg,
                            const esp_event_base_t event_base,
                            const int32_t event_id,
                            [[maybe_unused]] void *const event_data) {
  configASSERT(event_base == WIFI_EVENT);

  switch (event_id) {
  case WIFI_EVENT_STA_START:
    esp_wifi_connect();
    break;

  case WIFI_EVENT_STA_CONNECTED:
    ESP_LOGI(logTag, "connected to AP");
    break;

  case WIFI_EVENT_STA_DISCONNECTED:
    ESP_LOGI(logTag, "disconnected from AP");
    esp_wifi_connect();
    break;

  default:
    ESP_LOGI(logTag, "unexpected sta event %d", event_id);
    break;
  }
}

// https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/wifi.html#wi-fi-lwip-init-phase
void initWifi() {
  // initialize LwIP and main event loop
  ESP_ERROR_CHECK(esp_netif_init());
  ESP_ERROR_CHECK(esp_event_loop_create_default());

  // create station interface
  esp_netif_create_default_wifi_sta();

  // initialize Wi-Fi driver
  wifi_init_config_t wf_init_conf = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&wf_init_conf));

  // bind event handlers
  ESP_ERROR_CHECK(esp_event_handler_instance_register(
      WIFI_EVENT, ESP_EVENT_ANY_ID, handleWifiEvent, nullptr, nullptr));

  ESP_ERROR_CHECK(esp_event_handler_instance_register(
      IP_EVENT, ESP_EVENT_ANY_ID, handleIpEvent, nullptr, nullptr));

  // connect to station
  wifi_config_t wf_conf{
      .sta{
          .threshold =
              wifi_scan_threshold_t{
                  .authmode = WIFI_AUTH_WPA2_PSK,
              },
          .pmf_cfg{
              .capable = true,
              .required = false,
          },
      },
  };

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wstringop-truncation"
  strncpy(reinterpret_cast<char *>(wf_conf.sta.ssid),
          appSettings.wifi.ssid.c_str(), sizeof(wf_conf.sta.ssid));
  strncpy(reinterpret_cast<char *>(wf_conf.sta.password),
          appSettings.wifi.pass.c_str(), sizeof(wf_conf.sta.password));
#pragma GCC diagnostic pop

  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
  ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wf_conf));
  ESP_ERROR_CHECK(esp_wifi_start());
}
