#include "net.h"
#include "common.h"
#include "state.h"
#include <esp_event.h>
#include <esp_tls.h>
#include <esp_wifi.h>

static const char *TAG = "air/net";

static void on_ip_event(void *arg, esp_event_base_t event_base,
                        int32_t event_id, void *event_data) {
  configASSERT(event_base == IP_EVENT);

  app_state *state = arg;

  switch (event_id) {
  case IP_EVENT_STA_GOT_IP: {
    state_set(state, APP_ONLINE);
    ip_event_got_ip_t *evt = event_data;
    ESP_LOGI(TAG, "got ip %d.%d.%d.%d", IP2STR(&evt->ip_info.ip));
    break;
  }

  case IP_EVENT_STA_LOST_IP:
    state_reset(state, APP_ONLINE);
    ESP_LOGI(TAG, "lost ip");
    break;

  default:
    ESP_LOGI(TAG, "unexpected ip event %d", event_id);
    break;
  }
}

static void on_wifi_event(void *arg, esp_event_base_t event_base,
                          int32_t event_id, void *event_data) {
  configASSERT(event_base == WIFI_EVENT);

  switch (event_id) {
  case WIFI_EVENT_STA_START:
    ESP_LOGI(TAG, "connecting to WiFi");
    esp_wifi_connect();
    break;

  case WIFI_EVENT_STA_CONNECTED:
    ESP_LOGI(TAG, "WiFi connected");
    break;

  case WIFI_EVENT_STA_DISCONNECTED:
    ESP_LOGI(TAG, "WiFi disconnected");
    esp_wifi_connect();
    break;

  default:
    ESP_LOGI(TAG, "unexpected sta event %d", event_id);
    break;
  }
}

static_assert(sizeof(CONFIG_WIFI_SSID) <= sizeof((wifi_config_t){}.sta.ssid),
              "WiFi net name is too long");

static_assert(sizeof(CONFIG_WIFI_PASSWORD) <=
                  sizeof((wifi_config_t){}.sta.password),
              "WiFi password is too long");

// https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/wifi.html#wi-fi-lwip-init-phase
void init_wifi(app_state *state) {
  // initialize LwIP and main event loop
  esp_err_t err;

  err = esp_netif_init();
  ESP_ERROR_CHECK(err);

  err = esp_event_loop_create_default();
  ESP_ERROR_CHECK(err);

  // create station interface
  esp_netif_t *netif = esp_netif_create_default_wifi_sta();

  // initialize Wi-Fi driver
  wifi_init_config_t wf_init_conf = WIFI_INIT_CONFIG_DEFAULT();
  err = esp_wifi_init(&wf_init_conf);
  ESP_ERROR_CHECK(err);

  // bind event handlers
  err = esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                            on_wifi_event, NULL, NULL);
  ESP_ERROR_CHECK(err);

  err = esp_event_handler_instance_register(IP_EVENT, ESP_EVENT_ANY_ID,
                                            on_ip_event, state, NULL);
  ESP_ERROR_CHECK(err);

  // connect to station
  wifi_config_t wf_conf = {
      .sta =
          {
              .threshold =
                  {
                      .authmode = WIFI_AUTH_WPA2_PSK,
                  },
              .pmf_cfg =
                  {
                      .capable = true,
                      .required = false,
                  },
          },
  };

  strncpy((char *)wf_conf.sta.ssid, CONFIG_WIFI_SSID, sizeof(wf_conf.sta.ssid));

  strncpy((char *)wf_conf.sta.password, CONFIG_WIFI_PASSWORD,
          sizeof(wf_conf.sta.password));

  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
  ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wf_conf));
  ESP_ERROR_CHECK(esp_wifi_start());

  esp_netif_set_hostname(netif, CONFIG_DEV_NAME);
}
