#include "chrono.h"
#include "common.h"
#include "esp_netif_sntp.h"
#include "state.h"
#include "utils.h"
#include <esp_log.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

static const char *TAG = "air/time";

// time when the system was booted
static volatile time_t booted_at = 0;

static time_t runtime_seconds() {
  int64_t us_per_sec = 1000 * 1000;
  return esp_timer_get_time() / us_per_sec;
}

static bool is_valid_time(time_t ts) {
  // date --date=2025-01-01Z +%s
  return ts >= 1735689600;
}

// fix time assigned before SNTP sync
time_t fix_time(time_t tm) {
  if (is_valid_time(tm)) {
    return tm;
  }
  time_t adj = tm + booted_at;
  ESP_LOGI(TAG, "fixed time %lld into %lld", tm, adj);
  return adj;
}

time_t get_timestamp() {
  if (booted_at > 0) {
    return time(NULL);
  }
  return runtime_seconds();
}

static void on_time_updated(app_state *state) {
  if (booted_at == 0) {
    booted_at = time(NULL) - runtime_seconds();
    state_set(state, APP_TIME_VALID);
    ESP_LOGI(TAG, "set boot time to %lld", booted_at);
  }
}

[[noreturn]]
static void task_sntp(void *arg) {
  app_state *state = arg;

  state_wait(state, APP_ONLINE);

  esp_sntp_config_t config = ESP_NETIF_SNTP_DEFAULT_CONFIG("pool.ntp.org");
  esp_netif_sntp_init(&config);

  while (true) {
    esp_err_t err = esp_netif_sntp_sync_wait(portMAX_DELAY);
    if (err == ESP_OK) {
      break;
    }
    ESP_LOGW(TAG, "time not synced: 0x%x", err);
  }

  on_time_updated(state);

  // do not deactivate SNTP, let it update time periodically
  vTaskDelete(NULL);
  configASSERT(false);
}

void sntp_start(app_state *state) {
  xTaskCreate(task_sntp, "sntp", fromKiB(2), state, 1, NULL);
}
