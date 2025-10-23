#include "chrono.h"
#include "common.h"
#include "state.h"
#include "utils.h"
#include <esp_log.h>
#include <esp_sntp.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// UNIX time when the system was booted
static volatile time_t bootTimestamp = 0;

static constexpr bool isValidTimestamp(const time_t ts) {
  return ts >= 1577836800; // 2020-01-01 UTC
}

// fixes time if it was assigned before SNTP time was available
void fixInvalidTime(time_t &tm) {
  if (!isValidTimestamp(tm)) {
    tm += bootTimestamp;
  }
}

time_t getTimestamp() {
  if (bootTimestamp > 0) {
    return time(nullptr);
  }
  constexpr int64_t usPerSec = 1000 * 1000;
  return esp_timer_get_time() / usPerSec;
}

static void onTimeUpdated(timeval *const tm) {
  if (bootTimestamp == 0) {
    bootTimestamp = time(nullptr);
    state->set(AppState::STATE_TIME_VALID);
  }
  ESP_LOGI(logTag, "sntp time update finished");
}

static void taskSntpUpdate(void *const arg) {
  state->wait(AppState::STATE_NET_CONNECTED);

  esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
  esp_sntp_setservername(0, "pool.ntp.org");
  sntp_set_time_sync_notification_cb(onTimeUpdated);
  esp_sntp_init();

  ESP_LOGI(logTag, "sntp time update started");

  vTaskDelete(nullptr);
}

void startSntp() {
  xTaskCreate(taskSntpUpdate, "sntp_update", KiB(2), nullptr, 1, nullptr);
}