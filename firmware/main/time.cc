#include "time.hh"
#include "common.hh"
#include "state.hh"
#include "utils.hh"
#include <esp_log.h>
#include <esp_sntp.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

volatile time_t bootTimestamp = 0;

time_t getTimestamp() {
  if (bootTimestamp > 0) {
    return time(nullptr);
  }
  constexpr int64_t usPerSec = 1000 * 1000;
  return esp_timer_get_time() / usPerSec;
}

static void onTimeUpdated([[maybe_unused]] timeval *const tm) {
  if (bootTimestamp == 0) {
    bootTimestamp = time(nullptr);
    state->set(AppState::STATE_TIME_VALID);
  }
  ESP_LOGI(logTag, "sntp time update finished");
}

static void taskSntpUpdate([[maybe_unused]] void *const arg) {
  state->wait(AppState::STATE_NET_CONNECTED);

  sntp_setoperatingmode(SNTP_OPMODE_POLL);
  sntp_setservername(0, "pool.ntp.org");
  sntp_set_time_sync_notification_cb(onTimeUpdated);
  sntp_init();

  ESP_LOGI(logTag, "sntp time update started");

  vTaskDelete(nullptr);
}

void startSntp() {
  xTaskCreate(taskSntpUpdate, "sntp_update", KiB(2), nullptr, 1, nullptr);
}