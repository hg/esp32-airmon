#include "common.h"
#include "net.h"
#include "state.h"
#include "utils.h"

#include <esp_event.h>
#include <esp_tls.h>
#include <esp_wifi.h>

static const char *TAG = "air/res";

static void on_fire(TimerHandle_t tm) {
  multi_heap_info_t info = {};
  heap_caps_get_info(&info, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);

  ESP_LOGI(TAG, "heap: %uK free, %uK alloc, %uK min free",
           toKiB(info.total_free_bytes), toKiB(info.total_allocated_bytes),
           toKiB(info.minimum_free_bytes));
}

void start_resource_log() {
  TimerHandle_t tm = xTimerCreate("heap", seconds(60), true, NULL, on_fire);

  if (xTimerStart(tm, 0) != pdPASS) {
    ESP_LOGE(TAG, "unable to start resource timer");
  }
}
