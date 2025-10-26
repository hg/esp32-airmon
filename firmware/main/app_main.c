#include "chrono.h"
#include "co2.h"
#include "common.h"
#include "config.h"
#include "dallas.h"
#include "esp_log.h"
#include "mqtt.h"
#include "net.h"
#include "pms.h"
#include "queue.h"
#include "ram.h"
#include "state.h"
#include "temp.h"

static const char *TAG = "air/main";

static void init_log() {
#ifdef DEBUG
  esp_log_level_set("*", ESP_LOG_DEBUG);
#else
  esp_log_level_set("*", ESP_LOG_INFO);
#endif

  start_resource_log();
}

static void app_init(app_state *state) {
  init_log();
  init_wifi(state);
  sntp_start(state);
}

static void start_sensors(queue *q) {
  for (int i = 0; i < ITEMS(temp_sensors); i++) {
    temp_start(&temp_sensors[i], q);
  }
  for (int i = 0; i < ITEMS(pm_sensors); i++) {
    pms_start(&pm_sensors[i], q);
  }
  for (int i = 0; i < ITEMS(co2_sensors); i++) {
    co2_start(&co2_sensors[i], q);
  }
  temp_start_int();
}

[[noreturn]]
void app_main() {
  app_state state = {};
  state_init(&state);

  app_init(&state);

  // allocate most of the free RAM to the measurement queue to preserve data
  // during prolonged internet outages (roughly 100 KiB)
  queue q = {};
  queue_init(&q, 2000, sizeof(measurement));

  // start collectors right away, we can fix time later
  start_sensors(&q);
  state_wait(&state, APP_ONLINE);

  mqtt_client mq = {};
  mqtt_init(&mq, CONFIG_MQTT_BROKER_URI, CONFIG_DEV_NAME, CONFIG_MQTT_PSK);

  state_wait(&state, APP_TIME_VALID);

  for (char buf[512];;) {
    mqtt_wait(&mq, MQTT_READY);

    measurement ms = {};
    queue_take(&q, &ms);
    ms.time = fix_time(ms.time);

    const char *type = measure_get_type(&ms);
    measure_format(&ms, buf, sizeof(buf));

    if (!mqtt_send(&mq, type, buf)) {
      ESP_LOGE(TAG, "measurement send failed");
    }
  }

  configASSERT(false);
}
