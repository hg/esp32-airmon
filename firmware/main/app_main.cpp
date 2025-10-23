#include "co2.h"
#include "common.h"
#include "dallas.h"
#include "config.h"
#include "mqtt.h"
#include "net.h"
#include "pms.h"
#include "queue.h"
#include "state.h"
#include "chrono.h"
#include <esp_log.h>

State *state = nullptr;

static void initLog() {
#ifdef DEBUG
  esp_log_level_set("*", ESP_LOG_DEBUG);
#else
  esp_log_level_set("*", ESP_LOG_INFO);
#endif
}

static void initApp() {
  state = new State;

  initLog();
  initWifi();
  startSntp();
}

extern "C" [[noreturn]] void app_main() {
  initApp();

  // start collectors right away, we can fix time later
  Queue<Measurement> queue{500};

  for (ds::TempSensor &sens : tempSensors) {
    sens.start(queue);
  }

  for (pms::Station &stat : pmsStations) {
    stat.start(queue);
  }

  for (co2::Sensor &sensor : co2Sensors) {
    sensor.start(queue);
  }

  state->wait(AppState::STATE_NET_CONNECTED);

  mqtt::Client client{CONFIG_MQTT_BROKER_URI, CONFIG_DEV_NAME, CONFIG_MQTT_PSK};

  state->wait(AppState::STATE_TIME_VALID);

  for (char buf[256];;) {
    client.waitReady();

    Measurement ms = queue.take();
    ms.format(buf, sizeof(buf));

    if (!client.send(ms.getType(), buf)) {
      ESP_LOGE(logTag, "measurement send failed");
    }
  }

  configASSERT(false);
}