#include "ca.hh"
#include "co2.hh"
#include "common.hh"
#include "dallas.hh"
#include "device_config.hh"
#include "mqtt.hh"
#include "net.hh"
#include "pms.hh"
#include "queue.hh"
#include "state.hh"
#include "time.hh"
#include <driver/periph_ctrl.h>
#include <esp_log.h>

State *appState = nullptr;

static void initLog() {
#ifdef DEBUG
  esp_log_level_set("*", ESP_LOG_DEBUG);
#else
  esp_log_level_set("*", ESP_LOG_INFO);
#endif
}

static void restartPeripheral(const periph_module_t mod) {
  periph_module_disable(mod);
  periph_module_enable(mod);
}

static void initApp() {
  appState = new State;

  // reset peripherals in case of prior crash
  restartPeripheral(PERIPH_RMT_MODULE);
  restartPeripheral(PERIPH_UART0_MODULE);

  initLog();
  initWifi();
  startSntp();
}

extern "C" [[noreturn]] void app_main() {
  initApp();

  // start pollution collectors right away, we can fix time later
  Queue<Measurement> queue{CONFIG_MEASUREMENT_QUEUE_SIZE};

  for (ds::TempSensor &sens : tempSensors) {
    sens.start(queue);
  }

  for (pms::Station &stat : pmsStations) {
    stat.start(queue);
  }

  for (co2::Sensor &sensor : co2Sensors) {
    sensor.start(queue);
  }

  appState->wait(AppState::STATE_NET_CONNECTED);

  mqtt::Client client{CONFIG_MQTT_BROKER_URI, caPemStart, CONFIG_MQTT_USERNAME,
                      CONFIG_MQTT_PASSWORD};

  appState->wait(AppState::STATE_TIME_VALID);

  for (char buf[256];;) {
    client.waitReady();

    Measurement ms = queue.take();
    ms.fixTime();
    ms.formatMsg(buf, sizeof(buf));

    if (!client.send(ms.getType(), buf)) {
      ESP_LOGE(logTag, "measurement send failed");
    }
  }

  configASSERT(false);
}
