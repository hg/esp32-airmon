#include "measurement.hh"
#include "common.hh"
#include "time.hh"
#include <esp_log.h>

// fixes time if it was assigned before SNTP data became available
void Measurement::fixTime() {
  if (!isValidTimestamp(time)) {
    time += bootTimestamp;
  }
}

const char *Measurement::getType() const {
  switch (type) {
  case MeasurementType::TEMPERATURE:
    return "meas/temp";

  case MeasurementType::PARTICULATES:
    return "meas/part";

  case MeasurementType::CO2:
    return "meas/co2";

  default:
    configASSERT(false);
    return nullptr;
  }
}

bool Measurement::formatMsg(char *const buf, const size_t size) const {
  switch (type) {
  case MeasurementType::TEMPERATURE: {
    constexpr auto tpl = R"({"dev":"%s","time":%ld,"sens":"%s","temp":%f})";
    snprintf(buf, size, tpl, CONFIG_DEV_NAME, time, sensor, temp);
    return true;
  }

  case MeasurementType::PARTICULATES: {
    constexpr auto tpl =
        R"({"dev":"%s","time":%ld,"sens":"%s","std":{"pm1":%u,"pm2.5":%u,"pm10":%u},"atm":{"pm1":%u,"pm2.5":%u,"pm10":%u},"cnt":{"pm0.3":%u,"pm0.5":%u,"pm1":%u,"pm2.5":%u,"pm5":%u,"pm10":%u}})";
    snprintf(buf, size, tpl, CONFIG_DEV_NAME, time, sensor, pm.std.pm1Mcg,
             pm.std.pm2Mcg, pm.std.pm10Mcg, pm.atm.pm1Mcg, pm.atm.pm2Mcg,
             pm.atm.pm10Mcg, pm.cnt.pm03Count, pm.cnt.pm05Count,
             pm.cnt.pm1Count, pm.cnt.pm2Count, pm.cnt.pm5Count,
             pm.cnt.pm10Count);
    return true;
  }

  case MeasurementType::CO2: {
    constexpr auto tpl = R"({"dev":"%s","time":%ld,"sens":"%s","co2":%d})";
    snprintf(buf, size, tpl, CONFIG_DEV_NAME, time, sensor, co2);
    return true;
  }

  default:
    ESP_LOGE(logTag, "invalid message type %d", static_cast<int>(type));
    return false;
  }
}
