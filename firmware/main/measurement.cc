#include "measurement.hh"
#include "common.hh"
#include "settings.hh"
#include "time.hh"
#include <esp_log.h>

// fixes time if it was assigned before SNTP data became available
void Measurement::fixTime() {
  if (!isValidTimestamp(time)) {
    time += bootTimestamp;
  }
}

void Measurement::set(const pms::Response &res) {
  pm.atm.pm1Mcg = res.pm1McgAtm;
  pm.atm.pm2Mcg = res.pm2McgAtm;
  pm.atm.pm10Mcg = res.pm10McgAtm;

  pm.std.pm1Mcg = res.pm1McgStd;
  pm.std.pm2Mcg = res.pm2McgStd;
  pm.std.pm10Mcg = res.pm10McgStd;

  pm.cnt.pm03Count = res.pm03Count;
  pm.cnt.pm05Count = res.pm05Count;
  pm.cnt.pm1Count = res.pm1Count;
  pm.cnt.pm2Count = res.pm2Count;
  pm.cnt.pm5Count = res.pm5Count;
  pm.cnt.pm10Count = res.pm10Count;
}

void Measurement::set(const pms::ResponseSum &sum) {
  pm.atm.pm1Mcg = sum.atm.pm1Mcg / sum.count;
  pm.atm.pm2Mcg = sum.atm.pm2Mcg / sum.count;
  pm.atm.pm10Mcg = sum.atm.pm10Mcg / sum.count;

  pm.std.pm1Mcg = sum.std.pm1Mcg / sum.count;
  pm.std.pm2Mcg = sum.std.pm2Mcg / sum.count;
  pm.std.pm10Mcg = sum.std.pm10Mcg / sum.count;

  pm.cnt.pm03Count = sum.cnt.pm03Count / sum.count;
  pm.cnt.pm05Count = sum.cnt.pm05Count / sum.count;
  pm.cnt.pm1Count = sum.cnt.pm1Count / sum.count;
  pm.cnt.pm2Count = sum.cnt.pm2Count / sum.count;
  pm.cnt.pm5Count = sum.cnt.pm5Count / sum.count;
  pm.cnt.pm10Count = sum.cnt.pm10Count / sum.count;
}

const char *Measurement::getType() const {
  switch (type) {
  case MeasurementType::Temperature:
    return "meas/temp";

  case MeasurementType::Particulates:
    return "meas/part";

  default:
    configASSERT(false);
    return nullptr;
  }
}

bool Measurement::formatMsg(char *const msg, const size_t len) const {
  switch (type) {
  case MeasurementType::Temperature: {
    constexpr auto tpl = R"({"dev":"%s","time":%ld,"sens":"%s","temp":%f})";
    snprintf(msg, len, tpl, appSettings.devName, time, sensor, temp);
    return true;
  }

  case MeasurementType::Particulates: {
    constexpr auto tpl =
        R"({"dev":"%s","time":%ld,"sens":"%s","std":{"pm1":%u,"pm2.5":%u,"pm10":%u},"atm":{"pm1":%u,"pm2.5":%u,"pm10":%u},"cnt":{"pm0.3":%u,"pm0.5":%u,"pm1":%u,"pm2.5":%u,"pm5":%u,"pm10":%u}})";
    snprintf(msg, len, tpl, appSettings.devName, time, sensor, pm.std.pm1Mcg,
             pm.std.pm2Mcg, pm.std.pm10Mcg, pm.atm.pm1Mcg, pm.atm.pm2Mcg,
             pm.atm.pm10Mcg, pm.cnt.pm03Count, pm.cnt.pm05Count,
             pm.cnt.pm1Count, pm.cnt.pm2Count, pm.cnt.pm5Count,
             pm.cnt.pm10Count);
    return true;
  }

  default:
    ESP_LOGE(logTag, "invalid message type %d", static_cast<int>(type));
    return false;
  }
}
