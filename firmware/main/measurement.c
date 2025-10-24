#include "measurement.h"
#include "chrono.h"
#include "common.h"
#include <esp_log.h>

static const char *TAG = "air/meas";

const char *measure_get_type(const measurement *ms) {
  switch (ms->type) {
  case TEMPERATURE:
    return "meas/temp";
  case PARTICULATE:
    return "meas/part";
  case CO2:
    return "meas/co2";
  default:
    configASSERT(false);
    return NULL;
  }
}

bool measure_format(const measurement *ms, char *buf, const size_t size) {
  switch (ms->type) {
  case TEMPERATURE: {
    snprintf(buf, size, R"({"dev":"%s","time":%lld,"sens":"%s","temp":%f})",
             CONFIG_DEV_NAME, ms->time, ms->sensor, ms->temp);
    return true;
  }

  case PARTICULATE: {
    snprintf(
        buf, size,
        R"({"dev":"%s","time":%lld,"sens":"%s","std":{"pm1":%u,"pm2.5":%u,"pm10":%u},"atm":{"pm1":%u,"pm2.5":%u,"pm10":%u},"cnt":{"pm0.3":%u,"pm0.5":%u,"pm1":%u,"pm2.5":%u,"pm5":%u,"pm10":%u}})",
        CONFIG_DEV_NAME, ms->time, ms->sensor, ms->pm.std.pm1Mcg,
        ms->pm.std.pm2Mcg, ms->pm.std.pm10Mcg, ms->pm.atm.pm1Mcg,
        ms->pm.atm.pm2Mcg, ms->pm.atm.pm10Mcg, ms->pm.cnt.pm03Count,
        ms->pm.cnt.pm05Count, ms->pm.cnt.pm1Count, ms->pm.cnt.pm2Count,
        ms->pm.cnt.pm5Count, ms->pm.cnt.pm10Count);
    return true;
  }

  case CO2: {
    snprintf(buf, size, R"({"dev":"%s","time":%lld,"sens":"%s","co2":%d})",
             CONFIG_DEV_NAME, ms->time, ms->sensor, ms->co2);
    return true;
  }

  default:
    ESP_LOGE(TAG, "invalid message type %d", (int)(ms->type));
    return false;
  }
}

static void update_time(measurement *ms) {
  ms->time = get_timestamp();
}

void measure_set_pm(measurement *ms, const PM32 *sum, size_t count) {
  configASSERT(count > 0);

  ms->type = PARTICULATE;

  uint16_t *dst = (uint16_t *)&ms->pm;
  uint32_t *src = (uint32_t *)sum;

  for (size_t f = 0; f < PM_FIELDS; ++f) {
    dst[f] = (uint16_t)(src[f] / count);
  }

  update_time(ms);
}

void measure_set_temp(measurement *ms, float temp) {
  ms->type = TEMPERATURE;
  ms->temp = temp;
  update_time(ms);
}

void measure_set_co2(measurement *ms, uint16_t co2) {
  ms->type = CO2;
  ms->co2 = co2;
  update_time(ms);
}
