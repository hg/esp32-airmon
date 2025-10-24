#include "measurement.h"
#include "chrono.h"
#include "common.h"
#include <esp_log.h>

static const char *TAG = "air/meas";

const char *measure_get_type(const measurement *ms) {
  switch (ms->type) {
  case MEASURE_TEMP:
    return "meas/temp";
  case MEASURE_PM:
    return "meas/part";
  case MEASURE_CO2:
    return "meas/co2";
  default:
    configASSERT(false);
  }
}

bool measure_format(const measurement *ms, char *buf, const size_t size) {
  switch (ms->type) {
  case MEASURE_TEMP: {
    snprintf(buf, size, R"({"dev":"%s","time":%lld,"sens":"%s","temp":%f})",
             CONFIG_DEV_NAME, ms->time, ms->sensor, ms->temp);
    return true;
  }

  case MEASURE_PM: {
    snprintf(
        buf, size,
        R"({"dev":"%s","time":%lld,"sens":"%s","std":{"pm1":%u,"pm2.5":%u,"pm10":%u},"atm":{"pm1":%u,"pm2.5":%u,"pm10":%u},"cnt":{"pm0.3":%u,"pm0.5":%u,"pm1":%u,"pm2.5":%u,"pm5":%u,"pm10":%u}})",
        CONFIG_DEV_NAME, ms->time, ms->sensor, ms->pm.std.pm1_ug,
        ms->pm.std.pm2_ug, ms->pm.std.pm10_ug, ms->pm.atm.pm1_ug,
        ms->pm.atm.pm2_ug, ms->pm.atm.pm10_ug, ms->pm.cnt.pm03_count,
        ms->pm.cnt.pm05_count, ms->pm.cnt.pm1_count, ms->pm.cnt.pm2_count,
        ms->pm.cnt.pm5_count, ms->pm.cnt.pm10_count);
    return true;
  }

  case MEASURE_CO2: {
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

  ms->type = MEASURE_PM;

  uint16_t *dst = (uint16_t *)&ms->pm;
  uint32_t *src = (uint32_t *)sum;

  for (size_t f = 0; f < PM_FIELDS; ++f) {
    dst[f] = (uint16_t)(src[f] / count);
  }

  update_time(ms);
}

void measure_set_temp(measurement *ms, float temp) {
  ms->type = MEASURE_TEMP;
  ms->temp = temp;
  update_time(ms);
}

void measure_set_co2(measurement *ms, uint16_t co2) {
  ms->type = MEASURE_CO2;
  ms->co2 = co2;
  update_time(ms);
}
