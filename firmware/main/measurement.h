#pragma once

#include "pm.h"
#include "pms.h"

typedef enum {
  TEMPERATURE,
  PARTICULATE,
  CO2,
} measure_type;

typedef struct {
  measure_type type;
  time_t time;
  const char *sensor;

  union {
    float temp;
    uint16_t co2;
    PM16 pm;
  };
} measurement;

static_assert(sizeof(measurement) == 48, "qwe");

void measure_set_pm(measurement *ms, const PM32 *sum, size_t count);
void measure_set_temp(measurement *ms, float temp);
void measure_set_co2(measurement *ms, uint16_t co2);

const char *measure_get_type(const measurement *ms);
bool measure_format(const measurement *ms, char *buf, size_t size);
