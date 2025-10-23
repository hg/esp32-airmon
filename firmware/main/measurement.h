#pragma once

#include "pm.h"
#include "pms.h"
#include <ctime>

enum class MeasurementType { TEMPERATURE, PARTICULATES, CO2 };

struct Measurement {
  MeasurementType type;
  time_t time;
  const char *sensor;

  union {
    float temp;
    uint16_t co2;
    Pm<uint16_t> pm;
  };

  template <typename R> void setPM(const Pm<R> &value) {
    updateTime();
    pm = value;
    type = MeasurementType::PARTICULATES;
  }

  void setTemp(float value);
  void setCO2(uint16_t value);

  [[nodiscard]] const char *getType() const;
  bool format(char *buf, size_t size);

private:
  void updateTime();
};