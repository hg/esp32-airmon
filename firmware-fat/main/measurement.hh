#pragma once

#include "pm.hh"
#include "pms.hh"
#include "utils.hh"
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

  void fixTime();

  template <typename R> void set(const Pm<R> &rhs) { pm = rhs; }

  [[nodiscard]] const char *getType() const;

  bool formatMsg(char *buf, size_t size) const;
};
