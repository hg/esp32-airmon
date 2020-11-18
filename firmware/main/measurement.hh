#pragma once

#include "pms.hh"
#include "utils.hh"
#include <ctime>

enum class MeasurementType { TEMPERATURE, PARTICULATES };

struct Measurement {
  MeasurementType type;
  time_t time;
  const char *sensor;
  union {
    float temp;
    struct {
      struct {
        uint16_t pm1Mcg;
        uint16_t pm2Mcg;
        uint16_t pm10Mcg;
      } std;
      struct {
        uint16_t pm1Mcg;
        uint16_t pm2Mcg;
        uint16_t pm10Mcg;
      } atm;
      struct {
        uint16_t pm03Count;
        uint16_t pm05Count;
        uint16_t pm1Count;
        uint16_t pm2Count;
        uint16_t pm5Count;
        uint16_t pm10Count;
      } cnt;
    } pm;
  };

  void fixTime();

  void set(const pms::Response &res);

  void set(const pms::ResponseSum &sum);

  [[nodiscard]] const char *getType() const;

  bool formatMsg(char *buf, size_t size) const;
};
