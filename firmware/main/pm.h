#pragma once

#include "stddef.h"
#include "stdint.h"

#define PM_STRUCT(T)                                                           \
  struct {                                                                     \
    struct {                                                                   \
      T pm1Mcg;                                                                \
      T pm2Mcg;                                                                \
      T pm10Mcg;                                                               \
    } std;                                                                     \
    struct {                                                                   \
      T pm1Mcg;                                                                \
      T pm2Mcg;                                                                \
      T pm10Mcg;                                                               \
    } atm;                                                                     \
    struct {                                                                   \
      T pm03Count;                                                             \
      T pm05Count;                                                             \
      T pm1Count;                                                              \
      T pm2Count;                                                              \
      T pm5Count;                                                              \
      T pm10Count;                                                             \
    } cnt;                                                                     \
  } __attribute__((packed))

typedef PM_STRUCT(uint16_t) PM16;
typedef PM_STRUCT(uint32_t) PM32;

static const size_t PM_FIELDS = sizeof(PM32) / sizeof(uint32_t);
