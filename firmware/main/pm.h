#pragma once

#include "stddef.h"
#include "stdint.h"

#define PM_STRUCT(T)                                                           \
  struct {                                                                     \
    struct {                                                                   \
      T pm1_ug;                                                                \
      T pm2_ug;                                                                \
      T pm10_ug;                                                               \
    } std;                                                                     \
    struct {                                                                   \
      T pm1_ug;                                                                \
      T pm2_ug;                                                                \
      T pm10_ug;                                                               \
    } atm;                                                                     \
    struct {                                                                   \
      T pm03_count;                                                            \
      T pm05_count;                                                            \
      T pm1_count;                                                             \
      T pm2_count;                                                             \
      T pm5_count;                                                             \
      T pm10_count;                                                            \
    } cnt;                                                                     \
  } __attribute__((packed))

typedef PM_STRUCT(uint16_t) PM16;
typedef PM_STRUCT(uint32_t) PM32;

static const size_t PM_FIELDS = sizeof(PM32) / sizeof(uint32_t);
