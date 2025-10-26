#pragma once

#include "esp_check.h"
#include "esp_log.h"

#define ITEMS(array) (sizeof(array) / sizeof(array[0]))

#define BAIL_ON_ERROR(err, fmt, ...)                                           \
  ESP_RETURN_VOID_ON_ERROR(err, TAG, fmt, ##__VA_ARGS__)
