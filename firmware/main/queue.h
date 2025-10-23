#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "utils.h"

template <typename T> class Queue {
public:
  explicit Queue(const size_t size) {
    handle = xQueueCreate(size, sizeof(T));
    configASSERT(handle);
  }

  ~Queue() {
    vQueueDelete(handle);
    handle = nullptr;
  }

  Queue(const Queue &) = delete;

  Queue &operator=(const Queue &) = delete;

  bool put(const T &item, TickType_t wait = portMAX_DELAY) {
    return xQueueSendToBack(handle, &item, wait) != errQUEUE_FULL;
  }

  bool putRetrying(const T &item) {
    for (int retry = 0; retry < 5; ++retry) {
      if (retry > 0) {
        T drop;
        takeInto(drop, 0);
      }
      if (put(item, seconds(1))) {
        return true;
      }
    }
    return false;
  }

  T take() {
    T item;
    xQueueReceive(handle, &item, portMAX_DELAY);
    return item;
  }

  bool takeInto(T &item, const TickType_t wait = portMAX_DELAY) {
    return xQueueReceive(handle, &item, wait);
  }

private:
  QueueHandle_t handle;
};
