#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

typedef struct {
  QueueHandle_t handle;
  size_t size;
} queue;

void queue_init(queue *q, size_t items, size_t size);
void queue_free(queue *q);
bool queue_put(queue *q, void const *item);
void queue_take(queue *q, void *into);
