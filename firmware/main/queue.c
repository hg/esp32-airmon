#include "queue.h"
#include "utils.h"

void queue_init(queue *q, size_t items, size_t size) {
  q->handle = xQueueCreate(items, size);
  q->size = size;
}

void queue_free(queue *q) {
  vQueueDelete(q->handle);
  q->handle = NULL;
}

bool queue_put(queue *q, const void *item) {
  return xQueueSendToBack(q->handle, item, seconds(1)) != errQUEUE_FULL;
}

void queue_take(queue *q, void *into) {
  xQueueReceive(q->handle, into, portMAX_DELAY);
}
