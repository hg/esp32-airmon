#pragma once

#include "driver/gpio.h"
#include "driver/uart.h"

esp_err_t uart_install(uart_port_t port, gpio_num_t tx, gpio_num_t rx,
                       size_t item_size);