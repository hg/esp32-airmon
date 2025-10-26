#include "uart.h"
#include "esp_check.h"

static const char *TAG = "air/uart";

esp_err_t uart_install(uart_port_t port, gpio_num_t tx, gpio_num_t rx,
                       size_t item_size) {
  const uart_config_t conf = {
      .baud_rate = 9600,
      .data_bits = UART_DATA_8_BITS,
      .parity = UART_PARITY_DISABLE,
      .stop_bits = UART_STOP_BITS_1,
      .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
      .source_clk = UART_SCLK_APB,
  };

  size_t rx_size = item_size * 24;
  assert(rx_size >= UART_HW_FIFO_LEN(port));

  esp_err_t err;

  err = uart_driver_install(port, rx_size, 0, 0, NULL, 0);
  ESP_RETURN_ON_ERROR(err, TAG, "unable to install UART");

  err = uart_param_config(port, &conf);
  ESP_RETURN_ON_ERROR(err, TAG, "unable to setup UART");

  err = uart_set_pin(port, tx, rx, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
  ESP_RETURN_ON_ERROR(err, TAG, "unable to set UART pins");

  return ESP_OK;
}