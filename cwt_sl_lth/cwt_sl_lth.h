#pragma once

#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_err.h"
#include <stdbool.h>

// Forward-declared opaque handle
typedef struct cwt_sl_lth_dev *cwt_sl_lth_handle_t;

// Lux mode: how to interpret the raw lux register
typedef enum {
  // Raw register is direct Lux value (0–65535)
  CWT_SL_LTH_LUX_SIMPLE_1X = 0,

  // Raw register is in 0.01 Lux units (0–200000 Lux sensor)
  // Real lux = raw * 100.0f
  CWT_SL_LTH_LUX_0_01X
} cwt_sl_lth_lux_mode_t;

// Configuration for the CWT-SL-LTH sensor
typedef struct {
  uart_port_t uart_port; // UART_NUM_0/1/2
  int tx_gpio;           // TX pin
  int rx_gpio;           // RX pin
  int de_gpio;           // DE (Driver Enable), or GPIO_NUM_NC
  int re_gpio;           // RE (Receive Enable), or GPIO_NUM_NC
  int baud_rate;         // e.g. 4800
  uint8_t slave_id;      // Modbus ID, usually 1
  cwt_sl_lth_lux_mode_t lux_mode;
  bool init_uart; // true = driver configures UART+driver_install
} cwt_sl_lth_config_t;

/**
 * @brief Create and initialize a sensor handle.
 *
 * If config->init_uart is true, this function will:
 *  - configure the UART parameters
 *  - set UART pins
 *  - install the UART driver
 *
 * If you are sharing the same UART port with other RS485 devices that are
 * already initialized, set init_uart = false.
 */
esp_err_t cwt_sl_lth_create(const cwt_sl_lth_config_t *config,
                            cwt_sl_lth_handle_t *out_handle);

// Destroy the sensor handle and free resources (does NOT uninstall UART)
esp_err_t cwt_sl_lth_destroy(cwt_sl_lth_handle_t handle);

// Change slave ID at runtime
esp_err_t cwt_sl_lth_set_slave_id(cwt_sl_lth_handle_t handle, uint8_t slave_id);

// Read temperature (°C) and humidity (%RH)
esp_err_t cwt_sl_lth_read_temp_hum(cwt_sl_lth_handle_t handle,
                                   float *temperature, float *humidity);

// Read only Lux
esp_err_t cwt_sl_lth_read_lux(cwt_sl_lth_handle_t handle, float *lux);

// Read all three at once: temperature, humidity, lux
esp_err_t cwt_sl_lth_read_all(cwt_sl_lth_handle_t handle, float *temperature,
                              float *humidity, float *lux);

// Optional polling task helper; arg must be (cwt_sl_lth_handle_t)
void cwt_sl_lth_polling_task(void *arg);
