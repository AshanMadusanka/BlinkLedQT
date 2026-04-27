#pragma once

#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>

// Forward-declared opaque handle
typedef struct sn3002_co2_dev *sn3002_co2_handle_t;

/**
 * @brief Supported baud rates for the SN3002 CO2 sensor.
 */
typedef enum {
  SN3002_CO2_BAUD_RATE_2400 = 0, ///< Baud rate 2400
  SN3002_CO2_BAUD_RATE_4800 = 1, ///< Baud rate 4800
  SN3002_CO2_BAUD_RATE_9600 = 2, ///< Baud rate 9600
} sn3002_co2_baud_rate_t;

/**
 * @brief Configuration for the SN3002 CO2 sensor.
 */
typedef struct {
  uart_port_t uart_port; ///< UART port (UART_NUM_0/1/2)
  int tx_gpio;           ///< TX pin
  int rx_gpio;           ///< RX pin
  int de_gpio;           ///< Driver Enable GPIO, or GPIO_NUM_NC if not used
  int re_gpio;           ///< Receiver Enable GPIO, or GPIO_NUM_NC if not used
  int baud_rate;         ///< e.g. 9600
  uint8_t slave_id;      ///< Modbus slave ID (default 1)
  bool init_uart;        ///< true = driver configures UART+driver_install
} sn3002_co2_config_t;

/**
 * @brief Create and initialize a SN3002 CO2 sensor instance.
 *
 * If config->init_uart is true, this function will:
 *  - configure the UART parameters
 *  - set UART pins
 *  - install the UART driver
 *
 * If you are sharing the same UART port with other RS485 devices that are
 * already initialized, set init_uart = false.
 *
 * @param config      Pointer to sensor configuration
 * @param out_handle  Pointer to store the created handle
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t sn3002_co2_create(const sn3002_co2_config_t *config,
                            sn3002_co2_handle_t *out_handle);

/**
 * @brief Destroy the sensor handle and free resources.
 *
 * Note: This does NOT uninstall the UART driver (because it might be shared).
 *
 * @param handle  Sensor handle to destroy
 * @return ESP_OK on success
 */
esp_err_t sn3002_co2_destroy(sn3002_co2_handle_t handle);

/**
 * @brief Change the Modbus slave ID used by this handle.
 *
 * This only changes how we address the sensor, does not write to
 * the sensor's internal ID register.
 *
 * @param handle    Sensor handle
 * @param slave_id  New slave ID
 * @return ESP_OK on success
 */
esp_err_t sn3002_co2_set_slave_id(sn3002_co2_handle_t handle, uint8_t slave_id);

/**
 * @brief Read CO2 concentration from the sensor.
 *
 * @param handle   Sensor handle
 * @param co2_ppm  Pointer to store CO2 value in ppm
 * @return ESP_OK on success, ESP_FAIL on communication error
 */
esp_err_t sn3002_co2_read(sn3002_co2_handle_t handle, uint32_t *co2_ppm);

/**
 * @brief Calibrate the SN3002 CO2 sensor.
 *
 * Sends a calibration value to the sensor's calibration register.
 *
 * @param handle       Sensor handle
 * @param calib_value  Calibration value to set
 * @return ESP_OK on success, ESP_FAIL on error
 */
esp_err_t sn3002_co2_calibrate(sn3002_co2_handle_t handle, int calib_value);

/**
 * @brief Optional polling task helper.
 *
 * Periodically reads and logs CO2 values. Arg must be (sn3002_co2_handle_t).
 *
 * @param arg  Sensor handle cast to void*
 */
void sn3002_co2_polling_task(void *arg);
