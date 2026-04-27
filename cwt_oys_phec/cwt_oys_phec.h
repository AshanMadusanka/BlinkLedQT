#pragma once

#include <stdbool.h>
#include "esp_err.h"
#include "driver/uart.h"
#include "driver/gpio.h"

// Opaque device handle
typedef struct cwt_oys_phec_dev *cwt_oys_phec_handle_t;

// Configuration structure for the sensor
typedef struct {
    uart_port_t uart_port;   // UART_NUM_0 / UART_NUM_1 / UART_NUM_2
    int         tx_gpio;     // TX pin
    int         rx_gpio;     // RX pin
    int         de_gpio;     // DE (Driver Enable), or GPIO_NUM_NC
    int         re_gpio;     // RE (Receiver Enable), or GPIO_NUM_NC
    int         baud_rate;   // e.g. 9600
    uint8_t     slave_id;    // Modbus slave ID (default 1)
    bool        init_uart;   // true = driver configures UART+driver_install
} cwt_oys_phec_config_t;

/**
 * @brief Create and initialize a CWT-OYS-PHEC sensor instance.
 *
 * If config->init_uart is true, this function will:
 *  - configure the UART parameters
 *  - set UART pins
 *  - install the UART driver
 *
 * If you are sharing the same UART port with other RS485 devices that are
 * already initialized, set init_uart = false.
 */
esp_err_t cwt_oys_phec_create(const cwt_oys_phec_config_t *config,
                              cwt_oys_phec_handle_t *out_handle);

/**
 * @brief Destroy the sensor handle and free resources.
 *
 * Note: This does NOT uninstall the UART driver (because it might be shared).
 */
esp_err_t cwt_oys_phec_destroy(cwt_oys_phec_handle_t handle);

/**
 * @brief Change the Modbus slave ID used by this handle.
 * (Does not write to the sensor's ID register – just changes how we address it.)
 */
esp_err_t cwt_oys_phec_set_slave_id(cwt_oys_phec_handle_t handle,
                                    uint8_t slave_id);

/**
 * @brief Read pH, EC, temperature all in one Modbus request.
 *
 * @param handle       Sensor handle
 * @param ph           pH value (e.g. 7.05)
 * @param ec_uScm      EC in µS/cm
 * @param temperature  Temperature in °C
 */
esp_err_t cwt_oys_phec_read_all(cwt_oys_phec_handle_t handle,
                                float *ph,
                                float *ec_uScm,
                                float *temperature);

/**
 * @brief Read only pH value.
 */
esp_err_t cwt_oys_phec_read_ph(cwt_oys_phec_handle_t handle,
                               float *ph);

/**
 * @brief Read only EC value (µS/cm).
 */
esp_err_t cwt_oys_phec_read_ec(cwt_oys_phec_handle_t handle,
                               float *ec_uScm);

/**
 * @brief Read only temperature (°C).
 */
esp_err_t cwt_oys_phec_read_temperature(cwt_oys_phec_handle_t handle,
                                        float *temperature);

/**
 * @brief Optional polling task helper; arg must be (cwt_oys_phec_handle_t).
 *
 * This will periodically log the sensor values.
 */
void cwt_oys_phec_polling_task(void *arg);
