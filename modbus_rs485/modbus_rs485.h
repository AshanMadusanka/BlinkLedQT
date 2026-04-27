#pragma once

#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_err.h"
#include <stddef.h>
#include <stdint.h>


/**
 * @brief RS485 bus configuration for Modbus communication.
 */
typedef struct {
  uart_port_t uart_port; ///< UART port number (UART_NUM_0/1/2)
  int de_gpio;           ///< Driver Enable GPIO, or GPIO_NUM_NC if not used
  int re_gpio;           ///< Receiver Enable GPIO, or GPIO_NUM_NC if not used
} modbus_rs485_bus_t;

/**
 * @brief Calculate CRC16 (Modbus) for a data buffer.
 *
 * @param buf   Pointer to data buffer
 * @param len   Length of data in bytes
 * @return      CRC16 value (low byte first format)
 */
uint16_t modbus_rs485_crc16(const uint8_t *buf, uint16_t len);

/**
 * @brief Read holding registers from a Modbus slave device.
 *
 * This function sends a Modbus function 0x03 (Read Holding Registers) request
 * and waits for the response. Handles DE/RE pin toggling for RS485 half-duplex.
 *
 * @param bus           Pointer to RS485 bus configuration
 * @param slave_id      Modbus slave address
 * @param start_addr    Starting register address
 * @param num_regs      Number of registers to read
 * @param resp          Buffer to store response (must be at least 5 +
 * 2*num_regs bytes)
 * @param resp_len      Size of response buffer
 * @param timeout_ms    Timeout in milliseconds for response
 *
 * @return
 *      - ESP_OK on success
 *      - ESP_ERR_INVALID_ARG if parameters are invalid
 *      - ESP_ERR_INVALID_SIZE if response buffer too small
 *      - ESP_FAIL on communication error or CRC mismatch
 */
esp_err_t modbus_rs485_read_holding(const modbus_rs485_bus_t *bus,
                                    uint8_t slave_id, uint16_t start_addr,
                                    uint16_t num_regs, uint8_t *resp,
                                    size_t resp_len, int timeout_ms);
