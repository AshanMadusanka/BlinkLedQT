#ifndef XL9535_H
#define XL9535_H

#include "driver/i2c.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Register Definitions
#define XL9535_REGISTER_INPUT_PORT_0 0x00
#define XL9535_REGISTER_INPUT_PORT_1 0x01
#define XL9535_REGISTER_OUTPUT_PORT_0 0x02
#define XL9535_REGISTER_OUTPUT_PORT_1 0x03
#define XL9535_REGISTER_POLARITY_INV_0 0x04
#define XL9535_REGISTER_POLARITY_INV_1 0x05
#define XL9535_REGISTER_CONFIG_0 0x06
#define XL9535_REGISTER_CONFIG_1 0x07
typedef struct xl9535 xl9535_t;

/**
 * @brief XL9535 device descriptor
 *
 * You fill i2c_port + i2c_addr, set mutex = NULL,
 * then call xl9535_init().
 */
struct xl9535 {
  i2c_port_t i2c_port;
  uint8_t i2c_addr;        // 7-bit address
  SemaphoreHandle_t mutex; // optional, created if NULL
};

/** Two 8-bit ports on XL9535 */
typedef enum {
  XL9535_PORT_0 = 0,
  XL9535_PORT_1 = 1,
} xl9535_port_t;

/** Direction for each pin */
typedef enum {
  XL9535_PIN_DIR_OUTPUT = 0,
  XL9535_PIN_DIR_INPUT = 1,
} xl9535_pin_dir_t;

/** Logic level for each pin */
typedef enum {
  XL9535_PIN_LEVEL_LOW = 0,
  XL9535_PIN_LEVEL_HIGH = 1,
} xl9535_pin_level_t;

/**
 * Pin enumeration – this mirrors MCP23017 style where you had MCP_PIN_A0..B7.
 * Here: PORT0_0..PORT0_7, PORT1_0..PORT1_7.
 */
typedef enum {
  XL9535_PIN_P0_0 = 0,
  XL9535_PIN_P0_1,
  XL9535_PIN_P0_2,
  XL9535_PIN_P0_3,
  XL9535_PIN_P0_4,
  XL9535_PIN_P0_5,
  XL9535_PIN_P0_6,
  XL9535_PIN_P0_7,

  XL9535_PIN_P1_0,
  XL9535_PIN_P1_1,
  XL9535_PIN_P1_2,
  XL9535_PIN_P1_3,
  XL9535_PIN_P1_4,
  XL9535_PIN_P1_5,
  XL9535_PIN_P1_6,
  XL9535_PIN_P1_7,
} xl9535_pin_t;

/**
 * @brief Initialize the XL9535 device
 *
 * - Creates a mutex if dev->mutex == NULL
 * - Optionally checks I2C communication
 */
esp_err_t xl9535_init(xl9535_t *dev);

/**
 * @brief Configure pin direction (input / output)
 */
esp_err_t xl9535_set_pin_dir(xl9535_t *dev, xl9535_pin_t pin,
                             xl9535_pin_dir_t dir);

/**
 * @brief Read a single pin level
 */
esp_err_t xl9535_read_pin(xl9535_t *dev, xl9535_pin_t pin,
                          xl9535_pin_level_t *level);

/**
 * @brief Write a single pin level
 *
 * Preserves other bits on the same port.
 */
esp_err_t xl9535_write_pin(xl9535_t *dev, xl9535_pin_t pin,
                           xl9535_pin_level_t level);

/**
 * @brief Read an entire 8-bit port (PORT_0 or PORT_1)
 */
esp_err_t xl9535_read_port(xl9535_t *dev, xl9535_port_t port, uint8_t *value);

/**
 * @brief Write an entire 8-bit port (PORT_0 or PORT_1)
 */
esp_err_t xl9535_write_port(xl9535_t *dev, xl9535_port_t port, uint8_t value);

/**
 * @brief Modify selected bits of a port using mask & value
 *
 * Same concept as MCP driver:
 *   new_reg = (old_reg & ~mask) | (val & mask)
 */
esp_err_t xl9535_write_port_masked(xl9535_t *dev, xl9535_port_t port,
                                   uint8_t mask, uint8_t val);

/**
 * @brief Set polarity inversion for a single pin (for input mode)
 */
esp_err_t xl9535_set_pin_polarity(xl9535_t *dev, xl9535_pin_t pin, bool invert);

/**
 * @brief Low-level raw register access (mainly for debug/special cases)
 */
esp_err_t xl9535_write_register(xl9535_t *dev, uint8_t reg_addr, uint8_t value);
esp_err_t xl9535_read_register(xl9535_t *dev, uint8_t reg_addr, uint8_t *value);

#ifdef __cplusplus
}
#endif

#endif /* XL9535_H */
