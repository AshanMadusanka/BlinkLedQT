
#ifndef MCP23017_H
#define MCP23017_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

// MCP23017 pin enumeration (for nice API)
typedef enum
{
    MCP_PIN_A0 = 0,
    MCP_PIN_A1,
    MCP_PIN_A2,
    MCP_PIN_A3,
    MCP_PIN_A4,
    MCP_PIN_A5,
    MCP_PIN_A6,
    MCP_PIN_A7,
    MCP_PIN_B0,
    MCP_PIN_B1,
    MCP_PIN_B2,
    MCP_PIN_B3,
    MCP_PIN_B4,
    MCP_PIN_B5,
    MCP_PIN_B6,
    MCP_PIN_B7
} mcp23017_pin_t;

typedef enum
{
    MCP_PIN_DIR_OUTPUT = 0,
    MCP_PIN_DIR_INPUT = 1
} mcp23017_pin_dir_t;

typedef enum
{
    MCP_PIN_LEVEL_LOW = 0,
    MCP_PIN_LEVEL_HIGH = 1
} mcp23017_pin_level_t;

typedef enum
{
    MCP_PORT_A = 0,
    MCP_PORT_B = 1
} mcp23017_port_t;

// Device descriptor
typedef struct
{
    i2c_port_t i2c_port;     // I2C_NUM_0, I2C_NUM_1, etc.
    uint8_t i2c_addr;        // 0x20–0x27 typically
    SemaphoreHandle_t mutex; // optional; if NULL, driver creates its own
} mcp23017_t;

// Basic init (assumes I2C bus already configured and driver installed)
esp_err_t mcp23017_init(mcp23017_t *dev);

// Per-pin control
esp_err_t mcp23017_set_pin_dir(mcp23017_t *dev, mcp23017_pin_t pin, mcp23017_pin_dir_t dir);

esp_err_t mcp23017_enable_pullup(mcp23017_t *dev, mcp23017_pin_t pin, bool enable);

esp_err_t mcp23017_write_pin(mcp23017_t *dev, mcp23017_pin_t pin, mcp23017_pin_level_t level);

esp_err_t mcp23017_read_pin(mcp23017_t *dev, mcp23017_pin_t pin, mcp23017_pin_level_t *level_out);

// Whole-port functions (A or B at once)
esp_err_t mcp23017_set_port_dir(mcp23017_t *dev, mcp23017_port_t port, uint8_t dir_mask); // 1=input, 0=output

esp_err_t mcp23017_set_port_pullup(mcp23017_t *dev, mcp23017_port_t port, uint8_t pullup_mask); // 1=enable

esp_err_t mcp23017_read_port(mcp23017_t *dev, mcp23017_port_t port, uint8_t *value_out);

esp_err_t mcp23017_write_port(mcp23017_t *dev, mcp23017_port_t port, uint8_t value);

#endif // MCP23017_H
