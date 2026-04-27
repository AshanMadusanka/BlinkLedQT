#ifndef MCP23017_RPI_H
#define MCP23017_RPI_H

#include <stdbool.h>
#include <stdint.h>
#include <pthread.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MCP23017_RPI_DEFAULT_I2C_DEV "/dev/i2c-1"
#define MCP23017_RPI_DEFAULT_ADDR    0x20

typedef enum
{
    MCP23017_RPI_PIN_A0 = 0,
    MCP23017_RPI_PIN_A1,
    MCP23017_RPI_PIN_A2,
    MCP23017_RPI_PIN_A3,
    MCP23017_RPI_PIN_A4,
    MCP23017_RPI_PIN_A5,
    MCP23017_RPI_PIN_A6,
    MCP23017_RPI_PIN_A7,
    MCP23017_RPI_PIN_B0,
    MCP23017_RPI_PIN_B1,
    MCP23017_RPI_PIN_B2,
    MCP23017_RPI_PIN_B3,
    MCP23017_RPI_PIN_B4,
    MCP23017_RPI_PIN_B5,
    MCP23017_RPI_PIN_B6,
    MCP23017_RPI_PIN_B7
} mcp23017_rpi_pin_t;

typedef enum
{
    MCP23017_RPI_PIN_DIR_OUTPUT = 0,
    MCP23017_RPI_PIN_DIR_INPUT = 1
} mcp23017_rpi_pin_dir_t;

typedef enum
{
    MCP23017_RPI_PIN_LEVEL_LOW = 0,
    MCP23017_RPI_PIN_LEVEL_HIGH = 1
} mcp23017_rpi_pin_level_t;

typedef enum
{
    MCP23017_RPI_PORT_A = 0,
    MCP23017_RPI_PORT_B = 1
} mcp23017_rpi_port_t;

typedef struct
{
    int fd;
    uint8_t i2c_addr;
    pthread_mutex_t mutex;
    bool is_open;
    bool mutex_initialized;
} mcp23017_rpi_t;

/* All functions return 0 on success and a negative errno-style value on error. */

/* Open the Linux I2C device, select the MCP23017 slave address, and initialize safe defaults. */
int mcp23017_rpi_open(mcp23017_rpi_t *dev, const char *i2c_device, uint8_t i2c_addr);

/* Initialize an MCP23017 device; alias for mcp23017_rpi_open() to match init-style APIs. */
int mcp23017_rpi_init(mcp23017_rpi_t *dev, const char *i2c_device, uint8_t i2c_addr);

/* Close the Linux I2C file descriptor and release driver resources. */
void mcp23017_rpi_close(mcp23017_rpi_t *dev);

/* Write one byte to an MCP23017 register. */
int mcp23017_rpi_write_register(mcp23017_rpi_t *dev, uint8_t reg, uint8_t value);

/* Read one byte from an MCP23017 register. */
int mcp23017_rpi_read_register(mcp23017_rpi_t *dev, uint8_t reg, uint8_t *value_out);

/* Set a single pin direction: input sets IODIR bit to 1, output clears it to 0. */
int mcp23017_rpi_set_pin_dir(mcp23017_rpi_t *dev,
                             mcp23017_rpi_pin_t pin,
                             mcp23017_rpi_pin_dir_t dir);

/* Enable or disable the MCP23017 internal pull-up resistor for one pin. */
int mcp23017_rpi_enable_pullup(mcp23017_rpi_t *dev,
                               mcp23017_rpi_pin_t pin,
                               bool enable);

/* Drive one output pin high or low by updating the matching OLAT register bit. */
int mcp23017_rpi_write_pin(mcp23017_rpi_t *dev,
                           mcp23017_rpi_pin_t pin,
                           mcp23017_rpi_pin_level_t level);

/* Read one pin level from the matching GPIO register. */
int mcp23017_rpi_read_pin(mcp23017_rpi_t *dev,
                          mcp23017_rpi_pin_t pin,
                          mcp23017_rpi_pin_level_t *level_out);

/* Set all 8 direction bits for GPIOA or GPIOB; 1 means input and 0 means output. */
int mcp23017_rpi_set_port_dir(mcp23017_rpi_t *dev,
                              mcp23017_rpi_port_t port,
                              uint8_t dir_mask);

/* Set all 8 pull-up bits for GPIOA or GPIOB; 1 enables pull-up. */
int mcp23017_rpi_set_port_pullup(mcp23017_rpi_t *dev,
                                 mcp23017_rpi_port_t port,
                                 uint8_t pullup_mask);

/* Write all 8 output latch bits for GPIOA or GPIOB. */
int mcp23017_rpi_write_port(mcp23017_rpi_t *dev,
                            mcp23017_rpi_port_t port,
                            uint8_t value);

/* Read all 8 GPIO input/output levels from GPIOA or GPIOB. */
int mcp23017_rpi_read_port(mcp23017_rpi_t *dev,
                           mcp23017_rpi_port_t port,
                           uint8_t *value_out);

#ifdef __cplusplus
}
#endif

#endif /* MCP23017_RPI_H */
