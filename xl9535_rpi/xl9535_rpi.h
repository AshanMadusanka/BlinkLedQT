#ifndef XL9535_RPI_H
#define XL9535_RPI_H

#include <stdbool.h>
#include <stdint.h>
#include <pthread.h>

#ifdef __cplusplus
extern "C" {
#endif

#define XL9535_REGISTER_INPUT_PORT_0   0x00
#define XL9535_REGISTER_INPUT_PORT_1   0x01
#define XL9535_REGISTER_OUTPUT_PORT_0  0x02
#define XL9535_REGISTER_OUTPUT_PORT_1  0x03
#define XL9535_REGISTER_POLARITY_INV_0 0x04
#define XL9535_REGISTER_POLARITY_INV_1 0x05
#define XL9535_REGISTER_CONFIG_0       0x06
#define XL9535_REGISTER_CONFIG_1       0x07

#define XL9535_RPI_OK              0
#define XL9535_RPI_ERR_INVALID_ARG -1
#define XL9535_RPI_ERR_OPEN        -2
#define XL9535_RPI_ERR_IOCTL       -3
#define XL9535_RPI_ERR_READ        -4
#define XL9535_RPI_ERR_WRITE       -5
#define XL9535_RPI_ERR_MUTEX       -6

typedef struct {
    int fd;
    uint8_t i2c_addr;
    pthread_mutex_t mutex;
    int mutex_initialized;
} xl9535_rpi_t;

typedef enum {
    XL9535_RPI_PORT_0 = 0,
    XL9535_RPI_PORT_1 = 1,
} xl9535_rpi_port_t;

typedef enum {
    XL9535_RPI_PIN_DIR_OUTPUT = 0,
    XL9535_RPI_PIN_DIR_INPUT = 1,
} xl9535_rpi_pin_dir_t;

typedef enum {
    XL9535_RPI_PIN_LEVEL_LOW = 0,
    XL9535_RPI_PIN_LEVEL_HIGH = 1,
} xl9535_rpi_pin_level_t;

typedef enum {
    XL9535_RPI_PIN_P0_0 = 0,
    XL9535_RPI_PIN_P0_1,
    XL9535_RPI_PIN_P0_2,
    XL9535_RPI_PIN_P0_3,
    XL9535_RPI_PIN_P0_4,
    XL9535_RPI_PIN_P0_5,
    XL9535_RPI_PIN_P0_6,
    XL9535_RPI_PIN_P0_7,

    XL9535_RPI_PIN_P1_0,
    XL9535_RPI_PIN_P1_1,
    XL9535_RPI_PIN_P1_2,
    XL9535_RPI_PIN_P1_3,
    XL9535_RPI_PIN_P1_4,
    XL9535_RPI_PIN_P1_5,
    XL9535_RPI_PIN_P1_6,
    XL9535_RPI_PIN_P1_7,
} xl9535_rpi_pin_t;

/* Open a Linux I2C device, select the XL9535 address, initialize locking, and test communication. */
int xl9535_rpi_open(xl9535_rpi_t *dev,
                    const char *i2c_device,
                    uint8_t i2c_addr);

/* Close the Linux I2C file descriptor and destroy the mutex if initialized. */
void xl9535_rpi_close(xl9535_rpi_t *dev);

/* Write one byte to an XL9535 register. */
int xl9535_rpi_write_register(xl9535_rpi_t *dev,
                              uint8_t reg_addr,
                              uint8_t value);

/* Read one byte from an XL9535 register. */
int xl9535_rpi_read_register(xl9535_rpi_t *dev,
                             uint8_t reg_addr,
                             uint8_t *value);

/* Configure one pin direction; config bit 0 means output and 1 means input. */
int xl9535_rpi_set_pin_dir(xl9535_rpi_t *dev,
                           xl9535_rpi_pin_t pin,
                           xl9535_rpi_pin_dir_t dir);

/* Read one pin level from the selected input-port register. */
int xl9535_rpi_read_pin(xl9535_rpi_t *dev,
                        xl9535_rpi_pin_t pin,
                        xl9535_rpi_pin_level_t *level);

/* Write one output pin while preserving all other pins on that port. */
int xl9535_rpi_write_pin(xl9535_rpi_t *dev,
                         xl9535_rpi_pin_t pin,
                         xl9535_rpi_pin_level_t level);

/* Read a full 8-bit input port. */
int xl9535_rpi_read_port(xl9535_rpi_t *dev,
                         xl9535_rpi_port_t port,
                         uint8_t *value);

/* Write a full 8-bit output port. */
int xl9535_rpi_write_port(xl9535_rpi_t *dev,
                          xl9535_rpi_port_t port,
                          uint8_t value);

/* Write selected output-port bits using new_reg = (old_reg & ~mask) | (val & mask). */
int xl9535_rpi_write_port_masked(xl9535_rpi_t *dev,
                                 xl9535_rpi_port_t port,
                                 uint8_t mask,
                                 uint8_t val);

/* Enable or disable input polarity inversion for one pin. */
int xl9535_rpi_set_pin_polarity(xl9535_rpi_t *dev,
                                xl9535_rpi_pin_t pin,
                                bool invert);

#ifdef __cplusplus
}
#endif

#endif /* XL9535_RPI_H */
