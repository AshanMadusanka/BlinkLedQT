#include "mcp23017_rpi.h"

#include <errno.h>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <stddef.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

/* MCP23017 register map, BANK=0. */
#define MCP23017_REG_IODIRA 0x00
#define MCP23017_REG_IODIRB 0x01
#define MCP23017_REG_GPPUA  0x0C
#define MCP23017_REG_GPPUB  0x0D
#define MCP23017_REG_GPIOA  0x12
#define MCP23017_REG_GPIOB  0x13
#define MCP23017_REG_OLATA  0x14
#define MCP23017_REG_OLATB  0x15

/* Convert errno into the driver's negative int return convention. */
static int errno_to_negative(void)
{
    return (errno > 0) ? -errno : -1;
}

/* Check that the device descriptor points to an open Linux I2C device. */
static int validate_device(const mcp23017_rpi_t *dev)
{
    if (dev == NULL || !dev->is_open || dev->fd < 0) {
        return -EINVAL;
    }

    return 0;
}

/* Check that a port enum value selects GPIOA or GPIOB. */
static int validate_port(mcp23017_rpi_port_t port)
{
    return (port == MCP23017_RPI_PORT_A || port == MCP23017_RPI_PORT_B) ? 0 : -EINVAL;
}

/* Check that a pin enum value is one of the 16 MCP23017 pins. */
static int validate_pin(mcp23017_rpi_pin_t pin)
{
    return (pin >= MCP23017_RPI_PIN_A0 && pin <= MCP23017_RPI_PIN_B7) ? 0 : -EINVAL;
}

/* Return the IODIR register address for GPIOA or GPIOB. */
static uint8_t reg_iodir(mcp23017_rpi_port_t port)
{
    return (port == MCP23017_RPI_PORT_A) ? MCP23017_REG_IODIRA : MCP23017_REG_IODIRB;
}

/* Return the GPPU register address for GPIOA or GPIOB. */
static uint8_t reg_gppu(mcp23017_rpi_port_t port)
{
    return (port == MCP23017_RPI_PORT_A) ? MCP23017_REG_GPPUA : MCP23017_REG_GPPUB;
}

/* Return the GPIO register address for GPIOA or GPIOB. */
static uint8_t reg_gpio(mcp23017_rpi_port_t port)
{
    return (port == MCP23017_RPI_PORT_A) ? MCP23017_REG_GPIOA : MCP23017_REG_GPIOB;
}

/* Return the OLAT register address for GPIOA or GPIOB. */
static uint8_t reg_olat(mcp23017_rpi_port_t port)
{
    return (port == MCP23017_RPI_PORT_A) ? MCP23017_REG_OLATA : MCP23017_REG_OLATB;
}

/* Convert a linear pin enum into the corresponding MCP23017 port and bit index. */
static void pin_to_port_bit(mcp23017_rpi_pin_t pin,
                            mcp23017_rpi_port_t *port_out,
                            uint8_t *bit_out)
{
    if (pin <= MCP23017_RPI_PIN_A7) {
        *port_out = MCP23017_RPI_PORT_A;
        *bit_out = (uint8_t)pin;
    } else {
        *port_out = MCP23017_RPI_PORT_B;
        *bit_out = (uint8_t)(pin - MCP23017_RPI_PIN_B0);
    }
}

/* Lock the device mutex after confirming the descriptor is currently open. */
static int lock_device(mcp23017_rpi_t *dev)
{
    int err = validate_device(dev);

    if (err < 0) {
        return err;
    }

    err = pthread_mutex_lock(&dev->mutex);
    return (err == 0) ? 0 : -err;
}

/* Unlock the device mutex and convert pthread errors to negative return values. */
static int unlock_device(mcp23017_rpi_t *dev)
{
    int err = pthread_mutex_unlock(&dev->mutex);

    return (err == 0) ? 0 : -err;
}

/* Write a register while the caller already holds the device mutex. */
static int write_register_unlocked(mcp23017_rpi_t *dev, uint8_t reg, uint8_t value)
{
    uint8_t buffer[2] = {reg, value};
    ssize_t written = write(dev->fd, buffer, sizeof(buffer));

    if (written != (ssize_t)sizeof(buffer)) {
        return (written < 0) ? errno_to_negative() : -EIO;
    }

    return 0;
}

/* Read a register while the caller already holds the device mutex. */
static int read_register_unlocked(mcp23017_rpi_t *dev, uint8_t reg, uint8_t *value_out)
{
    ssize_t written;
    ssize_t bytes_read;

    if (value_out == NULL) {
        return -EINVAL;
    }

    written = write(dev->fd, &reg, sizeof(reg));
    if (written != (ssize_t)sizeof(reg)) {
        return (written < 0) ? errno_to_negative() : -EIO;
    }

    bytes_read = read(dev->fd, value_out, sizeof(*value_out));
    if (bytes_read != (ssize_t)sizeof(*value_out)) {
        return (bytes_read < 0) ? errno_to_negative() : -EIO;
    }

    return 0;
}

/* Read, modify, and write one register bit while holding the mutex across the full operation. */
static int update_register_bit(mcp23017_rpi_t *dev, uint8_t reg, uint8_t bit, bool set)
{
    uint8_t value;
    int err;

    err = lock_device(dev);
    if (err < 0) {
        return err;
    }

    err = read_register_unlocked(dev, reg, &value);
    if (err == 0) {
        if (set) {
            value |= (uint8_t)(1u << bit);
        } else {
            value &= (uint8_t)~(1u << bit);
        }

        err = write_register_unlocked(dev, reg, value);
    }

    (void)unlock_device(dev);
    return err;
}

/* Open the Linux I2C device, select the MCP23017 slave address, and initialize safe defaults. */
int mcp23017_rpi_open(mcp23017_rpi_t *dev, const char *i2c_device, uint8_t i2c_addr)
{
    int fd;
    int err;

    if (dev == NULL || i2c_device == NULL) {
        return -EINVAL;
    }

    memset(dev, 0, sizeof(*dev));
    dev->fd = -1;
    dev->i2c_addr = i2c_addr;

    err = pthread_mutex_init(&dev->mutex, NULL);
    if (err != 0) {
        return -err;
    }
    dev->mutex_initialized = true;

    fd = open(i2c_device, O_RDWR);
    if (fd < 0) {
        err = errno_to_negative();
        mcp23017_rpi_close(dev);
        return err;
    }

    if (ioctl(fd, I2C_SLAVE, i2c_addr) < 0) {
        err = errno_to_negative();
        (void)close(fd);
        mcp23017_rpi_close(dev);
        return err;
    }

    dev->fd = fd;
    dev->is_open = true;

    err = mcp23017_rpi_write_register(dev, MCP23017_REG_IODIRA, 0xFF);
    if (err == 0) {
        err = mcp23017_rpi_write_register(dev, MCP23017_REG_IODIRB, 0xFF);
    }
    if (err == 0) {
        err = mcp23017_rpi_write_register(dev, MCP23017_REG_GPPUA, 0x00);
    }
    if (err == 0) {
        err = mcp23017_rpi_write_register(dev, MCP23017_REG_GPPUB, 0x00);
    }
    if (err < 0) {
        mcp23017_rpi_close(dev);
        return err;
    }

    return 0;
}

/* Initialize an MCP23017 device; alias for mcp23017_rpi_open() to match init-style APIs. */
int mcp23017_rpi_init(mcp23017_rpi_t *dev, const char *i2c_device, uint8_t i2c_addr)
{
    return mcp23017_rpi_open(dev, i2c_device, i2c_addr);
}

/* Close the Linux I2C file descriptor and release driver resources. */
void mcp23017_rpi_close(mcp23017_rpi_t *dev)
{
    if (dev == NULL) {
        return;
    }

    if (dev->is_open && dev->fd >= 0) {
        (void)close(dev->fd);
    }

    if (dev->mutex_initialized) {
        (void)pthread_mutex_destroy(&dev->mutex);
    }

    dev->fd = -1;
    dev->i2c_addr = 0;
    dev->is_open = false;
    dev->mutex_initialized = false;
}

/* Write one byte to an MCP23017 register. */
int mcp23017_rpi_write_register(mcp23017_rpi_t *dev, uint8_t reg, uint8_t value)
{
    int err = lock_device(dev);

    if (err < 0) {
        return err;
    }

    err = write_register_unlocked(dev, reg, value);
    (void)unlock_device(dev);
    return err;
}

/* Read one byte from an MCP23017 register. */
int mcp23017_rpi_read_register(mcp23017_rpi_t *dev, uint8_t reg, uint8_t *value_out)
{
    int err = lock_device(dev);

    if (err < 0) {
        return err;
    }

    err = read_register_unlocked(dev, reg, value_out);
    (void)unlock_device(dev);
    return err;
}

/* Set a single pin direction: input sets IODIR bit to 1, output clears it to 0. */
int mcp23017_rpi_set_pin_dir(mcp23017_rpi_t *dev,
                             mcp23017_rpi_pin_t pin,
                             mcp23017_rpi_pin_dir_t dir)
{
    mcp23017_rpi_port_t port;
    uint8_t bit;
    int err;

    err = validate_pin(pin);
    if (err < 0) {
        return err;
    }
    if (dir != MCP23017_RPI_PIN_DIR_OUTPUT && dir != MCP23017_RPI_PIN_DIR_INPUT) {
        return -EINVAL;
    }

    pin_to_port_bit(pin, &port, &bit);
    return update_register_bit(dev, reg_iodir(port), bit, dir == MCP23017_RPI_PIN_DIR_INPUT);
}

/* Enable or disable the MCP23017 internal pull-up resistor for one pin. */
int mcp23017_rpi_enable_pullup(mcp23017_rpi_t *dev,
                               mcp23017_rpi_pin_t pin,
                               bool enable)
{
    mcp23017_rpi_port_t port;
    uint8_t bit;
    int err;

    err = validate_pin(pin);
    if (err < 0) {
        return err;
    }

    pin_to_port_bit(pin, &port, &bit);
    return update_register_bit(dev, reg_gppu(port), bit, enable);
}

/* Drive one output pin high or low by updating the matching OLAT register bit. */
int mcp23017_rpi_write_pin(mcp23017_rpi_t *dev,
                           mcp23017_rpi_pin_t pin,
                           mcp23017_rpi_pin_level_t level)
{
    mcp23017_rpi_port_t port;
    uint8_t bit;
    uint8_t value;
    int err;

    err = validate_pin(pin);
    if (err < 0) {
        return err;
    }
    if (level != MCP23017_RPI_PIN_LEVEL_LOW && level != MCP23017_RPI_PIN_LEVEL_HIGH) {
        return -EINVAL;
    }

    pin_to_port_bit(pin, &port, &bit);

    err = lock_device(dev);
    if (err < 0) {
        return err;
    }

    err = read_register_unlocked(dev, reg_olat(port), &value);
    if (err == 0) {
        if (level == MCP23017_RPI_PIN_LEVEL_HIGH) {
            value |= (uint8_t)(1u << bit);
        } else {
            value &= (uint8_t)~(1u << bit);
        }

        err = write_register_unlocked(dev, reg_olat(port), value);
    }

    (void)unlock_device(dev);
    return err;
}

/* Read one pin level from the matching GPIO register. */
int mcp23017_rpi_read_pin(mcp23017_rpi_t *dev,
                          mcp23017_rpi_pin_t pin,
                          mcp23017_rpi_pin_level_t *level_out)
{
    mcp23017_rpi_port_t port;
    uint8_t bit;
    uint8_t value;
    int err;

    if (level_out == NULL) {
        return -EINVAL;
    }

    err = validate_pin(pin);
    if (err < 0) {
        return err;
    }

    pin_to_port_bit(pin, &port, &bit);

    err = mcp23017_rpi_read_register(dev, reg_gpio(port), &value);
    if (err < 0) {
        return err;
    }

    *level_out = (value & (uint8_t)(1u << bit)) ? MCP23017_RPI_PIN_LEVEL_HIGH
                                                : MCP23017_RPI_PIN_LEVEL_LOW;
    return 0;
}

/* Set all 8 direction bits for GPIOA or GPIOB; 1 means input and 0 means output. */
int mcp23017_rpi_set_port_dir(mcp23017_rpi_t *dev,
                              mcp23017_rpi_port_t port,
                              uint8_t dir_mask)
{
    int err = validate_port(port);

    if (err < 0) {
        return err;
    }

    return mcp23017_rpi_write_register(dev, reg_iodir(port), dir_mask);
}

/* Set all 8 pull-up bits for GPIOA or GPIOB; 1 enables pull-up. */
int mcp23017_rpi_set_port_pullup(mcp23017_rpi_t *dev,
                                 mcp23017_rpi_port_t port,
                                 uint8_t pullup_mask)
{
    int err = validate_port(port);

    if (err < 0) {
        return err;
    }

    return mcp23017_rpi_write_register(dev, reg_gppu(port), pullup_mask);
}

/* Write all 8 output latch bits for GPIOA or GPIOB. */
int mcp23017_rpi_write_port(mcp23017_rpi_t *dev,
                            mcp23017_rpi_port_t port,
                            uint8_t value)
{
    int err = validate_port(port);

    if (err < 0) {
        return err;
    }

    return mcp23017_rpi_write_register(dev, reg_olat(port), value);
}

/* Read all 8 GPIO input/output levels from GPIOA or GPIOB. */
int mcp23017_rpi_read_port(mcp23017_rpi_t *dev,
                           mcp23017_rpi_port_t port,
                           uint8_t *value_out)
{
    int err;

    if (value_out == NULL) {
        return -EINVAL;
    }

    err = validate_port(port);
    if (err < 0) {
        return err;
    }

    return mcp23017_rpi_read_register(dev, reg_gpio(port), value_out);
}
