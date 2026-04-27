#include "xl9535_rpi.h"

#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <stddef.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

static int validate_device(const xl9535_rpi_t *dev)
{
    if (dev == NULL || dev->fd < 0 || !dev->mutex_initialized) {
        return XL9535_RPI_ERR_INVALID_ARG;
    }

    return XL9535_RPI_OK;
}

static int validate_port(xl9535_rpi_port_t port)
{
    return (port == XL9535_RPI_PORT_0 || port == XL9535_RPI_PORT_1)
               ? XL9535_RPI_OK
               : XL9535_RPI_ERR_INVALID_ARG;
}

static int validate_pin(xl9535_rpi_pin_t pin)
{
    return (pin >= XL9535_RPI_PIN_P0_0 && pin <= XL9535_RPI_PIN_P1_7)
               ? XL9535_RPI_OK
               : XL9535_RPI_ERR_INVALID_ARG;
}

static int validate_dir(xl9535_rpi_pin_dir_t dir)
{
    return (dir == XL9535_RPI_PIN_DIR_OUTPUT || dir == XL9535_RPI_PIN_DIR_INPUT)
               ? XL9535_RPI_OK
               : XL9535_RPI_ERR_INVALID_ARG;
}

static int validate_level(xl9535_rpi_pin_level_t level)
{
    return (level == XL9535_RPI_PIN_LEVEL_LOW || level == XL9535_RPI_PIN_LEVEL_HIGH)
               ? XL9535_RPI_OK
               : XL9535_RPI_ERR_INVALID_ARG;
}

static int lock_device(xl9535_rpi_t *dev)
{
    int err = validate_device(dev);

    if (err < 0) {
        return err;
    }

    return (pthread_mutex_lock(&dev->mutex) == 0) ? XL9535_RPI_OK
                                                  : XL9535_RPI_ERR_MUTEX;
}

static int unlock_device(xl9535_rpi_t *dev)
{
    return (pthread_mutex_unlock(&dev->mutex) == 0) ? XL9535_RPI_OK
                                                    : XL9535_RPI_ERR_MUTEX;
}

static void pin_to_port_bit(xl9535_rpi_pin_t pin,
                            xl9535_rpi_port_t *port,
                            uint8_t *bit)
{
    if (pin <= XL9535_RPI_PIN_P0_7) {
        *port = XL9535_RPI_PORT_0;
        *bit = (uint8_t)pin;
    } else {
        *port = XL9535_RPI_PORT_1;
        *bit = (uint8_t)(pin - XL9535_RPI_PIN_P1_0);
    }
}

static uint8_t reg_input(xl9535_rpi_port_t port)
{
    return (port == XL9535_RPI_PORT_0) ? XL9535_REGISTER_INPUT_PORT_0
                                       : XL9535_REGISTER_INPUT_PORT_1;
}

static uint8_t reg_output(xl9535_rpi_port_t port)
{
    return (port == XL9535_RPI_PORT_0) ? XL9535_REGISTER_OUTPUT_PORT_0
                                       : XL9535_REGISTER_OUTPUT_PORT_1;
}

static uint8_t reg_polarity(xl9535_rpi_port_t port)
{
    return (port == XL9535_RPI_PORT_0) ? XL9535_REGISTER_POLARITY_INV_0
                                       : XL9535_REGISTER_POLARITY_INV_1;
}

static uint8_t reg_config(xl9535_rpi_port_t port)
{
    return (port == XL9535_RPI_PORT_0) ? XL9535_REGISTER_CONFIG_0
                                       : XL9535_REGISTER_CONFIG_1;
}

static int write_register_unlocked(xl9535_rpi_t *dev,
                                   uint8_t reg_addr,
                                   uint8_t value)
{
    uint8_t data[2] = {reg_addr, value};
    ssize_t written = write(dev->fd, data, sizeof(data));

    if (written != (ssize_t)sizeof(data)) {
        return XL9535_RPI_ERR_WRITE;
    }

    return XL9535_RPI_OK;
}

static int read_register_unlocked(xl9535_rpi_t *dev,
                                  uint8_t reg_addr,
                                  uint8_t *value)
{
    ssize_t written;
    ssize_t bytes_read;

    if (value == NULL) {
        return XL9535_RPI_ERR_INVALID_ARG;
    }

    written = write(dev->fd, &reg_addr, sizeof(reg_addr));
    if (written != (ssize_t)sizeof(reg_addr)) {
        return XL9535_RPI_ERR_WRITE;
    }

    bytes_read = read(dev->fd, value, sizeof(*value));
    if (bytes_read != (ssize_t)sizeof(*value)) {
        return XL9535_RPI_ERR_READ;
    }

    return XL9535_RPI_OK;
}

static int update_register_bit(xl9535_rpi_t *dev,
                               uint8_t reg_addr,
                               uint8_t bit,
                               bool set)
{
    uint8_t value;
    int err;

    err = lock_device(dev);
    if (err < 0) {
        return err;
    }

    err = read_register_unlocked(dev, reg_addr, &value);
    if (err == XL9535_RPI_OK) {
        if (set) {
            value |= (uint8_t)(1u << bit);
        } else {
            value &= (uint8_t)~(1u << bit);
        }
        err = write_register_unlocked(dev, reg_addr, value);
    }

    (void)unlock_device(dev);
    return err;
}

int xl9535_rpi_open(xl9535_rpi_t *dev,
                    const char *i2c_device,
                    uint8_t i2c_addr)
{
    int fd;
    int err;
    uint8_t config0;

    if (dev == NULL || i2c_device == NULL) {
        return XL9535_RPI_ERR_INVALID_ARG;
    }

    memset(dev, 0, sizeof(*dev));
    dev->fd = -1;
    dev->i2c_addr = i2c_addr;

    fd = open(i2c_device, O_RDWR);
    if (fd < 0) {
        return XL9535_RPI_ERR_OPEN;
    }

    if (ioctl(fd, I2C_SLAVE, i2c_addr) < 0) {
        (void)close(fd);
        return XL9535_RPI_ERR_IOCTL;
    }

    err = pthread_mutex_init(&dev->mutex, NULL);
    if (err != 0) {
        (void)close(fd);
        return XL9535_RPI_ERR_MUTEX;
    }

    dev->fd = fd;
    dev->mutex_initialized = 1;

    /*
     * Communication test only. This intentionally does not force any port
     * direction; configure pins explicitly with xl9535_rpi_set_pin_dir().
     */
    err = xl9535_rpi_read_register(dev, XL9535_REGISTER_CONFIG_0, &config0);
    if (err < 0) {
        xl9535_rpi_close(dev);
        return err;
    }

    return XL9535_RPI_OK;
}

void xl9535_rpi_close(xl9535_rpi_t *dev)
{
    if (dev == NULL) {
        return;
    }

    if (dev->fd >= 0) {
        (void)close(dev->fd);
    }

    if (dev->mutex_initialized) {
        (void)pthread_mutex_destroy(&dev->mutex);
    }

    dev->fd = -1;
    dev->i2c_addr = 0;
    dev->mutex_initialized = 0;
}

int xl9535_rpi_write_register(xl9535_rpi_t *dev,
                              uint8_t reg_addr,
                              uint8_t value)
{
    int err = lock_device(dev);

    if (err < 0) {
        return err;
    }

    err = write_register_unlocked(dev, reg_addr, value);
    (void)unlock_device(dev);
    return err;
}

int xl9535_rpi_read_register(xl9535_rpi_t *dev,
                             uint8_t reg_addr,
                             uint8_t *value)
{
    int err;

    if (value == NULL) {
        return XL9535_RPI_ERR_INVALID_ARG;
    }

    err = lock_device(dev);
    if (err < 0) {
        return err;
    }

    err = read_register_unlocked(dev, reg_addr, value);
    (void)unlock_device(dev);
    return err;
}

int xl9535_rpi_set_pin_dir(xl9535_rpi_t *dev,
                           xl9535_rpi_pin_t pin,
                           xl9535_rpi_pin_dir_t dir)
{
    xl9535_rpi_port_t port;
    uint8_t bit;
    int err;

    err = validate_pin(pin);
    if (err < 0) {
        return err;
    }

    err = validate_dir(dir);
    if (err < 0) {
        return err;
    }

    pin_to_port_bit(pin, &port, &bit);
    return update_register_bit(dev,
                               reg_config(port),
                               bit,
                               dir == XL9535_RPI_PIN_DIR_INPUT);
}

int xl9535_rpi_read_pin(xl9535_rpi_t *dev,
                        xl9535_rpi_pin_t pin,
                        xl9535_rpi_pin_level_t *level)
{
    xl9535_rpi_port_t port;
    uint8_t bit;
    uint8_t value;
    int err;

    if (level == NULL) {
        return XL9535_RPI_ERR_INVALID_ARG;
    }

    err = validate_pin(pin);
    if (err < 0) {
        return err;
    }

    pin_to_port_bit(pin, &port, &bit);

    err = xl9535_rpi_read_register(dev, reg_input(port), &value);
    if (err < 0) {
        return err;
    }

    *level = (value & (uint8_t)(1u << bit)) ? XL9535_RPI_PIN_LEVEL_HIGH
                                            : XL9535_RPI_PIN_LEVEL_LOW;
    return XL9535_RPI_OK;
}

int xl9535_rpi_write_pin(xl9535_rpi_t *dev,
                         xl9535_rpi_pin_t pin,
                         xl9535_rpi_pin_level_t level)
{
    xl9535_rpi_port_t port;
    uint8_t bit;
    int err;

    err = validate_pin(pin);
    if (err < 0) {
        return err;
    }

    err = validate_level(level);
    if (err < 0) {
        return err;
    }

    pin_to_port_bit(pin, &port, &bit);
    return update_register_bit(dev,
                               reg_output(port),
                               bit,
                               level == XL9535_RPI_PIN_LEVEL_HIGH);
}

int xl9535_rpi_read_port(xl9535_rpi_t *dev,
                         xl9535_rpi_port_t port,
                         uint8_t *value)
{
    int err;

    if (value == NULL) {
        return XL9535_RPI_ERR_INVALID_ARG;
    }

    err = validate_port(port);
    if (err < 0) {
        return err;
    }

    return xl9535_rpi_read_register(dev, reg_input(port), value);
}

int xl9535_rpi_write_port(xl9535_rpi_t *dev,
                          xl9535_rpi_port_t port,
                          uint8_t value)
{
    int err = validate_port(port);

    if (err < 0) {
        return err;
    }

    return xl9535_rpi_write_register(dev, reg_output(port), value);
}

int xl9535_rpi_write_port_masked(xl9535_rpi_t *dev,
                                 xl9535_rpi_port_t port,
                                 uint8_t mask,
                                 uint8_t val)
{
    uint8_t old_reg;
    uint8_t new_reg;
    int err;

    err = validate_port(port);
    if (err < 0) {
        return err;
    }

    err = lock_device(dev);
    if (err < 0) {
        return err;
    }

    err = read_register_unlocked(dev, reg_output(port), &old_reg);
    if (err == XL9535_RPI_OK) {
        new_reg = (uint8_t)((old_reg & (uint8_t)~mask) | (val & mask));
        err = write_register_unlocked(dev, reg_output(port), new_reg);
    }

    (void)unlock_device(dev);
    return err;
}

int xl9535_rpi_set_pin_polarity(xl9535_rpi_t *dev,
                                xl9535_rpi_pin_t pin,
                                bool invert)
{
    xl9535_rpi_port_t port;
    uint8_t bit;
    int err;

    err = validate_pin(pin);
    if (err < 0) {
        return err;
    }

    pin_to_port_bit(pin, &port, &bit);
    return update_register_bit(dev, reg_polarity(port), bit, invert);
}
