#define _DEFAULT_SOURCE

#include "ads1115_rpi.h"

#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

#define ADS1115_RPI_OS_START             0x8000u
#define ADS1115_RPI_MODE_SINGLE_SHOT     0x0100u
#define ADS1115_RPI_COMP_QUE_DISABLE     0x0003u
#define ADS1115_RPI_OS_READY             0x8000u

#define ADS1115_RPI_MUX_SINGLE_ENDED_A0  0x04u
#define ADS1115_RPI_CONVERSION_POLL_US   1000u
#define ADS1115_RPI_CONVERSION_TIMEOUT_US 1000000u

static int ads1115_rpi_validate_device(const ads1115_rpi_t *dev)
{
    if (dev == NULL || dev->fd < 0) {
        return ADS1115_RPI_ERR_INVALID_ARG;
    }

    return ADS1115_RPI_OK;
}

static int ads1115_rpi_validate_channel(ads1115_rpi_channel_t channel)
{
    return (channel >= ADS1115_RPI_CHANNEL_A0 &&
            channel <= ADS1115_RPI_CHANNEL_A3)
               ? ADS1115_RPI_OK
               : ADS1115_RPI_ERR_INVALID_ARG;
}

static int ads1115_rpi_validate_gain(ads1115_rpi_gain_t gain)
{
    return (gain >= ADS1115_RPI_GAIN_6_144V &&
            gain <= ADS1115_RPI_GAIN_0_256V)
               ? ADS1115_RPI_OK
               : ADS1115_RPI_ERR_INVALID_ARG;
}

static int ads1115_rpi_validate_data_rate(ads1115_rpi_data_rate_t data_rate)
{
    return (data_rate >= ADS1115_RPI_DATA_RATE_8SPS &&
            data_rate <= ADS1115_RPI_DATA_RATE_860SPS)
               ? ADS1115_RPI_OK
               : ADS1115_RPI_ERR_INVALID_ARG;
}

static uint16_t ads1115_rpi_build_config(const ads1115_rpi_t *dev,
                                         ads1115_rpi_channel_t channel)
{
    uint16_t mux = (uint16_t)(ADS1115_RPI_MUX_SINGLE_ENDED_A0 +
                              (uint16_t)channel);
    uint16_t pga = (uint16_t)dev->gain;
    uint16_t dr = (uint16_t)dev->data_rate;

    return (uint16_t)(ADS1115_RPI_OS_START |
                      (uint16_t)(mux << 12) |
                      (uint16_t)(pga << 9) |
                      ADS1115_RPI_MODE_SINGLE_SHOT |
                      (uint16_t)(dr << 5) |
                      ADS1115_RPI_COMP_QUE_DISABLE);
}

static int ads1115_rpi_wait_conversion_ready(ads1115_rpi_t *dev)
{
    uint32_t waited_us = 0;

    while (waited_us < ADS1115_RPI_CONVERSION_TIMEOUT_US) {
        uint16_t config = 0;
        int err = ads1115_rpi_read_register(dev,
                                            ADS1115_RPI_REG_CONFIG,
                                            &config);

        if (err < 0) {
            return err;
        }

        if ((config & ADS1115_RPI_OS_READY) != 0u) {
            return ADS1115_RPI_OK;
        }

        usleep(ADS1115_RPI_CONVERSION_POLL_US);
        waited_us += ADS1115_RPI_CONVERSION_POLL_US;
    }

    return ADS1115_RPI_ERR_TIMEOUT;
}

int ads1115_rpi_open(ads1115_rpi_t *dev,
                     const char *i2c_device,
                     uint8_t i2c_addr)
{
    int fd;

    if (dev == NULL || i2c_device == NULL || i2c_addr > 0x7Fu) {
        return ADS1115_RPI_ERR_INVALID_ARG;
    }

    memset(dev, 0, sizeof(*dev));
    dev->fd = -1;

    fd = open(i2c_device, O_RDWR);
    if (fd < 0) {
        return ADS1115_RPI_ERR_OPEN;
    }

    if (ioctl(fd, I2C_SLAVE, i2c_addr) < 0) {
        (void)close(fd);
        return ADS1115_RPI_ERR_IOCTL;
    }

    dev->fd = fd;
    dev->i2c_addr = i2c_addr;
    dev->gain = ADS1115_RPI_GAIN_4_096V;
    dev->data_rate = ADS1115_RPI_DATA_RATE_128SPS;

    return ADS1115_RPI_OK;
}

void ads1115_rpi_close(ads1115_rpi_t *dev)
{
    if (dev == NULL) {
        return;
    }

    if (dev->fd >= 0) {
        (void)close(dev->fd);
    }

    dev->fd = -1;
    dev->i2c_addr = 0;
    dev->gain = ADS1115_RPI_GAIN_4_096V;
    dev->data_rate = ADS1115_RPI_DATA_RATE_128SPS;
}

int ads1115_rpi_set_gain(ads1115_rpi_t *dev,
                         ads1115_rpi_gain_t gain)
{
    int err = ads1115_rpi_validate_device(dev);

    if (err < 0) {
        return err;
    }

    err = ads1115_rpi_validate_gain(gain);
    if (err < 0) {
        return err;
    }

    dev->gain = gain;
    return ADS1115_RPI_OK;
}

int ads1115_rpi_get_gain(ads1115_rpi_t *dev,
                         ads1115_rpi_gain_t *gain)
{
    int err = ads1115_rpi_validate_device(dev);

    if (err < 0) {
        return err;
    }

    if (gain == NULL) {
        return ADS1115_RPI_ERR_INVALID_ARG;
    }

    *gain = dev->gain;
    return ADS1115_RPI_OK;
}

int ads1115_rpi_set_data_rate(ads1115_rpi_t *dev,
                              ads1115_rpi_data_rate_t data_rate)
{
    int err = ads1115_rpi_validate_device(dev);

    if (err < 0) {
        return err;
    }

    err = ads1115_rpi_validate_data_rate(data_rate);
    if (err < 0) {
        return err;
    }

    dev->data_rate = data_rate;
    return ADS1115_RPI_OK;
}

int ads1115_rpi_get_data_rate(ads1115_rpi_t *dev,
                              ads1115_rpi_data_rate_t *data_rate)
{
    int err = ads1115_rpi_validate_device(dev);

    if (err < 0) {
        return err;
    }

    if (data_rate == NULL) {
        return ADS1115_RPI_ERR_INVALID_ARG;
    }

    *data_rate = dev->data_rate;
    return ADS1115_RPI_OK;
}

int ads1115_rpi_read_raw(ads1115_rpi_t *dev,
                         ads1115_rpi_channel_t channel,
                         int16_t *raw)
{
    uint16_t config;
    uint16_t value;
    int err;

    err = ads1115_rpi_validate_device(dev);
    if (err < 0) {
        return err;
    }

    err = ads1115_rpi_validate_channel(channel);
    if (err < 0) {
        return err;
    }

    err = ads1115_rpi_validate_gain(dev->gain);
    if (err < 0) {
        return err;
    }

    err = ads1115_rpi_validate_data_rate(dev->data_rate);
    if (err < 0) {
        return err;
    }

    if (raw == NULL) {
        return ADS1115_RPI_ERR_INVALID_ARG;
    }

    config = ads1115_rpi_build_config(dev, channel);

    err = ads1115_rpi_write_register(dev, ADS1115_RPI_REG_CONFIG, config);
    if (err < 0) {
        return err;
    }

    err = ads1115_rpi_wait_conversion_ready(dev);
    if (err < 0) {
        return err;
    }

    err = ads1115_rpi_read_register(dev, ADS1115_RPI_REG_CONVERSION, &value);
    if (err < 0) {
        return err;
    }

    *raw = (int16_t)value;
    return ADS1115_RPI_OK;
}

int ads1115_rpi_read_voltage(ads1115_rpi_t *dev,
                             ads1115_rpi_channel_t channel,
                             float *voltage)
{
    int16_t raw = 0;
    int err;

    if (voltage == NULL) {
        return ADS1115_RPI_ERR_INVALID_ARG;
    }

    err = ads1115_rpi_read_raw(dev, channel, &raw);
    if (err < 0) {
        return err;
    }

    *voltage = ((float)raw * ads1115_rpi_gain_full_scale_voltage(dev->gain)) /
               32768.0f;

    return ADS1115_RPI_OK;
}

float ads1115_rpi_gain_full_scale_voltage(ads1115_rpi_gain_t gain)
{
    switch (gain) {
    case ADS1115_RPI_GAIN_6_144V:
        return 6.144f;
    case ADS1115_RPI_GAIN_4_096V:
        return 4.096f;
    case ADS1115_RPI_GAIN_2_048V:
        return 2.048f;
    case ADS1115_RPI_GAIN_1_024V:
        return 1.024f;
    case ADS1115_RPI_GAIN_0_512V:
        return 0.512f;
    case ADS1115_RPI_GAIN_0_256V:
        return 0.256f;
    default:
        return 0.0f;
    }
}

int ads1115_rpi_write_register(ads1115_rpi_t *dev,
                               uint8_t reg,
                               uint16_t value)
{
    uint8_t data[3];
    ssize_t written;
    int err = ads1115_rpi_validate_device(dev);

    if (err < 0) {
        return err;
    }

    data[0] = reg;
    data[1] = (uint8_t)((value >> 8) & 0xFFu);
    data[2] = (uint8_t)(value & 0xFFu);

    written = write(dev->fd, data, sizeof(data));
    if (written != (ssize_t)sizeof(data)) {
        return ADS1115_RPI_ERR_WRITE;
    }

    return ADS1115_RPI_OK;
}

int ads1115_rpi_read_register(ads1115_rpi_t *dev,
                              uint8_t reg,
                              uint16_t *value)
{
    uint8_t data[2];
    ssize_t written;
    ssize_t bytes_read;
    int err = ads1115_rpi_validate_device(dev);

    if (err < 0) {
        return err;
    }

    if (value == NULL) {
        return ADS1115_RPI_ERR_INVALID_ARG;
    }

    written = write(dev->fd, &reg, sizeof(reg));
    if (written != (ssize_t)sizeof(reg)) {
        return ADS1115_RPI_ERR_WRITE;
    }

    bytes_read = read(dev->fd, data, sizeof(data));
    if (bytes_read != (ssize_t)sizeof(data)) {
        return ADS1115_RPI_ERR_READ;
    }

    *value = (uint16_t)(((uint16_t)data[0] << 8) | (uint16_t)data[1]);
    return ADS1115_RPI_OK;
}
