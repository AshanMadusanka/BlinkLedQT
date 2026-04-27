#ifndef _DEFAULT_SOURCE
#define _DEFAULT_SOURCE
#endif

#include "sn3002_co2_rpi.h"

#include <errno.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <sys/select.h>
#include <sys/time.h>
#include <termios.h>
#include <unistd.h>

#define SN3002_CO2_RPI_REG_CO2         0x0002u
#define SN3002_CO2_RPI_REG_CALIB       0x0052u
#define SN3002_CO2_RPI_REG_DEVICE_ADDR 0x07D0u
#define SN3002_CO2_RPI_REG_BAUD_RATE   0x07D1u

#define SN3002_CO2_RPI_FUNC_WRITE_SINGLE 0x06u
#define SN3002_CO2_RPI_FRAME_LEN         8u
#define SN3002_CO2_RPI_RETRIES           2
#define SN3002_CO2_RPI_TIMEOUT_MS        2000
#define SN3002_CO2_RPI_RETRY_US          (50 * 1000)

struct sn3002_co2_rpi_dev {
    modbus_rs485_rpi_bus_t *bus;
    uint8_t slave_id;
};

static int set_direction(const modbus_rs485_rpi_bus_t *bus, int tx_enable)
{
    int err;

    if (bus == NULL || bus->gpio_set == NULL) {
        return SN3002_CO2_RPI_OK;
    }

    if (bus->de_gpio != MODBUS_RS485_RPI_GPIO_UNUSED) {
        err = bus->gpio_set(bus->de_gpio, tx_enable ? 1 : 0, bus->gpio_user_ctx);
        if (err < 0) {
            return SN3002_CO2_RPI_ERR_WRITE;
        }
    }

    if (bus->re_gpio != MODBUS_RS485_RPI_GPIO_UNUSED) {
        err = bus->gpio_set(bus->re_gpio, tx_enable ? 1 : 0, bus->gpio_user_ctx);
        if (err < 0) {
            return SN3002_CO2_RPI_ERR_WRITE;
        }
    }

    return SN3002_CO2_RPI_OK;
}

static int write_all(int fd, const uint8_t *buf, size_t len)
{
    size_t total = 0;

    while (total < len) {
        ssize_t written = write(fd, buf + total, len - total);

        if (written < 0) {
            if (errno == EINTR) {
                continue;
            }
            return SN3002_CO2_RPI_ERR_WRITE;
        }

        if (written == 0) {
            return SN3002_CO2_RPI_ERR_WRITE;
        }

        total += (size_t)written;
    }

    return SN3002_CO2_RPI_OK;
}

static int elapsed_timeout_ms(const struct timeval *start, int timeout_ms)
{
    struct timeval now;
    long elapsed_ms;

    if (gettimeofday(&now, NULL) < 0) {
        return 0;
    }

    elapsed_ms = (now.tv_sec - start->tv_sec) * 1000L;
    elapsed_ms += (now.tv_usec - start->tv_usec) / 1000L;

    return elapsed_ms >= timeout_ms;
}

static int remaining_timeout_ms(const struct timeval *start, int timeout_ms)
{
    struct timeval now;
    long elapsed_ms;

    if (gettimeofday(&now, NULL) < 0) {
        return timeout_ms;
    }

    elapsed_ms = (now.tv_sec - start->tv_sec) * 1000L;
    elapsed_ms += (now.tv_usec - start->tv_usec) / 1000L;

    if (elapsed_ms >= timeout_ms) {
        return 0;
    }

    return (int)(timeout_ms - elapsed_ms);
}

static int read_exact_timeout(int fd, uint8_t *buf, size_t len, int timeout_ms)
{
    size_t total = 0;
    struct timeval start;

    if (gettimeofday(&start, NULL) < 0) {
        return SN3002_CO2_RPI_ERR_READ;
    }

    while (total < len) {
        fd_set read_fds;
        struct timeval tv;
        int remaining_ms;
        int ready;

        if (elapsed_timeout_ms(&start, timeout_ms)) {
            return SN3002_CO2_RPI_ERR_TIMEOUT;
        }

        remaining_ms = remaining_timeout_ms(&start, timeout_ms);
        tv.tv_sec = remaining_ms / 1000;
        tv.tv_usec = (remaining_ms % 1000) * 1000;

        FD_ZERO(&read_fds);
        FD_SET(fd, &read_fds);

        ready = select(fd + 1, &read_fds, NULL, NULL, &tv);
        if (ready < 0) {
            if (errno == EINTR) {
                continue;
            }
            return SN3002_CO2_RPI_ERR_READ;
        }

        if (ready == 0) {
            return SN3002_CO2_RPI_ERR_TIMEOUT;
        }

        if (FD_ISSET(fd, &read_fds)) {
            ssize_t bytes_read = read(fd, buf + total, len - total);

            if (bytes_read < 0) {
                if (errno == EINTR) {
                    continue;
                }
                return SN3002_CO2_RPI_ERR_READ;
            }

            if (bytes_read == 0) {
                return SN3002_CO2_RPI_ERR_TIMEOUT;
            }

            total += (size_t)bytes_read;
        }
    }

    return SN3002_CO2_RPI_OK;
}

int sn3002_co2_rpi_create(const sn3002_co2_rpi_config_t *config,
                          sn3002_co2_rpi_handle_t *out_handle)
{
    struct sn3002_co2_rpi_dev *dev;

    if (config == NULL || out_handle == NULL || config->bus == NULL) {
        return SN3002_CO2_RPI_ERR_INVALID_ARG;
    }

    dev = (struct sn3002_co2_rpi_dev *)calloc(1, sizeof(*dev));
    if (dev == NULL) {
        return SN3002_CO2_RPI_ERR_NO_MEMORY;
    }

    dev->bus = config->bus;
    dev->slave_id = config->slave_id;

    *out_handle = dev;
    return SN3002_CO2_RPI_OK;
}

void sn3002_co2_rpi_destroy(sn3002_co2_rpi_handle_t handle)
{
    free(handle);
}

int sn3002_co2_rpi_set_slave_id(sn3002_co2_rpi_handle_t handle,
                                uint8_t slave_id)
{
    if (handle == NULL) {
        return SN3002_CO2_RPI_ERR_INVALID_ARG;
    }

    handle->slave_id = slave_id;
    return SN3002_CO2_RPI_OK;
}

int sn3002_co2_rpi_read(sn3002_co2_rpi_handle_t handle,
                        uint32_t *co2_ppm)
{
    uint8_t resp[16];

    if (handle == NULL || handle->bus == NULL || co2_ppm == NULL) {
        return SN3002_CO2_RPI_ERR_INVALID_ARG;
    }

    for (int attempt = 0; attempt < SN3002_CO2_RPI_RETRIES; ++attempt) {
        int err = modbus_rs485_rpi_read_holding(handle->bus,
                                                handle->slave_id,
                                                SN3002_CO2_RPI_REG_CO2,
                                                1,
                                                resp,
                                                sizeof(resp),
                                                SN3002_CO2_RPI_TIMEOUT_MS);
        if (err == MODBUS_RS485_RPI_OK) {
            uint16_t raw_co2 = ((uint16_t)resp[3] << 8) | (uint16_t)resp[4];
            *co2_ppm = (uint32_t)raw_co2;
            return SN3002_CO2_RPI_OK;
        }

        if (attempt + 1 < SN3002_CO2_RPI_RETRIES) {
            usleep(SN3002_CO2_RPI_RETRY_US);
        }
    }

    return SN3002_CO2_RPI_ERR_MODBUS;
}

int sn3002_co2_rpi_calibrate(sn3002_co2_rpi_handle_t handle,
                             int calib_value)
{
    uint8_t req[SN3002_CO2_RPI_FRAME_LEN];
    uint8_t resp[SN3002_CO2_RPI_FRAME_LEN];
    uint16_t calib_u16;
    uint16_t crc;
    uint16_t rx_crc;
    uint16_t calc_crc;
    int err;

    if (handle == NULL || handle->bus == NULL || handle->bus->fd < 0) {
        return SN3002_CO2_RPI_ERR_INVALID_ARG;
    }

    calib_u16 = (uint16_t)((int16_t)calib_value);

    req[0] = handle->slave_id;
    req[1] = SN3002_CO2_RPI_FUNC_WRITE_SINGLE;
    req[2] = (uint8_t)((SN3002_CO2_RPI_REG_CALIB >> 8) & 0xFFu);
    req[3] = (uint8_t)(SN3002_CO2_RPI_REG_CALIB & 0xFFu);
    req[4] = (uint8_t)((calib_u16 >> 8) & 0xFFu);
    req[5] = (uint8_t)(calib_u16 & 0xFFu);

    crc = modbus_rs485_rpi_crc16(req, 6);
    req[6] = (uint8_t)(crc & 0xFFu);
    req[7] = (uint8_t)((crc >> 8) & 0xFFu);

    if (tcflush(handle->bus->fd, TCIFLUSH) < 0) {
        return SN3002_CO2_RPI_ERR_READ;
    }

    err = set_direction(handle->bus, 1);
    if (err < 0) {
        return err;
    }

    err = write_all(handle->bus->fd, req, sizeof(req));
    if (err < 0) {
        (void)set_direction(handle->bus, 0);
        return err;
    }

    if (tcdrain(handle->bus->fd) < 0) {
        (void)set_direction(handle->bus, 0);
        return SN3002_CO2_RPI_ERR_WRITE;
    }

    err = set_direction(handle->bus, 0);
    if (err < 0) {
        return err;
    }

    err = read_exact_timeout(handle->bus->fd,
                             resp,
                             sizeof(resp),
                             SN3002_CO2_RPI_TIMEOUT_MS);
    if (err < 0) {
        return err;
    }

    rx_crc = ((uint16_t)resp[7] << 8) | (uint16_t)resp[6];
    calc_crc = modbus_rs485_rpi_crc16(resp, 6);
    if (rx_crc != calc_crc) {
        return SN3002_CO2_RPI_ERR_CRC;
    }

    if (resp[0] != handle->slave_id ||
        resp[1] != SN3002_CO2_RPI_FUNC_WRITE_SINGLE ||
        resp[2] != req[2] ||
        resp[3] != req[3] ||
        resp[4] != req[4] ||
        resp[5] != req[5]) {
        return SN3002_CO2_RPI_ERR_HEADER;
    }

    return SN3002_CO2_RPI_OK;
}
