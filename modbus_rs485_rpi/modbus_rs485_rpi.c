#include "modbus_rs485_rpi.h"

#include <errno.h>
#include <fcntl.h>
#include <stdbool.h>
#include <string.h>
#include <sys/select.h>
#include <sys/time.h>
#include <termios.h>
#include <unistd.h>

#define MODBUS_FUNC_READ_HOLDING 0x03
#define MODBUS_RTU_READ_HOLDING_REQ_LEN 8

static speed_t baudrate_to_speed(int baudrate)
{
    switch (baudrate) {
    case 1200:
        return B1200;
    case 2400:
        return B2400;
    case 4800:
        return B4800;
    case 9600:
        return B9600;
    case 19200:
        return B19200;
    case 38400:
        return B38400;
    case 57600:
        return B57600;
    case 115200:
        return B115200;
    default:
        return 0;
    }
}

static int set_raw_8n1(int fd, int baudrate)
{
    struct termios tty;
    speed_t speed = baudrate_to_speed(baudrate);

    if (speed == 0) {
        return MODBUS_RS485_RPI_ERR_UART_CONFIG;
    }

    if (tcgetattr(fd, &tty) < 0) {
        return MODBUS_RS485_RPI_ERR_UART_CONFIG;
    }

    tty.c_cflag &= (tcflag_t)~PARENB;
    tty.c_cflag &= (tcflag_t)~CSTOPB;
    tty.c_cflag &= (tcflag_t)~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag |= CLOCAL | CREAD;
#ifdef CRTSCTS
    tty.c_cflag &= (tcflag_t)~CRTSCTS;
#endif

    tty.c_iflag &= (tcflag_t)~(IGNBRK | BRKINT | PARMRK | ISTRIP |
                               INLCR | IGNCR | ICRNL | IXON | IXOFF | IXANY);
    tty.c_oflag &= (tcflag_t)~OPOST;
    tty.c_lflag &= (tcflag_t)~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);

    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 0;

    if (cfsetispeed(&tty, speed) < 0 || cfsetospeed(&tty, speed) < 0) {
        return MODBUS_RS485_RPI_ERR_UART_CONFIG;
    }

    if (tcsetattr(fd, TCSANOW, &tty) < 0) {
        return MODBUS_RS485_RPI_ERR_UART_CONFIG;
    }

    if (tcflush(fd, TCIOFLUSH) < 0) {
        return MODBUS_RS485_RPI_ERR_UART_CONFIG;
    }

    return MODBUS_RS485_RPI_OK;
}

static int set_direction(const modbus_rs485_rpi_bus_t *bus, int tx_enable)
{
    int err;

    if (bus == NULL || bus->gpio_set == NULL) {
        return MODBUS_RS485_RPI_OK;
    }

    if (bus->de_gpio != MODBUS_RS485_RPI_GPIO_UNUSED) {
        err = bus->gpio_set(bus->de_gpio, tx_enable ? 1 : 0, bus->gpio_user_ctx);
        if (err < 0) {
            return err;
        }
    }

    if (bus->re_gpio != MODBUS_RS485_RPI_GPIO_UNUSED) {
        err = bus->gpio_set(bus->re_gpio, tx_enable ? 1 : 0, bus->gpio_user_ctx);
        if (err < 0) {
            return err;
        }
    }

    return MODBUS_RS485_RPI_OK;
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
            return MODBUS_RS485_RPI_ERR_WRITE;
        }

        if (written == 0) {
            return MODBUS_RS485_RPI_ERR_WRITE;
        }

        total += (size_t)written;
    }

    return MODBUS_RS485_RPI_OK;
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
        return MODBUS_RS485_RPI_ERR_READ;
    }

    while (total < len) {
        fd_set read_fds;
        struct timeval tv;
        int remaining_ms;
        int ready;

        if (elapsed_timeout_ms(&start, timeout_ms)) {
            return MODBUS_RS485_RPI_ERR_TIMEOUT;
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
            return MODBUS_RS485_RPI_ERR_READ;
        }

        if (ready == 0) {
            return MODBUS_RS485_RPI_ERR_TIMEOUT;
        }

        if (FD_ISSET(fd, &read_fds)) {
            ssize_t bytes_read = read(fd, buf + total, len - total);

            if (bytes_read < 0) {
                if (errno == EINTR) {
                    continue;
                }
                return MODBUS_RS485_RPI_ERR_READ;
            }

            if (bytes_read == 0) {
                return MODBUS_RS485_RPI_ERR_TIMEOUT;
            }

            total += (size_t)bytes_read;
        }
    }

    return MODBUS_RS485_RPI_OK;
}

uint16_t modbus_rs485_rpi_crc16(const uint8_t *buf, uint16_t len)
{
    uint16_t crc = 0xFFFF;

    if (buf == NULL) {
        return 0;
    }

    for (uint16_t pos = 0; pos < len; pos++) {
        crc ^= (uint16_t)buf[pos];
        for (int i = 0; i < 8; i++) {
            if ((crc & 0x0001u) != 0u) {
                crc >>= 1;
                crc ^= 0xA001u;
            } else {
                crc >>= 1;
            }
        }
    }

    return crc;
}

int modbus_rs485_rpi_open(modbus_rs485_rpi_bus_t *bus,
                          const char *device_path,
                          int baudrate)
{
    int fd;
    int err;

    if (bus == NULL || device_path == NULL) {
        return MODBUS_RS485_RPI_ERR_INVALID_ARG;
    }

    memset(bus, 0, sizeof(*bus));
    bus->fd = -1;
    bus->device_path = device_path;
    bus->baudrate = baudrate;
    bus->de_gpio = MODBUS_RS485_RPI_GPIO_UNUSED;
    bus->re_gpio = MODBUS_RS485_RPI_GPIO_UNUSED;

    fd = open(device_path, O_RDWR | O_NOCTTY);
    if (fd < 0) {
        return MODBUS_RS485_RPI_ERR_OPEN;
    }

    err = set_raw_8n1(fd, baudrate);
    if (err < 0) {
        (void)close(fd);
        return err;
    }

    bus->fd = fd;
    return MODBUS_RS485_RPI_OK;
}

void modbus_rs485_rpi_close(modbus_rs485_rpi_bus_t *bus)
{
    if (bus == NULL) {
        return;
    }

    if (bus->fd >= 0) {
        (void)close(bus->fd);
    }

    bus->fd = -1;
    bus->device_path = NULL;
    bus->baudrate = 0;
    bus->de_gpio = MODBUS_RS485_RPI_GPIO_UNUSED;
    bus->re_gpio = MODBUS_RS485_RPI_GPIO_UNUSED;
    bus->gpio_set = NULL;
    bus->gpio_user_ctx = NULL;
}

int modbus_rs485_rpi_set_direction_control(modbus_rs485_rpi_bus_t *bus,
                                           int de_gpio,
                                           int re_gpio,
                                           modbus_rs485_gpio_set_cb_t gpio_set,
                                           void *user_ctx)
{
    if (bus == NULL) {
        return MODBUS_RS485_RPI_ERR_INVALID_ARG;
    }

    if ((de_gpio != MODBUS_RS485_RPI_GPIO_UNUSED ||
         re_gpio != MODBUS_RS485_RPI_GPIO_UNUSED) &&
        gpio_set == NULL) {
        return MODBUS_RS485_RPI_ERR_INVALID_ARG;
    }

    bus->de_gpio = de_gpio;
    bus->re_gpio = re_gpio;
    bus->gpio_set = gpio_set;
    bus->gpio_user_ctx = user_ctx;

    return MODBUS_RS485_RPI_OK;
}

int modbus_rs485_rpi_read_holding(const modbus_rs485_rpi_bus_t *bus,
                                  uint8_t slave_id,
                                  uint16_t start_addr,
                                  uint16_t num_regs,
                                  uint8_t *resp,
                                  size_t resp_len,
                                  int timeout_ms)
{
    uint8_t req[MODBUS_RTU_READ_HOLDING_REQ_LEN];
    uint16_t crc;
    uint16_t rx_crc;
    uint16_t calc_crc;
    size_t expected;
    int err;

    if (bus == NULL || bus->fd < 0 || resp == NULL || num_regs == 0 || timeout_ms <= 0) {
        return MODBUS_RS485_RPI_ERR_INVALID_ARG;
    }

    expected = 5u + (2u * (size_t)num_regs);
    if (resp_len < expected) {
        return MODBUS_RS485_RPI_ERR_INVALID_SIZE;
    }

    req[0] = slave_id;
    req[1] = MODBUS_FUNC_READ_HOLDING;
    req[2] = (uint8_t)((start_addr >> 8) & 0xFFu);
    req[3] = (uint8_t)(start_addr & 0xFFu);
    req[4] = (uint8_t)((num_regs >> 8) & 0xFFu);
    req[5] = (uint8_t)(num_regs & 0xFFu);

    crc = modbus_rs485_rpi_crc16(req, 6);
    req[6] = (uint8_t)(crc & 0xFFu);
    req[7] = (uint8_t)((crc >> 8) & 0xFFu);

    (void)tcflush(bus->fd, TCIFLUSH);

    err = set_direction(bus, 1);
    if (err < 0) {
        return MODBUS_RS485_RPI_ERR_WRITE;
    }

    err = write_all(bus->fd, req, sizeof(req));
    if (err < 0) {
        (void)set_direction(bus, 0);
        return err;
    }

    if (tcdrain(bus->fd) < 0) {
        (void)set_direction(bus, 0);
        return MODBUS_RS485_RPI_ERR_WRITE;
    }

    err = set_direction(bus, 0);
    if (err < 0) {
        return MODBUS_RS485_RPI_ERR_WRITE;
    }

    err = read_exact_timeout(bus->fd, resp, expected, timeout_ms);
    if (err < 0) {
        return err;
    }

    rx_crc = ((uint16_t)resp[expected - 1] << 8) | (uint16_t)resp[expected - 2];
    calc_crc = modbus_rs485_rpi_crc16(resp, (uint16_t)(expected - 2u));
    if (rx_crc != calc_crc) {
        return MODBUS_RS485_RPI_ERR_CRC;
    }

    if (resp[0] != slave_id || resp[1] != MODBUS_FUNC_READ_HOLDING ||
        resp[2] != (uint8_t)(2u * num_regs)) {
        return MODBUS_RS485_RPI_ERR_HEADER;
    }

    return MODBUS_RS485_RPI_OK;
}

int modbus_rs485_rpi_get_u16(const uint8_t *resp,
                             size_t resp_len,
                             uint16_t reg_index,
                             uint16_t *value)
{
    size_t high_index;

    if (resp == NULL || value == NULL) {
        return MODBUS_RS485_RPI_ERR_INVALID_ARG;
    }

    high_index = 3u + ((size_t)reg_index * 2u);
    if ((high_index + 1u) >= resp_len) {
        return MODBUS_RS485_RPI_ERR_INVALID_SIZE;
    }

    *value = ((uint16_t)resp[high_index] << 8) | (uint16_t)resp[high_index + 1u];
    return MODBUS_RS485_RPI_OK;
}
