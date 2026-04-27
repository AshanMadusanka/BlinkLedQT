#ifndef MODBUS_RS485_RPI_H
#define MODBUS_RS485_RPI_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MODBUS_RS485_RPI_OK                0
#define MODBUS_RS485_RPI_ERR_INVALID_ARG  -1
#define MODBUS_RS485_RPI_ERR_INVALID_SIZE -2
#define MODBUS_RS485_RPI_ERR_OPEN         -3
#define MODBUS_RS485_RPI_ERR_UART_CONFIG  -4
#define MODBUS_RS485_RPI_ERR_WRITE        -5
#define MODBUS_RS485_RPI_ERR_TIMEOUT      -6
#define MODBUS_RS485_RPI_ERR_CRC          -7
#define MODBUS_RS485_RPI_ERR_HEADER       -8
#define MODBUS_RS485_RPI_ERR_READ         -9

#define MODBUS_RS485_RPI_GPIO_UNUSED -1

typedef int (*modbus_rs485_gpio_set_cb_t)(int gpio, int level, void *user_ctx);

typedef struct {
    int fd;
    const char *device_path;
    int baudrate;
    int de_gpio;
    int re_gpio;
    modbus_rs485_gpio_set_cb_t gpio_set;
    void *gpio_user_ctx;
} modbus_rs485_rpi_bus_t;

/* Calculate Modbus RTU CRC16 for a buffer. The wire format sends low byte first. */
uint16_t modbus_rs485_rpi_crc16(const uint8_t *buf, uint16_t len);

/* Open and configure a Linux UART device for Modbus RTU RS485 as 8N1 raw serial. */
int modbus_rs485_rpi_open(modbus_rs485_rpi_bus_t *bus,
                          const char *device_path,
                          int baudrate);

/* Close the UART file descriptor and reset the bus descriptor to a safe closed state. */
void modbus_rs485_rpi_close(modbus_rs485_rpi_bus_t *bus);

/* Configure optional DE/RE direction GPIO control through a user-provided callback. */
int modbus_rs485_rpi_set_direction_control(modbus_rs485_rpi_bus_t *bus,
                                           int de_gpio,
                                           int re_gpio,
                                           modbus_rs485_gpio_set_cb_t gpio_set,
                                           void *user_ctx);

/* Send Modbus function 0x03 and read holding-register response bytes into resp. */
int modbus_rs485_rpi_read_holding(const modbus_rs485_rpi_bus_t *bus,
                                  uint8_t slave_id,
                                  uint16_t start_addr,
                                  uint16_t num_regs,
                                  uint8_t *resp,
                                  size_t resp_len,
                                  int timeout_ms);

/* Extract one big-endian 16-bit register value from a valid function 0x03 response. */
int modbus_rs485_rpi_get_u16(const uint8_t *resp,
                             size_t resp_len,
                             uint16_t reg_index,
                             uint16_t *value);

#ifdef __cplusplus
}
#endif

#endif /* MODBUS_RS485_RPI_H */
