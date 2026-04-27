#include "modbus_rs485.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"


static const char *TAG = "MODBUS_RS485";

// Modbus function code
#define MODBUS_FUNC_READ_HOLDING 0x03

uint16_t modbus_rs485_crc16(const uint8_t *buf, uint16_t len) {
  uint16_t crc = 0xFFFF;
  for (uint16_t pos = 0; pos < len; pos++) {
    crc ^= (uint16_t)buf[pos];
    for (int i = 0; i < 8; i++) {
      if (crc & 0x0001) {
        crc >>= 1;
        crc ^= 0xA001;
      } else {
        crc >>= 1;
      }
    }
  }
  return crc;
}

esp_err_t modbus_rs485_read_holding(const modbus_rs485_bus_t *bus,
                                    uint8_t slave_id, uint16_t start_addr,
                                    uint16_t num_regs, uint8_t *resp,
                                    size_t resp_len, int timeout_ms) {
  if (!bus || !resp) {
    return ESP_ERR_INVALID_ARG;
  }

  // Build Modbus request frame
  uint8_t req[8];
  req[0] = slave_id;
  req[1] = MODBUS_FUNC_READ_HOLDING;
  req[2] = (start_addr >> 8) & 0xFF;
  req[3] = start_addr & 0xFF;
  req[4] = (num_regs >> 8) & 0xFF;
  req[5] = num_regs & 0xFF;

  uint16_t crc = modbus_rs485_crc16(req, 6);
  req[6] = crc & 0xFF;        // CRC low byte
  req[7] = (crc >> 8) & 0xFF; // CRC high byte

  // Expected response: addr(1) + func(1) + byte_count(1) + data(2*regs) +
  // CRC(2)
  size_t expected = 5 + 2 * num_regs;
  if (resp_len < expected) {
    return ESP_ERR_INVALID_SIZE;
  }

  // Switch to TX mode if DE/RE pins are configured
  if (bus->de_gpio != GPIO_NUM_NC) {
    gpio_set_level(bus->de_gpio, 1);
  }
  if (bus->re_gpio != GPIO_NUM_NC) {
    gpio_set_level(bus->re_gpio, 1);
  }

  // Flush input and send request
  uart_flush_input(bus->uart_port);
  int written =
      uart_write_bytes(bus->uart_port, (const char *)req, sizeof(req));
  if (written != sizeof(req)) {
    ESP_LOGW(TAG, "UART write incomplete: %d/%d", written, (int)sizeof(req));
    // Switch back to RX mode
    if (bus->de_gpio != GPIO_NUM_NC) {
      gpio_set_level(bus->de_gpio, 0);
    }
    if (bus->re_gpio != GPIO_NUM_NC) {
      gpio_set_level(bus->re_gpio, 0);
    }
    return ESP_FAIL;
  }

  // Wait for TX to complete before switching to RX
  uart_wait_tx_done(bus->uart_port, pdMS_TO_TICKS(100));

  // Switch to RX mode
  if (bus->de_gpio != GPIO_NUM_NC) {
    gpio_set_level(bus->de_gpio, 0);
  }
  if (bus->re_gpio != GPIO_NUM_NC) {
    gpio_set_level(bus->re_gpio, 0);
  }

  // Read response
  int len = uart_read_bytes(bus->uart_port, resp, expected,
                            pdMS_TO_TICKS(timeout_ms));
  if (len != (int)expected) {
    ESP_LOGD(TAG, "Modbus read size mismatch: got %d exp %d", len,
             (int)expected);
    return ESP_FAIL;
  }

  // Verify CRC
  uint16_t rx_crc =
      ((uint16_t)resp[expected - 1] << 8) | (uint16_t)resp[expected - 2];
  uint16_t calc_crc = modbus_rs485_crc16(resp, expected - 2);
  if (rx_crc != calc_crc) {
    ESP_LOGW(TAG, "Modbus CRC error: rx=0x%04X calc=0x%04X", rx_crc, calc_crc);
    return ESP_FAIL;
  }

  // Verify header
  if (resp[0] != slave_id || resp[1] != MODBUS_FUNC_READ_HOLDING) {
    ESP_LOGW(TAG, "Modbus header mismatch: addr=%d func=%d", resp[0], resp[1]);
    return ESP_FAIL;
  }

  return ESP_OK;
}
