#include "sn3002_co2.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "modbus_rs485.h"
#include <stdlib.h>

static const char *TAG = "SN3002_CO2";

// Modbus register addresses for SN3002 CO2 sensor
#define SN3002_CO2_REG_CO2 0x0002         // CO2 ppm value register
#define SN3002_CO2_REG_CALIB 0x0052       // Calibration register
#define SN3002_CO2_REG_DEVICE_ADDR 0x07D0 // Device address register
#define SN3002_CO2_REG_BAUD_RATE 0x07D1   // Baud rate register

// Modbus function codes
#define MODBUS_FUNC_WRITE_SINGLE 0x06

typedef struct sn3002_co2_dev {
  sn3002_co2_config_t cfg;
  modbus_rs485_bus_t bus;
} sn3002_co2_dev_t;

// ---- Public API ----

esp_err_t sn3002_co2_create(const sn3002_co2_config_t *config,
                            sn3002_co2_handle_t *out_handle) {
  if (!config || !out_handle) {
    return ESP_ERR_INVALID_ARG;
  }

  sn3002_co2_dev_t *dev = calloc(1, sizeof(sn3002_co2_dev_t));
  if (!dev) {
    return ESP_ERR_NO_MEM;
  }

  dev->cfg = *config;

  // Setup bus configuration for shared Modbus layer
  dev->bus.uart_port = config->uart_port;
  dev->bus.de_gpio = config->de_gpio;
  dev->bus.re_gpio = config->re_gpio;

  if (config->init_uart) {
    uart_config_t uart_config = {
        .baud_rate = config->baud_rate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };

    ESP_ERROR_CHECK(uart_param_config(config->uart_port, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(config->uart_port, config->tx_gpio,
                                 config->rx_gpio, UART_PIN_NO_CHANGE,
                                 UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(
        uart_driver_install(config->uart_port, 256, 256, 0, NULL, 0));
  }

  // Setup DE/RE pins if present
  if (config->de_gpio != GPIO_NUM_NC) {
    gpio_reset_pin(config->de_gpio);
    gpio_set_direction(config->de_gpio, GPIO_MODE_OUTPUT);
    gpio_set_level(config->de_gpio, 0); // default RX mode
  }
  if (config->re_gpio != GPIO_NUM_NC) {
    gpio_reset_pin(config->re_gpio);
    gpio_set_direction(config->re_gpio, GPIO_MODE_OUTPUT);
    gpio_set_level(config->re_gpio, 0); // default RX mode
  }

  *out_handle = (sn3002_co2_handle_t)dev;
  ESP_LOGI(TAG, "SN3002 CO2 sensor created on UART%d, slave_id=%d",
           config->uart_port, config->slave_id);
  return ESP_OK;
}

esp_err_t sn3002_co2_destroy(sn3002_co2_handle_t handle) {
  if (!handle) {
    return ESP_ERR_INVALID_ARG;
  }
  sn3002_co2_dev_t *dev = (sn3002_co2_dev_t *)handle;
  free(dev);
  return ESP_OK;
}

esp_err_t sn3002_co2_set_slave_id(sn3002_co2_handle_t handle,
                                  uint8_t slave_id) {
  if (!handle) {
    return ESP_ERR_INVALID_ARG;
  }
  sn3002_co2_dev_t *dev = (sn3002_co2_dev_t *)handle;
  dev->cfg.slave_id = slave_id;
  return ESP_OK;
}

esp_err_t sn3002_co2_read(sn3002_co2_handle_t handle, uint32_t *co2_ppm) {
  if (!handle || !co2_ppm) {
    return ESP_ERR_INVALID_ARG;
  }

  sn3002_co2_dev_t *dev = (sn3002_co2_dev_t *)handle;

  uint8_t buf[16] = {0};

  // Retry mechanism for robustness
  for (int attempt = 0; attempt < 2; ++attempt) {
    esp_err_t ret = modbus_rs485_read_holding(&dev->bus, dev->cfg.slave_id,
                                              SN3002_CO2_REG_CO2, 1, buf,
                                              sizeof(buf), 2000);
    if (ret == ESP_OK) {
      // Response format: addr(1) + func(1) + byte_count(1) + data(2) + CRC(2)
      // Data starts at buf[3]
      uint16_t raw_co2 = ((uint16_t)buf[3] << 8) | buf[4];
      *co2_ppm = (uint32_t)raw_co2;
      return ESP_OK;
    }

    // Small delay before retry
    vTaskDelay(pdMS_TO_TICKS(50));
  }

  return ESP_FAIL;
}

esp_err_t sn3002_co2_calibrate(sn3002_co2_handle_t handle, int calib_value) {
  if (!handle) {
    return ESP_ERR_INVALID_ARG;
  }

  sn3002_co2_dev_t *dev = (sn3002_co2_dev_t *)handle;

  // Handle negative calibration values (two's complement)
  uint16_t calib_u16 =
      (calib_value < 0) ? (uint16_t)(~calib_value + 1) : (uint16_t)calib_value;

  // Build Modbus write single register request (function 0x06)
  uint8_t req[8];
  req[0] = dev->cfg.slave_id;
  req[1] = MODBUS_FUNC_WRITE_SINGLE;
  req[2] = (SN3002_CO2_REG_CALIB >> 8) & 0xFF;
  req[3] = SN3002_CO2_REG_CALIB & 0xFF;
  req[4] = (calib_u16 >> 8) & 0xFF;
  req[5] = calib_u16 & 0xFF;

  uint16_t crc = modbus_rs485_crc16(req, 6);
  req[6] = crc & 0xFF;
  req[7] = (crc >> 8) & 0xFF;

  // Switch to TX mode
  if (dev->bus.de_gpio != GPIO_NUM_NC) {
    gpio_set_level(dev->bus.de_gpio, 1);
  }
  if (dev->bus.re_gpio != GPIO_NUM_NC) {
    gpio_set_level(dev->bus.re_gpio, 1);
  }

  // Flush and send
  uart_flush_input(dev->bus.uart_port);
  int written =
      uart_write_bytes(dev->bus.uart_port, (const char *)req, sizeof(req));
  if (written != sizeof(req)) {
    ESP_LOGW(TAG, "Calibration write incomplete: %d/%d", written,
             (int)sizeof(req));
    // Switch back to RX mode
    if (dev->bus.de_gpio != GPIO_NUM_NC) {
      gpio_set_level(dev->bus.de_gpio, 0);
    }
    if (dev->bus.re_gpio != GPIO_NUM_NC) {
      gpio_set_level(dev->bus.re_gpio, 0);
    }
    return ESP_FAIL;
  }

  // Wait for TX to complete
  uart_wait_tx_done(dev->bus.uart_port, pdMS_TO_TICKS(100));

  // Switch to RX mode
  if (dev->bus.de_gpio != GPIO_NUM_NC) {
    gpio_set_level(dev->bus.de_gpio, 0);
  }
  if (dev->bus.re_gpio != GPIO_NUM_NC) {
    gpio_set_level(dev->bus.re_gpio, 0);
  }

  // Read response (echo of the request for write single register)
  uint8_t resp[8] = {0};
  int len = uart_read_bytes(dev->bus.uart_port, resp, sizeof(resp),
                            pdMS_TO_TICKS(1000));
  if (len != sizeof(resp)) {
    ESP_LOGW(TAG, "Calibration response size mismatch: got %d exp %d", len,
             (int)sizeof(resp));
    return ESP_FAIL;
  }

  // Verify CRC
  uint16_t rx_crc = ((uint16_t)resp[7] << 8) | resp[6];
  uint16_t calc_crc = modbus_rs485_crc16(resp, 6);
  if (rx_crc != calc_crc) {
    ESP_LOGW(TAG, "Calibration CRC error: rx=0x%04X calc=0x%04X", rx_crc,
             calc_crc);
    return ESP_FAIL;
  }

  ESP_LOGI(TAG, "Calibration successful with value %d", calib_value);
  return ESP_OK;
}

void sn3002_co2_polling_task(void *arg) {
  sn3002_co2_handle_t handle = (sn3002_co2_handle_t)arg;
  if (!handle) {
    ESP_LOGE(TAG, "polling_task got NULL handle");
    vTaskDelete(NULL);
    return;
  }

  ESP_LOGI(TAG, "SN3002 CO2 polling task started");

  while (1) {
    uint32_t co2_ppm;
    esp_err_t err = sn3002_co2_read(handle, &co2_ppm);
    if (err == ESP_OK) {
      ESP_LOGI(TAG, "SN3002 CO2 -> %lu ppm", (unsigned long)co2_ppm);
    } else {
      ESP_LOGW(TAG, "SN3002 CO2 read failed: %s", esp_err_to_name(err));
    }

    vTaskDelay(pdMS_TO_TICKS(5000));
  }
}
