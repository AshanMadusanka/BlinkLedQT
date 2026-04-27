#include "cwt_sl_lth.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "modbus_rs485.h"
#include <stdlib.h>

static const char *TAG = "CWT_SL_LTH";

// Registers (from datasheet)
#define CWT_SL_LTH_REG_RH_TEMP 0x0000 // 2 regs: RH, Temp (0.1 units)
#define CWT_SL_LTH_REG_LUX 0x0006     // 1 reg: Lux

typedef struct cwt_sl_lth_dev {
  cwt_sl_lth_config_t cfg;
  modbus_rs485_bus_t bus;
} cwt_sl_lth_dev_t;

// ========= Public API =========

esp_err_t cwt_sl_lth_create(const cwt_sl_lth_config_t *config,
                            cwt_sl_lth_handle_t *out_handle) {
  if (!config || !out_handle) {
    return ESP_ERR_INVALID_ARG;
  }

  cwt_sl_lth_dev_t *dev = calloc(1, sizeof(cwt_sl_lth_dev_t));
  if (!dev) {
    return ESP_ERR_NO_MEM;
  }

  dev->cfg = *config; // copy config

  // Setup bus configuration for shared Modbus layer
  dev->bus.uart_port = config->uart_port;
  dev->bus.de_gpio = config->de_gpio;
  dev->bus.re_gpio = config->re_gpio;

  // Conditionally initialize UART
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

  // Handle DE/RE lines if given
  if (config->de_gpio != GPIO_NUM_NC) {
    gpio_reset_pin(config->de_gpio);
    gpio_set_direction(config->de_gpio, GPIO_MODE_OUTPUT);
    gpio_set_level(config->de_gpio, 0); // default RX mode
  }
  if (config->re_gpio != GPIO_NUM_NC) {
    gpio_reset_pin(config->re_gpio);
    gpio_set_direction(config->re_gpio, GPIO_MODE_OUTPUT);
    gpio_set_level(config->re_gpio, 0); // default RX enable
  }

  *out_handle = (cwt_sl_lth_handle_t)dev;
  ESP_LOGI(TAG, "CWT-SL-LTH created on UART%d, slave_id=%d", config->uart_port,
           config->slave_id);
  return ESP_OK;
}

esp_err_t cwt_sl_lth_destroy(cwt_sl_lth_handle_t handle) {
  if (!handle)
    return ESP_ERR_INVALID_ARG;
  cwt_sl_lth_dev_t *dev = (cwt_sl_lth_dev_t *)handle;

  // We do NOT uninstall UART here (other devices may share it).
  free(dev);
  return ESP_OK;
}

esp_err_t cwt_sl_lth_set_slave_id(cwt_sl_lth_handle_t handle,
                                  uint8_t slave_id) {
  if (!handle)
    return ESP_ERR_INVALID_ARG;
  cwt_sl_lth_dev_t *dev = (cwt_sl_lth_dev_t *)handle;
  dev->cfg.slave_id = slave_id;
  return ESP_OK;
}

esp_err_t cwt_sl_lth_read_temp_hum(cwt_sl_lth_handle_t handle,
                                   float *temperature, float *humidity) {
  if (!handle || !temperature || !humidity) {
    return ESP_ERR_INVALID_ARG;
  }

  cwt_sl_lth_dev_t *dev = (cwt_sl_lth_dev_t *)handle;

  uint8_t buf[16] = {0};

  // Retry mechanism for robustness
  for (int attempt = 0; attempt < 2; ++attempt) {
    esp_err_t ret = modbus_rs485_read_holding(&dev->bus, dev->cfg.slave_id,
                                              CWT_SL_LTH_REG_RH_TEMP, 2, buf,
                                              sizeof(buf), 2000);
    if (ret == ESP_OK) {
      // buf[0] = addr, [1] = func, [2] = byte_count (0x04), then data
      uint16_t rh_raw = (uint16_t)buf[3] << 8 | buf[4];
      int16_t t_raw = (int16_t)((uint16_t)buf[5] << 8 | buf[6]);

      *humidity = rh_raw / 10.0f;   // 0.1 %RH
      *temperature = t_raw / 10.0f; // 0.1 °C, signed
      return ESP_OK;
    }

    // Small delay before retry
    vTaskDelay(pdMS_TO_TICKS(50));
  }

  return ESP_FAIL;
}

esp_err_t cwt_sl_lth_read_lux(cwt_sl_lth_handle_t handle, float *lux) {
  if (!handle || !lux)
    return ESP_ERR_INVALID_ARG;

  cwt_sl_lth_dev_t *dev = (cwt_sl_lth_dev_t *)handle;

  uint8_t buf[16] = {0};

  // Retry mechanism for robustness
  for (int attempt = 0; attempt < 2; ++attempt) {
    esp_err_t ret =
        modbus_rs485_read_holding(&dev->bus, dev->cfg.slave_id,
                                  CWT_SL_LTH_REG_LUX, 1, buf, sizeof(buf), 2000);
    if (ret == ESP_OK) {
      uint16_t lux_raw = (uint16_t)buf[3] << 8 | buf[4];

      if (dev->cfg.lux_mode == CWT_SL_LTH_LUX_SIMPLE_1X) {
        *lux = (float)lux_raw;
      } else {
        *lux = (float)lux_raw * 100.0f; // 0.01 units
      }
      return ESP_OK;
    }

    // Small delay before retry
    vTaskDelay(pdMS_TO_TICKS(50));
  }

  return ESP_FAIL;
}

esp_err_t cwt_sl_lth_read_all(cwt_sl_lth_handle_t handle, float *temperature,
                              float *humidity, float *lux) {
  if (!handle || !temperature || !humidity || !lux) {
    return ESP_ERR_INVALID_ARG;
  }

  esp_err_t rt = cwt_sl_lth_read_temp_hum(handle, temperature, humidity);
  esp_err_t rl = cwt_sl_lth_read_lux(handle, lux);

  if (rt != ESP_OK)
    return rt;
  if (rl != ESP_OK)
    return rl;
  return ESP_OK;
}

#if 0
// Optional demo polling task: prints values every 5 seconds
void cwt_sl_lth_polling_task(void *arg) {
  cwt_sl_lth_handle_t handle = (cwt_sl_lth_handle_t)arg;

  if (!handle) {
    ESP_LOGE(TAG, "polling_task got NULL handle");
    vTaskDelete(NULL);
    return;
  }

  ESP_LOGI(TAG, "CWT-SL-LTH polling task started");

  while (1) {
    float t, h, lux;
    esp_err_t err = cwt_sl_lth_read_all(handle, &t, &h, &lux);
    if (err == ESP_OK) {
      ESP_LOGI(TAG, "-> T=%.1f C, RH=%.1f %%, Lux=%.0f", t, h, lux);
    } else {
      ESP_LOGW(TAG, "CWT-SL-LTH read failed: %s", esp_err_to_name(err));
    }
    vTaskDelay(pdMS_TO_TICKS(5000));
  }
}

#endif