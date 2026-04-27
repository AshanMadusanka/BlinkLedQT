#include "cwt_oys_phec.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "modbus_rs485.h"
#include <stdlib.h>

static const char *TAG = "CWT_OYS_PHEC";

// Registers (from datasheet)
#define CWT_OYS_PHEC_REG_PH 0x0000   // 0.01 pH units
#define CWT_OYS_PHEC_REG_EC 0x0001   // 1 µS/cm
#define CWT_OYS_PHEC_REG_TEMP 0x0002 // 0.1 °C
#define CWT_OYS_PHEC_REG_SLAVE_ID 0x0030
#define CWT_OYS_PHEC_REG_BAUD_RATE 0x0031

typedef struct cwt_oys_phec_dev {
  cwt_oys_phec_config_t cfg;
  modbus_rs485_bus_t bus;
} cwt_oys_phec_dev_t;

// ---- Public API ----
esp_err_t cwt_oys_phec_create(const cwt_oys_phec_config_t *config,
                              cwt_oys_phec_handle_t *out_handle) {
  if (!config || !out_handle) {
    return ESP_ERR_INVALID_ARG;
  }

  cwt_oys_phec_dev_t *dev = calloc(1, sizeof(cwt_oys_phec_dev_t));
  if (!dev)
    return ESP_ERR_NO_MEM;

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
    gpio_set_level(config->de_gpio, 0); // default RX
  }
  if (config->re_gpio != GPIO_NUM_NC) {
    gpio_reset_pin(config->re_gpio);
    gpio_set_direction(config->re_gpio, GPIO_MODE_OUTPUT);
    gpio_set_level(config->re_gpio, 0); // default RX
  }

  *out_handle = (cwt_oys_phec_handle_t)dev;
  ESP_LOGI(TAG, "CWT-OYS-PHEC created on UART%d, slave_id=%d",
           config->uart_port, config->slave_id);
  return ESP_OK;
}

esp_err_t cwt_oys_phec_destroy(cwt_oys_phec_handle_t handle) {
  if (!handle)
    return ESP_ERR_INVALID_ARG;
  cwt_oys_phec_dev_t *dev = (cwt_oys_phec_dev_t *)handle;

  free(dev);
  return ESP_OK;
}

esp_err_t cwt_oys_phec_set_slave_id(cwt_oys_phec_handle_t handle,
                                    uint8_t slave_id) {
  if (!handle)
    return ESP_ERR_INVALID_ARG;
  cwt_oys_phec_dev_t *dev = (cwt_oys_phec_dev_t *)handle;
  dev->cfg.slave_id = slave_id;
  return ESP_OK;
}

esp_err_t cwt_oys_phec_read_all(cwt_oys_phec_handle_t handle, float *ph,
                                float *ec_uScm, float *temperature) {
  if (!handle || !ph || !ec_uScm || !temperature) {
    return ESP_ERR_INVALID_ARG;
  }

  cwt_oys_phec_dev_t *dev = (cwt_oys_phec_dev_t *)handle;

  uint8_t buf[16] = {0};

  for (int attempt = 0; attempt < 2; ++attempt) {
    esp_err_t ret = modbus_rs485_read_holding(&dev->bus, dev->cfg.slave_id,
                                              CWT_OYS_PHEC_REG_PH, 3, buf,
                                              sizeof(buf), 2000);
    if (ret == ESP_OK) {
      uint16_t ph_raw = ((uint16_t)buf[3] << 8) | buf[4];
      uint16_t ec_raw = ((uint16_t)buf[5] << 8) | buf[6];
      int16_t t_raw = (int16_t)(((uint16_t)buf[7] << 8) | buf[8]);

      *ph = ph_raw / 100.0f;
      *ec_uScm = (float)ec_raw;
      *temperature = t_raw / 10.0f;
      return ESP_OK;
    }

    // small delay before retry
    vTaskDelay(pdMS_TO_TICKS(50));
  }

  return ESP_FAIL;
}

esp_err_t cwt_oys_phec_read_ph(cwt_oys_phec_handle_t handle, float *ph) {
  if (!handle || !ph)
    return ESP_ERR_INVALID_ARG;

  float dummy_ec, dummy_temp;
  return cwt_oys_phec_read_all(handle, ph, &dummy_ec, &dummy_temp);
}

esp_err_t cwt_oys_phec_read_ec(cwt_oys_phec_handle_t handle, float *ec_uScm) {
  if (!handle || !ec_uScm)
    return ESP_ERR_INVALID_ARG;

  float dummy_ph, dummy_temp;
  return cwt_oys_phec_read_all(handle, &dummy_ph, ec_uScm, &dummy_temp);
}

esp_err_t cwt_oys_phec_read_temperature(cwt_oys_phec_handle_t handle,
                                        float *temperature) {
  if (!handle || !temperature)
    return ESP_ERR_INVALID_ARG;

  float dummy_ph, dummy_ec;
  return cwt_oys_phec_read_all(handle, &dummy_ph, &dummy_ec, temperature);
}

#if 0
void cwt_oys_phec_polling_task(void *arg) {
  cwt_oys_phec_handle_t handle = (cwt_oys_phec_handle_t)arg;
  if (!handle) {
    ESP_LOGE(TAG, "polling_task got NULL handle");
    vTaskDelete(NULL);
    return;
  }

  ESP_LOGI(TAG, "CWT-OYS-PHEC polling task started");

  while (1) {
    float ph, ec, t;
    esp_err_t err = cwt_oys_phec_read_all(handle, &ph, &ec, &t);
    if (err == ESP_OK) {
      ESP_LOGI(TAG, "CWT-OYS-PHEC -> pH=%.2f, EC=%.0f uS/cm, T=%.1f C", ph, ec,
               t);
    } else {
      ESP_LOGW(TAG, "CWT-OYS-PHEC read failed: %s", esp_err_to_name(err));
    }

    vTaskDelay(pdMS_TO_TICKS(5000));
  }
}

#endif