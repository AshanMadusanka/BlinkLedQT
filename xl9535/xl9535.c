#include "xl9535.h"
#include "esp_log.h"

static const char *TAG = "XL9535";


/* ========= Internal helpers ========= */

static esp_err_t lock(xl9535_t *dev)
{
    if (!dev) return ESP_ERR_INVALID_ARG;
    if (dev->mutex) {
        if (xSemaphoreTake(dev->mutex, portMAX_DELAY) != pdTRUE) {
            return ESP_FAIL;
        }
    }
    return ESP_OK;
}

static void unlock(xl9535_t *dev)
{
    if (dev && dev->mutex) {
        xSemaphoreGive(dev->mutex);
    }
}

// Map pin -> port + bit index (same pattern as MCP driver)
static inline xl9535_port_t pin_to_port(xl9535_pin_t pin)
{
    return (pin <= XL9535_PIN_P0_7) ? XL9535_PORT_0 : XL9535_PORT_1;
}

static inline uint8_t pin_to_bit(xl9535_pin_t pin)
{
    if (pin <= XL9535_PIN_P0_7) {
        return (uint8_t)pin;            // 0..7
    } else {
        return (uint8_t)(pin - 8);      // 8..15 -> 0..7
    }
}

// Register select helpers
static inline uint8_t reg_input(xl9535_port_t port)
{
    return (port == XL9535_PORT_0) ? XL9535_REGISTER_INPUT_PORT_0
                                   : XL9535_REGISTER_INPUT_PORT_1;
}

static inline uint8_t reg_output(xl9535_port_t port)
{
    return (port == XL9535_PORT_0) ? XL9535_REGISTER_OUTPUT_PORT_0
                                   : XL9535_REGISTER_OUTPUT_PORT_1;
}

static inline uint8_t reg_config(xl9535_port_t port)
{
    return (port == XL9535_PORT_0) ? XL9535_REGISTER_CONFIG_0
                                   : XL9535_REGISTER_CONFIG_1;
}

static inline uint8_t reg_polarity(xl9535_port_t port)
{
    return (port == XL9535_PORT_0) ? XL9535_REGISTER_POLARITY_INV_0
                                   : XL9535_REGISTER_POLARITY_INV_1;
}

/* ========= Low-level register access ========= */

esp_err_t xl9535_write_register(xl9535_t *dev, uint8_t reg_addr, uint8_t value)
{
    if (!dev) return ESP_ERR_INVALID_ARG;

    esp_err_t err = lock(dev);
    if (err != ESP_OK) return err;

    uint8_t data[2] = { reg_addr, value };
    err = i2c_master_write_to_device(dev->i2c_port,
                                     dev->i2c_addr,
                                     data,
                                     sizeof(data),
                                     pdMS_TO_TICKS(1000));

    unlock(dev);
    return err;
}

esp_err_t xl9535_read_register(xl9535_t *dev, uint8_t reg_addr, uint8_t *value)
{
    if (!dev || !value) return ESP_ERR_INVALID_ARG;

    esp_err_t err = lock(dev);
    if (err != ESP_OK) return err;

    err = i2c_master_write_read_device(dev->i2c_port,
                                       dev->i2c_addr,
                                       &reg_addr,
                                       1,
                                       value,
                                       1,
                                       pdMS_TO_TICKS(1000));

    unlock(dev);
    return err;
}

/* ========= Public API ========= */

esp_err_t xl9535_init(xl9535_t *dev)
{
    if (!dev) {
        ESP_LOGE(TAG, "Device pointer is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    if (dev->mutex == NULL) {
        dev->mutex = xSemaphoreCreateMutex();
        if (!dev->mutex) {
            ESP_LOGE(TAG, "Failed to create mutex");
            return ESP_FAIL;
        }
    }

    // Optional sanity check: read CONFIG_0
    uint8_t dummy = 0;
    esp_err_t ret = xl9535_read_register(dev, XL9535_REGISTER_CONFIG_0, &dummy);
    if (ret == ESP_OK) {

        ESP_ERROR_CHECK(xl9535_write_port(dev, XL9535_PORT_0, 0x00));   // all OFF in latch
           // 2) Configure PORT0 as outputs in one go
        ESP_ERROR_CHECK(xl9535_write_register(dev, XL9535_REGISTER_CONFIG_0, 0x00)); // 0 = outputs
        ESP_LOGI(TAG, "XL9535 init OK (addr 0x%02X)", dev->i2c_addr);
    } else {
        ESP_LOGE(TAG, "XL9535 init: I2C read failed (addr 0x%02X)", dev->i2c_addr);
    }

    return ret;
}

esp_err_t xl9535_set_pin_dir(xl9535_t *dev,
                             xl9535_pin_t pin,
                             xl9535_pin_dir_t dir)
{
    if (!dev) return ESP_ERR_INVALID_ARG;

    xl9535_port_t port = pin_to_port(pin);
    uint8_t bit        = pin_to_bit(pin);

    uint8_t reg;
    esp_err_t err = xl9535_read_register(dev, reg_config(port), &reg);
    if (err != ESP_OK) return err;

    if (dir == XL9535_PIN_DIR_INPUT) {
        reg |= (1u << bit);     // 1 = input
    } else {
        reg &= ~(1u << bit);    // 0 = output
    }

    return xl9535_write_register(dev, reg_config(port), reg);
}

esp_err_t xl9535_read_pin(xl9535_t *dev,
                          xl9535_pin_t pin,
                          xl9535_pin_level_t *level)
{
    if (!dev || !level) return ESP_ERR_INVALID_ARG;

    xl9535_port_t port = pin_to_port(pin);
    uint8_t bit        = pin_to_bit(pin);

    uint8_t reg;
    esp_err_t err = xl9535_read_register(dev, reg_input(port), &reg);
    if (err != ESP_OK) return err;

    *level = (reg & (1u << bit)) ? XL9535_PIN_LEVEL_HIGH : XL9535_PIN_LEVEL_LOW;
    return ESP_OK;
}

esp_err_t xl9535_write_pin(xl9535_t *dev,
                           xl9535_pin_t pin,
                           xl9535_pin_level_t level)
{
    if (!dev) return ESP_ERR_INVALID_ARG;

    xl9535_port_t port = pin_to_port(pin);
    uint8_t bit        = pin_to_bit(pin);

    uint8_t reg;
    esp_err_t err = xl9535_read_register(dev, reg_output(port), &reg);
    if (err != ESP_OK) return err;

    if (level == XL9535_PIN_LEVEL_HIGH) {
        reg |= (1u << bit);
    } else {
        reg &= ~(1u << bit);
    }

    return xl9535_write_register(dev, reg_output(port), reg);
}

esp_err_t xl9535_read_port(xl9535_t *dev,
                           xl9535_port_t port,
                           uint8_t *value)
{
    if (!dev || !value) return ESP_ERR_INVALID_ARG;
    return xl9535_read_register(dev, reg_input(port), value);
}

esp_err_t xl9535_write_port(xl9535_t *dev,
                            xl9535_port_t port,
                            uint8_t value)
{
    if (!dev) return ESP_ERR_INVALID_ARG;
    return xl9535_write_register(dev, reg_output(port), value);
}

esp_err_t xl9535_write_port_masked(xl9535_t *dev,
                                   xl9535_port_t port,
                                   uint8_t mask,
                                   uint8_t val)
{
    if (!dev) return ESP_ERR_INVALID_ARG;

    uint8_t reg;
    esp_err_t err = xl9535_read_register(dev, reg_output(port), &reg);
    if (err != ESP_OK) return err;

    // Same pattern as MCP: keep bits outside mask
    reg = (uint8_t)((reg & ~mask) | (val & mask));

    return xl9535_write_register(dev, reg_output(port), reg);
}

esp_err_t xl9535_set_pin_polarity(xl9535_t *dev,
                                  xl9535_pin_t pin,
                                  bool invert)
{
    if (!dev) return ESP_ERR_INVALID_ARG;

    xl9535_port_t port = pin_to_port(pin);
    uint8_t bit        = pin_to_bit(pin);

    uint8_t reg;
    esp_err_t err = xl9535_read_register(dev, reg_polarity(port), &reg);
    if (err != ESP_OK) return err;

    if (invert) {
        reg |= (1u << bit);
    } else {
        reg &= ~(1u << bit);
    }

    return xl9535_write_register(dev, reg_polarity(port), reg);
}
