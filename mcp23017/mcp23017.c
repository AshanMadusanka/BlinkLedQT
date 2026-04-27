#include "mcp23017.h"
#include "esp_log.h"
#include "driver/i2c.h"

static const char *TAG = "MCP23017";

// MCP23017 register map (BANK=0)
#define MCP_REG_IODIRA   0x00
#define MCP_REG_IODIRB   0x01
#define MCP_REG_IPOLA    0x02
#define MCP_REG_IPOLB    0x03
#define MCP_REG_GPINTENA 0x04
#define MCP_REG_GPINTENB 0x05
#define MCP_REG_DEFVALA  0x06
#define MCP_REG_DEFVALB  0x07
#define MCP_REG_INTCONA  0x08
#define MCP_REG_INTCONB  0x09
#define MCP_REG_IOCON    0x0A  // also 0x0B (same)
#define MCP_REG_GPPUA    0x0C
#define MCP_REG_GPPUB    0x0D
#define MCP_REG_INTFA    0x0E
#define MCP_REG_INTFB    0x0F
#define MCP_REG_INTCAPA  0x10
#define MCP_REG_INTCAPB  0x11
#define MCP_REG_GPIOA    0x12
#define MCP_REG_GPIOB    0x13
#define MCP_REG_OLATA    0x14
#define MCP_REG_OLATB    0x15

// Helpers to map port to registers
static uint8_t reg_iodir(mcp23017_port_t port)
{
    return (port == MCP_PORT_A) ? MCP_REG_IODIRA : MCP_REG_IODIRB;
}

static uint8_t reg_gppu(mcp23017_port_t port)
{
    return (port == MCP_PORT_A) ? MCP_REG_GPPUA : MCP_REG_GPPUB;
}

static uint8_t reg_gpio(mcp23017_port_t port)
{
    return (port == MCP_PORT_A) ? MCP_REG_GPIOA : MCP_REG_GPIOB;
}

static uint8_t reg_olat(mcp23017_port_t port)
{
    return (port == MCP_PORT_A) ? MCP_REG_OLATA : MCP_REG_OLATB;
}

// Convert pin enum to port / bit index
static void pin_to_port_bit(mcp23017_pin_t pin,
                            mcp23017_port_t *port_out,
                            uint8_t *bit_out)
{
    if (pin <= MCP_PIN_A7) {
        *port_out = MCP_PORT_A;
        *bit_out  = (uint8_t)pin;          // 0–7
    } else {
        *port_out = MCP_PORT_B;
        *bit_out  = (uint8_t)(pin - MCP_PIN_B0);  // 0–7
    }
}

static esp_err_t lock(mcp23017_t *dev)
{
    if (dev->mutex) {
        if (xSemaphoreTake(dev->mutex, portMAX_DELAY) != pdTRUE) {
            return ESP_FAIL;
        }
    }
    return ESP_OK;
}

static void unlock(mcp23017_t *dev)
{
    if (dev->mutex) {
        xSemaphoreGive(dev->mutex);
    }
}

// Low-level I2C read/write
static esp_err_t mcp_write_reg(mcp23017_t *dev, uint8_t reg, uint8_t value)
{
    esp_err_t err = lock(dev);
    if (err != ESP_OK) return err;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd,
                          (dev->i2c_addr << 1) | I2C_MASTER_WRITE,
                          true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write_byte(cmd, value, true);
    i2c_master_stop(cmd);

    err = i2c_master_cmd_begin(dev->i2c_port, cmd,
                               pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);

    unlock(dev);
    return err;
}

static esp_err_t mcp_read_reg(mcp23017_t *dev, uint8_t reg, uint8_t *value)
{
    esp_err_t err = lock(dev);
    if (err != ESP_OK) return err;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd,
                          (dev->i2c_addr << 1) | I2C_MASTER_WRITE,
                          true);
    i2c_master_write_byte(cmd, reg, true);

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd,
                          (dev->i2c_addr << 1) | I2C_MASTER_READ,
                          true);
    i2c_master_read_byte(cmd, value, I2C_MASTER_NACK);
    i2c_master_stop(cmd);

    err = i2c_master_cmd_begin(dev->i2c_port, cmd,
                               pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);

    unlock(dev);
    return err;
}

// Read-modify-write helper for a single bit in a register
static esp_err_t mcp_update_reg_bit(mcp23017_t *dev,
                                    uint8_t reg,
                                    uint8_t bit,
                                    bool set)
{
    uint8_t val;
    esp_err_t err = mcp_read_reg(dev, reg, &val);
    if (err != ESP_OK) return err;

    if (set) {
        val |= (1u << bit);
    } else {
        val &= ~(1u << bit);
    }

    return mcp_write_reg(dev, reg, val);
}

// ================= PUBLIC API =================

esp_err_t mcp23017_init(mcp23017_t *dev)
{
    if (!dev) return ESP_ERR_INVALID_ARG;

    // Create mutex if user didn't provide one
    if (dev->mutex == NULL) {
        dev->mutex = xSemaphoreCreateMutex();
        if (!dev->mutex) {
            ESP_LOGE(TAG, "Failed to create mutex");
            return ESP_FAIL;
        }
    }

    // Optionally configure IOCON here (not strictly needed for defaults)
    // For safety you can clear both ports: all inputs, no pullups.
    esp_err_t err;
    err = mcp_write_reg(dev, MCP_REG_IODIRA, 0xFF); // all A inputs
    if (err != ESP_OK) return err;
    err = mcp_write_reg(dev, MCP_REG_IODIRB, 0xFF); // all B inputs
    if (err != ESP_OK) return err;
    err = mcp_write_reg(dev, MCP_REG_GPPUA, 0x00);  // pullups off
    if (err != ESP_OK) return err;
    err = mcp_write_reg(dev, MCP_REG_GPPUB, 0x00);  // pullups off
    if (err != ESP_OK) return err;

    return ESP_OK;
}

// ---------- Port-level functions ----------

esp_err_t mcp23017_set_port_dir(mcp23017_t *dev,
                                mcp23017_port_t port,
                                uint8_t dir_mask)
{
    return mcp_write_reg(dev, reg_iodir(port), dir_mask);
}

esp_err_t mcp23017_set_port_pullup(mcp23017_t *dev,
                                   mcp23017_port_t port,
                                   uint8_t pullup_mask)
{
    return mcp_write_reg(dev, reg_gppu(port), pullup_mask);
}

esp_err_t mcp23017_read_port(mcp23017_t *dev,
                             mcp23017_port_t port,
                             uint8_t *value_out)
{
    if (!value_out) return ESP_ERR_INVALID_ARG;
    return mcp_read_reg(dev, reg_gpio(port), value_out);
}

esp_err_t mcp23017_write_port(mcp23017_t *dev,
                              mcp23017_port_t port,
                              uint8_t value)
{
    // Writing to OLAT is recommended; but writing GPIO also works.
    return mcp_write_reg(dev, reg_olat(port), value);
}

// ---------- Pin-level functions ----------

esp_err_t mcp23017_set_pin_dir(mcp23017_t *dev,
                               mcp23017_pin_t pin,
                               mcp23017_pin_dir_t dir)
{
    mcp23017_port_t port;
    uint8_t bit;
    pin_to_port_bit(pin, &port, &bit);

    // IODIR: 1 = input, 0 = output
    bool set_bit = (dir == MCP_PIN_DIR_INPUT);
    return mcp_update_reg_bit(dev, reg_iodir(port), bit, set_bit);
}

esp_err_t mcp23017_enable_pullup(mcp23017_t *dev,
                                 mcp23017_pin_t pin,
                                 bool enable)
{
    mcp23017_port_t port;
    uint8_t bit;
    pin_to_port_bit(pin, &port, &bit);

    // GPPU: 1 = pull-up enabled
    return mcp_update_reg_bit(dev, reg_gppu(port), bit, enable);
}

esp_err_t mcp23017_write_pin(mcp23017_t *dev,
                             mcp23017_pin_t pin,
                             mcp23017_pin_level_t level)
{
    mcp23017_port_t port;
    uint8_t bit;
    pin_to_port_bit(pin, &port, &bit);

    uint8_t val;
    esp_err_t err = mcp_read_reg(dev, reg_olat(port), &val);
    if (err != ESP_OK) return err;

    if (level == MCP_PIN_LEVEL_HIGH) {
        val |= (1u << bit);
    } else {
        val &= ~(1u << bit);
    }

    return mcp_write_reg(dev, reg_olat(port), val);
}

esp_err_t mcp23017_read_pin(mcp23017_t *dev,
                            mcp23017_pin_t pin,
                            mcp23017_pin_level_t *level_out)
{
    if (!level_out) return ESP_ERR_INVALID_ARG;

    mcp23017_port_t port;
    uint8_t bit;
    pin_to_port_bit(pin, &port, &bit);

    uint8_t val;
    esp_err_t err = mcp_read_reg(dev, reg_gpio(port), &val);
    if (err != ESP_OK) return err;

    *level_out = (val & (1u << bit)) ? MCP_PIN_LEVEL_HIGH
                                     : MCP_PIN_LEVEL_LOW;
    return ESP_OK;
}
