#ifndef _DEFAULT_SOURCE
#define _DEFAULT_SOURCE
#endif

#include "cwt_sl_lth_rpi.h"

#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>

#define CWT_SL_LTH_RPI_REG_RH_TEMP 0x0000u
#define CWT_SL_LTH_RPI_REG_LUX     0x0006u
#define CWT_SL_LTH_RPI_RETRIES     2
#define CWT_SL_LTH_RPI_TIMEOUT_MS  2000
#define CWT_SL_LTH_RPI_RETRY_US    (50 * 1000)

struct cwt_sl_lth_rpi_dev {
    modbus_rs485_rpi_bus_t *bus;
    uint8_t slave_id;
    cwt_sl_lth_rpi_lux_mode_t lux_mode;
};

static int valid_lux_mode(cwt_sl_lth_rpi_lux_mode_t lux_mode)
{
    return lux_mode == CWT_SL_LTH_RPI_LUX_SIMPLE_1X ||
           lux_mode == CWT_SL_LTH_RPI_LUX_0_01X;
}

int cwt_sl_lth_rpi_create(const cwt_sl_lth_rpi_config_t *config,
                          cwt_sl_lth_rpi_handle_t *out_handle)
{
    struct cwt_sl_lth_rpi_dev *dev;

    if (config == NULL || out_handle == NULL || config->bus == NULL ||
        !valid_lux_mode(config->lux_mode)) {
        return CWT_SL_LTH_RPI_ERR_INVALID_ARG;
    }

    dev = (struct cwt_sl_lth_rpi_dev *)calloc(1, sizeof(*dev));
    if (dev == NULL) {
        return CWT_SL_LTH_RPI_ERR_NO_MEMORY;
    }

    dev->bus = config->bus;
    dev->slave_id = config->slave_id;
    dev->lux_mode = config->lux_mode;

    *out_handle = dev;
    return CWT_SL_LTH_RPI_OK;
}

void cwt_sl_lth_rpi_destroy(cwt_sl_lth_rpi_handle_t handle)
{
    free(handle);
}

int cwt_sl_lth_rpi_set_slave_id(cwt_sl_lth_rpi_handle_t handle,
                                uint8_t slave_id)
{
    if (handle == NULL) {
        return CWT_SL_LTH_RPI_ERR_INVALID_ARG;
    }

    handle->slave_id = slave_id;
    return CWT_SL_LTH_RPI_OK;
}

int cwt_sl_lth_rpi_read_temp_hum(cwt_sl_lth_rpi_handle_t handle,
                                 float *temperature,
                                 float *humidity)
{
    uint8_t resp[9];
    uint16_t humidity_raw;
    uint16_t temperature_raw_u16;
    int16_t temperature_raw;

    if (handle == NULL || temperature == NULL || humidity == NULL) {
        return CWT_SL_LTH_RPI_ERR_INVALID_ARG;
    }

    for (int attempt = 0; attempt < CWT_SL_LTH_RPI_RETRIES; ++attempt) {
        int err = modbus_rs485_rpi_read_holding(handle->bus,
                                                handle->slave_id,
                                                CWT_SL_LTH_RPI_REG_RH_TEMP,
                                                2,
                                                resp,
                                                sizeof(resp),
                                                CWT_SL_LTH_RPI_TIMEOUT_MS);
        if (err == MODBUS_RS485_RPI_OK) {
            err = modbus_rs485_rpi_get_u16(resp, sizeof(resp), 0, &humidity_raw);
            if (err == MODBUS_RS485_RPI_OK) {
                err = modbus_rs485_rpi_get_u16(resp,
                                               sizeof(resp),
                                               1,
                                               &temperature_raw_u16);
            }
            if (err == MODBUS_RS485_RPI_OK) {
                temperature_raw = (int16_t)temperature_raw_u16;
                *humidity = (float)humidity_raw / 10.0f;
                *temperature = (float)temperature_raw / 10.0f;
                return CWT_SL_LTH_RPI_OK;
            }
        }

        if (attempt + 1 < CWT_SL_LTH_RPI_RETRIES) {
            usleep(CWT_SL_LTH_RPI_RETRY_US);
        }
    }

    return CWT_SL_LTH_RPI_ERR_MODBUS;
}

int cwt_sl_lth_rpi_read_lux(cwt_sl_lth_rpi_handle_t handle,
                            float *lux)
{
    uint8_t resp[7];
    uint16_t lux_raw;

    if (handle == NULL || lux == NULL) {
        return CWT_SL_LTH_RPI_ERR_INVALID_ARG;
    }

    for (int attempt = 0; attempt < CWT_SL_LTH_RPI_RETRIES; ++attempt) {
        int err = modbus_rs485_rpi_read_holding(handle->bus,
                                                handle->slave_id,
                                                CWT_SL_LTH_RPI_REG_LUX,
                                                1,
                                                resp,
                                                sizeof(resp),
                                                CWT_SL_LTH_RPI_TIMEOUT_MS);
        if (err == MODBUS_RS485_RPI_OK) {
            err = modbus_rs485_rpi_get_u16(resp, sizeof(resp), 0, &lux_raw);
            if (err == MODBUS_RS485_RPI_OK) {
                if (handle->lux_mode == CWT_SL_LTH_RPI_LUX_SIMPLE_1X) {
                    *lux = (float)lux_raw;
                } else {
                    *lux = (float)lux_raw * 100.0f;
                }
                return CWT_SL_LTH_RPI_OK;
            }
        }

        if (attempt + 1 < CWT_SL_LTH_RPI_RETRIES) {
            usleep(CWT_SL_LTH_RPI_RETRY_US);
        }
    }

    return CWT_SL_LTH_RPI_ERR_MODBUS;
}

int cwt_sl_lth_rpi_read_all(cwt_sl_lth_rpi_handle_t handle,
                            float *temperature,
                            float *humidity,
                            float *lux)
{
    int err;

    if (handle == NULL || temperature == NULL || humidity == NULL || lux == NULL) {
        return CWT_SL_LTH_RPI_ERR_INVALID_ARG;
    }

    err = cwt_sl_lth_rpi_read_temp_hum(handle, temperature, humidity);
    if (err != CWT_SL_LTH_RPI_OK) {
        return err;
    }

    return cwt_sl_lth_rpi_read_lux(handle, lux);
}
