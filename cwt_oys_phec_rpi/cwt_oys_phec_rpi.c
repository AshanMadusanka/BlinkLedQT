#ifndef _DEFAULT_SOURCE
#define _DEFAULT_SOURCE
#endif

#include "cwt_oys_phec_rpi.h"

#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>

#define CWT_OYS_PHEC_RPI_REG_PH        0x0000u
#define CWT_OYS_PHEC_RPI_REG_EC        0x0001u
#define CWT_OYS_PHEC_RPI_REG_TEMP      0x0002u
#define CWT_OYS_PHEC_RPI_REG_SLAVE_ID  0x0030u
#define CWT_OYS_PHEC_RPI_REG_BAUD_RATE 0x0031u

#define CWT_OYS_PHEC_RPI_RETRIES    2
#define CWT_OYS_PHEC_RPI_TIMEOUT_MS 2000
#define CWT_OYS_PHEC_RPI_RETRY_US   (50 * 1000)

struct cwt_oys_phec_rpi_dev {
    modbus_rs485_rpi_bus_t *bus;
    uint8_t slave_id;
};

int cwt_oys_phec_rpi_create(const cwt_oys_phec_rpi_config_t *config,
                            cwt_oys_phec_rpi_handle_t *out_handle)
{
    struct cwt_oys_phec_rpi_dev *dev;

    if (config == NULL || out_handle == NULL || config->bus == NULL) {
        return CWT_OYS_PHEC_RPI_ERR_INVALID_ARG;
    }

    dev = (struct cwt_oys_phec_rpi_dev *)calloc(1, sizeof(*dev));
    if (dev == NULL) {
        return CWT_OYS_PHEC_RPI_ERR_NO_MEMORY;
    }

    dev->bus = config->bus;
    dev->slave_id = config->slave_id;

    *out_handle = dev;
    return CWT_OYS_PHEC_RPI_OK;
}

void cwt_oys_phec_rpi_destroy(cwt_oys_phec_rpi_handle_t handle)
{
    free(handle);
}

int cwt_oys_phec_rpi_set_slave_id(cwt_oys_phec_rpi_handle_t handle,
                                  uint8_t slave_id)
{
    if (handle == NULL) {
        return CWT_OYS_PHEC_RPI_ERR_INVALID_ARG;
    }

    handle->slave_id = slave_id;
    return CWT_OYS_PHEC_RPI_OK;
}

int cwt_oys_phec_rpi_read_all(cwt_oys_phec_rpi_handle_t handle,
                              float *ph,
                              float *ec_uScm,
                              float *temperature)
{
    uint8_t resp[16];
    uint16_t ph_raw;
    uint16_t ec_raw;
    uint16_t temperature_raw_u16;
    int16_t temperature_raw;

    if (handle == NULL || ph == NULL || ec_uScm == NULL || temperature == NULL) {
        return CWT_OYS_PHEC_RPI_ERR_INVALID_ARG;
    }

    for (int attempt = 0; attempt < CWT_OYS_PHEC_RPI_RETRIES; ++attempt) {
        int err = modbus_rs485_rpi_read_holding(handle->bus,
                                                handle->slave_id,
                                                CWT_OYS_PHEC_RPI_REG_PH,
                                                3,
                                                resp,
                                                sizeof(resp),
                                                CWT_OYS_PHEC_RPI_TIMEOUT_MS);
        if (err == MODBUS_RS485_RPI_OK) {
            err = modbus_rs485_rpi_get_u16(resp, sizeof(resp), 0, &ph_raw);
            if (err == MODBUS_RS485_RPI_OK) {
                err = modbus_rs485_rpi_get_u16(resp, sizeof(resp), 1, &ec_raw);
            }
            if (err == MODBUS_RS485_RPI_OK) {
                err = modbus_rs485_rpi_get_u16(resp,
                                               sizeof(resp),
                                               2,
                                               &temperature_raw_u16);
            }
            if (err == MODBUS_RS485_RPI_OK) {
                temperature_raw = (int16_t)temperature_raw_u16;
                *ph = (float)ph_raw / 100.0f;
                *ec_uScm = (float)ec_raw;
                *temperature = (float)temperature_raw / 10.0f;
                return CWT_OYS_PHEC_RPI_OK;
            }
        }

        if (attempt + 1 < CWT_OYS_PHEC_RPI_RETRIES) {
            usleep(CWT_OYS_PHEC_RPI_RETRY_US);
        }
    }

    return CWT_OYS_PHEC_RPI_ERR_MODBUS;
}

int cwt_oys_phec_rpi_read_ph(cwt_oys_phec_rpi_handle_t handle,
                             float *ph)
{
    float ec_uScm;
    float temperature;

    if (handle == NULL || ph == NULL) {
        return CWT_OYS_PHEC_RPI_ERR_INVALID_ARG;
    }

    return cwt_oys_phec_rpi_read_all(handle, ph, &ec_uScm, &temperature);
}

int cwt_oys_phec_rpi_read_ec(cwt_oys_phec_rpi_handle_t handle,
                             float *ec_uScm)
{
    float ph;
    float temperature;

    if (handle == NULL || ec_uScm == NULL) {
        return CWT_OYS_PHEC_RPI_ERR_INVALID_ARG;
    }

    return cwt_oys_phec_rpi_read_all(handle, &ph, ec_uScm, &temperature);
}

int cwt_oys_phec_rpi_read_temperature(cwt_oys_phec_rpi_handle_t handle,
                                      float *temperature)
{
    float ph;
    float ec_uScm;

    if (handle == NULL || temperature == NULL) {
        return CWT_OYS_PHEC_RPI_ERR_INVALID_ARG;
    }

    return cwt_oys_phec_rpi_read_all(handle, &ph, &ec_uScm, temperature);
}
