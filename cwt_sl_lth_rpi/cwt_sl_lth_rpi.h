#ifndef CWT_SL_LTH_RPI_H
#define CWT_SL_LTH_RPI_H

#include <stdint.h>

#include "modbus_rs485_rpi/modbus_rs485_rpi.h"

#ifdef __cplusplus
extern "C" {
#endif

#define CWT_SL_LTH_RPI_OK               0
#define CWT_SL_LTH_RPI_ERR_INVALID_ARG -1
#define CWT_SL_LTH_RPI_ERR_NO_MEMORY   -2
#define CWT_SL_LTH_RPI_ERR_MODBUS      -3

typedef struct cwt_sl_lth_rpi_dev *cwt_sl_lth_rpi_handle_t;

typedef enum {
    CWT_SL_LTH_RPI_LUX_SIMPLE_1X = 0,
    CWT_SL_LTH_RPI_LUX_0_01X
} cwt_sl_lth_rpi_lux_mode_t;

typedef struct {
    modbus_rs485_rpi_bus_t *bus;
    uint8_t slave_id;
    cwt_sl_lth_rpi_lux_mode_t lux_mode;
} cwt_sl_lth_rpi_config_t;

int cwt_sl_lth_rpi_create(const cwt_sl_lth_rpi_config_t *config,
                          cwt_sl_lth_rpi_handle_t *out_handle);

void cwt_sl_lth_rpi_destroy(cwt_sl_lth_rpi_handle_t handle);

int cwt_sl_lth_rpi_set_slave_id(cwt_sl_lth_rpi_handle_t handle,
                                uint8_t slave_id);

int cwt_sl_lth_rpi_read_temp_hum(cwt_sl_lth_rpi_handle_t handle,
                                 float *temperature,
                                 float *humidity);

int cwt_sl_lth_rpi_read_lux(cwt_sl_lth_rpi_handle_t handle,
                            float *lux);

int cwt_sl_lth_rpi_read_all(cwt_sl_lth_rpi_handle_t handle,
                            float *temperature,
                            float *humidity,
                            float *lux);

#ifdef __cplusplus
}
#endif

#endif /* CWT_SL_LTH_RPI_H */
