#ifndef CWT_OYS_PHEC_RPI_H
#define CWT_OYS_PHEC_RPI_H

#include <stdint.h>

#include "modbus_rs485_rpi/modbus_rs485_rpi.h"

#ifdef __cplusplus
extern "C" {
#endif

#define CWT_OYS_PHEC_RPI_OK               0
#define CWT_OYS_PHEC_RPI_ERR_INVALID_ARG -1
#define CWT_OYS_PHEC_RPI_ERR_NO_MEMORY   -2
#define CWT_OYS_PHEC_RPI_ERR_MODBUS      -3

typedef struct cwt_oys_phec_rpi_dev *cwt_oys_phec_rpi_handle_t;

typedef struct {
    modbus_rs485_rpi_bus_t *bus;
    uint8_t slave_id;
} cwt_oys_phec_rpi_config_t;

/* Create a CWT-OYS-PHEC sensor handle using an already-open shared RS485 bus. */
int cwt_oys_phec_rpi_create(const cwt_oys_phec_rpi_config_t *config,
                            cwt_oys_phec_rpi_handle_t *out_handle);

/* Destroy a sensor handle allocated by cwt_oys_phec_rpi_create(). */
void cwt_oys_phec_rpi_destroy(cwt_oys_phec_rpi_handle_t handle);

/* Change only the local slave ID used by this handle; this does not write the sensor ID register. */
int cwt_oys_phec_rpi_set_slave_id(cwt_oys_phec_rpi_handle_t handle,
                                  uint8_t slave_id);

/* Read pH, EC, and temperature in one Modbus holding-register request. */
int cwt_oys_phec_rpi_read_all(cwt_oys_phec_rpi_handle_t handle,
                              float *ph,
                              float *ec_uScm,
                              float *temperature);

/* Read only pH. */
int cwt_oys_phec_rpi_read_ph(cwt_oys_phec_rpi_handle_t handle,
                             float *ph);

/* Read only electrical conductivity in uS/cm. */
int cwt_oys_phec_rpi_read_ec(cwt_oys_phec_rpi_handle_t handle,
                             float *ec_uScm);

/* Read only temperature in degrees C. */
int cwt_oys_phec_rpi_read_temperature(cwt_oys_phec_rpi_handle_t handle,
                                      float *temperature);

#ifdef __cplusplus
}
#endif

#endif /* CWT_OYS_PHEC_RPI_H */
