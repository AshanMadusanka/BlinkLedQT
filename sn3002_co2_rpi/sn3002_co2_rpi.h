#ifndef SN3002_CO2_RPI_H
#define SN3002_CO2_RPI_H

#include <stdint.h>

#include "modbus_rs485_rpi/modbus_rs485_rpi.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SN3002_CO2_RPI_OK              0
#define SN3002_CO2_RPI_ERR_INVALID_ARG -1
#define SN3002_CO2_RPI_ERR_NO_MEMORY   -2
#define SN3002_CO2_RPI_ERR_MODBUS      -3
#define SN3002_CO2_RPI_ERR_WRITE       -4
#define SN3002_CO2_RPI_ERR_READ        -5
#define SN3002_CO2_RPI_ERR_TIMEOUT     -6
#define SN3002_CO2_RPI_ERR_CRC         -7
#define SN3002_CO2_RPI_ERR_HEADER      -8

typedef enum {
    SN3002_CO2_RPI_BAUD_RATE_2400 = 0,
    SN3002_CO2_RPI_BAUD_RATE_4800 = 1,
    SN3002_CO2_RPI_BAUD_RATE_9600 = 2
} sn3002_co2_rpi_baud_rate_t;

typedef struct sn3002_co2_rpi_dev *sn3002_co2_rpi_handle_t;

typedef struct {
    modbus_rs485_rpi_bus_t *bus;
    uint8_t slave_id;
} sn3002_co2_rpi_config_t;

/* Create an SN3002 CO2 sensor handle using an already-open shared RS485 bus. */
int sn3002_co2_rpi_create(const sn3002_co2_rpi_config_t *config,
                          sn3002_co2_rpi_handle_t *out_handle);

/* Destroy a sensor handle allocated by sn3002_co2_rpi_create(). */
void sn3002_co2_rpi_destroy(sn3002_co2_rpi_handle_t handle);

/* Change only the local slave ID used by this handle; this does not write the sensor ID register. */
int sn3002_co2_rpi_set_slave_id(sn3002_co2_rpi_handle_t handle,
                                uint8_t slave_id);

/* Read CO2 concentration in ppm from holding register 0x0002. */
int sn3002_co2_rpi_read(sn3002_co2_rpi_handle_t handle,
                        uint32_t *co2_ppm);

/* Write a calibration value to the SN3002 calibration register. */
int sn3002_co2_rpi_calibrate(sn3002_co2_rpi_handle_t handle,
                             int calib_value);

#ifdef __cplusplus
}
#endif

#endif /* SN3002_CO2_RPI_H */
