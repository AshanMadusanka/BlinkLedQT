#ifndef FLOW_SENSOR_RPI_H
#define FLOW_SENSOR_RPI_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define FLOW_SENSOR_RPI_OK              0
#define FLOW_SENSOR_RPI_ERR_INVALID_ARG -1
#define FLOW_SENSOR_RPI_ERR_NO_MEMORY   -2
#define FLOW_SENSOR_RPI_ERR_GPIO        -3
#define FLOW_SENSOR_RPI_ERR_THREAD      -4
#define FLOW_SENSOR_RPI_ERR_NOT_FOUND   -5

#define FLOW_SENSOR_RPI_MAX_SENSORS 3
#define FLOW_SENSOR_RPI_DEFAULT_GPIOCHIP "/dev/gpiochip0"

typedef struct flow_sensor_rpi_dev *flow_sensor_rpi_handle_t;

typedef struct {
    int gpio;                    /* BCM GPIO number, not physical pin number. */
    uint8_t sensor_id;           /* User-defined sensor id. */
    float calibration_factor;    /* Hz per L/min, for example 7.5. */
    const char *gpiochip;        /* Normally "/dev/gpiochip0"; NULL uses default. */
} flow_sensor_rpi_config_t;

/* Initialize a flow sensor, request rising-edge GPIO events, and start counting. */
int flow_sensor_rpi_init(const flow_sensor_rpi_config_t *config,
                         flow_sensor_rpi_handle_t *out_handle);

/* Stop the worker thread, release GPIO resources, remove from global list, and free memory. */
int flow_sensor_rpi_deinit(flow_sensor_rpi_handle_t handle);

/* Return the latest calculated pulse frequency in Hz. */
int flow_sensor_rpi_get_frequency(flow_sensor_rpi_handle_t handle,
                                  float *frequency_hz);

/* Return the latest calculated flow rate in L/min. */
int flow_sensor_rpi_get_flow_rate_l_min(flow_sensor_rpi_handle_t handle,
                                        float *flow_rate_l_min);

/* Return total pulses counted since init or the most recent reset. */
int flow_sensor_rpi_get_total_pulses(flow_sensor_rpi_handle_t handle,
                                     uint64_t *total_pulses);

/* Reset the total pulse counter to zero. */
int flow_sensor_rpi_reset_total_pulses(flow_sensor_rpi_handle_t handle);

/* Find a sensor by id and return its latest frequency in Hz. */
int flow_sensor_rpi_get_frequency_by_sensor_id(uint8_t sensor_id,
                                               float *frequency_hz);

/* Find a sensor by id and return its latest flow rate in L/min. */
int flow_sensor_rpi_get_flow_rate_l_min_by_sensor_id(uint8_t sensor_id,
                                                     float *flow_rate_l_min);

/* Update the calibration factor used for future L/min calculations. */
int flow_sensor_rpi_set_calibration_factor(flow_sensor_rpi_handle_t handle,
                                           float calibration_factor);

/* Return the current calibration factor. */
int flow_sensor_rpi_get_calibration_factor(flow_sensor_rpi_handle_t handle,
                                           float *calibration_factor);

#ifdef __cplusplus
}
#endif

#endif /* FLOW_SENSOR_RPI_H */
