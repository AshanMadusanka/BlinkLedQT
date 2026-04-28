#ifndef FLOW_SENSOR_H
#define FLOW_SENSOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
#include "esp_log.h"

#define MAX_FLOW_SENSORS 3

// Struct to hold configuration parameters for the flow sensor
typedef struct {
    uint8_t flow_sensor_pin;        // GPIO pin for sensor input
    float pulse_to_liters_factor;   // Conversion factor: pulses per liter/min
	uint8_t sensor_id;              // Id of the sensor
    volatile uint32_t pulse_count;  // To store pulse count from ISR
    uint32_t flow_frequency;        // Flow frequency in Hz (pulses per second)
    TimerHandle_t flow_timer;       // FreeRTOS timer handle
} flow_sensor_handle_t;

// Function to initialize the flow sensor (sets up GPIO, ISR, and FreeRTOS timer)
esp_err_t flow_sensor_init(flow_sensor_handle_t *config);

// Function to get the current flow frequency in pulses per second (Hz) using sensor id
uint32_t get_flow_frequency_by_sensor_id(uint8_t sensor_id);

// Function to get the current flow frequency in pulses per second (Hz)
uint32_t get_flow_frequency(flow_sensor_handle_t *config);

#ifdef __cplusplus
}
#endif

#endif  // FLOW_SENSOR_H