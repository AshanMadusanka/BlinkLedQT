/*
 * Install libgpiod development files:
 *   sudo apt update
 *   sudo apt install libgpiod-dev gpiod
 *
 * Compile:
 *   gcc -Wall -Wextra -pedantic flow_sensor_rpi_example.c flow_sensor_rpi.c -o flow_sensor_test -lgpiod -pthread
 *
 * Compile from this repository root:
 *   gcc -Wall -Wextra -pedantic -I. flow_sensor_rpi/flow_sensor_rpi_example.c flow_sensor_rpi/flow_sensor_rpi.c -o flow_sensor_test -lgpiod -pthread
 *
 * Run:
 *   sudo ./flow_sensor_test
 */

#include "flow_sensor_rpi.h"

#include <inttypes.h>
#include <signal.h>
#include <stdio.h>
#include <unistd.h>

/*
 * Use BCM GPIO numbering, not physical pin numbering.
 * Flow sensor signal wire goes to the selected GPIO.
 * Sensor GND must connect to Raspberry Pi GND.
 * Many flow sensors require 5V power, but their signal may be 5V.
 * Raspberry Pi GPIO is 3.3V only.
 * Use level shifting or a safe pull-up to 3.3V if sensor output is 5V.
 * Calibration factor depends on sensor model.
 * Common example: YF-S201 is often around 7.5 Hz per L/min, but final value
 * should be calibrated with your plumbing and sensor.
 */
#define FLOW_SENSOR_GPIO               17
#define FLOW_SENSOR_ID                 1
#define FLOW_SENSOR_CALIBRATION_FACTOR 7.5f

static volatile sig_atomic_t keep_running = 1;

static void handle_signal(int signum)
{
    (void)signum;
    keep_running = 0;
}

int main(void)
{
    flow_sensor_rpi_handle_t sensor = NULL;
    flow_sensor_rpi_config_t config = {
        .gpio = FLOW_SENSOR_GPIO,
        .sensor_id = FLOW_SENSOR_ID,
        .calibration_factor = FLOW_SENSOR_CALIBRATION_FACTOR,
        .gpiochip = FLOW_SENSOR_RPI_DEFAULT_GPIOCHIP,
    };
    int err;

    signal(SIGINT, handle_signal);
    signal(SIGTERM, handle_signal);

    err = flow_sensor_rpi_init(&config, &sensor);
    if (err < 0) {
        fprintf(stderr, "Failed to initialize flow sensor: %d\n", err);
        return 1;
    }

    while (keep_running) {
        float frequency_hz = 0.0f;
        float flow_rate_l_min = 0.0f;
        uint64_t total_pulses = 0;

        sleep(1);

        err = flow_sensor_rpi_get_frequency(sensor, &frequency_hz);
        if (err < 0) {
            fprintf(stderr, "Failed to read frequency: %d\n", err);
            break;
        }

        err = flow_sensor_rpi_get_flow_rate_l_min(sensor, &flow_rate_l_min);
        if (err < 0) {
            fprintf(stderr, "Failed to read flow rate: %d\n", err);
            break;
        }

        err = flow_sensor_rpi_get_total_pulses(sensor, &total_pulses);
        if (err < 0) {
            fprintf(stderr, "Failed to read total pulses: %d\n", err);
            break;
        }

        printf("Frequency: %.2f Hz\n", frequency_hz);
        printf("Flow rate: %.2f L/min\n", flow_rate_l_min);
        printf("Total pulses: %" PRIu64 "\n\n", total_pulses);
    }

    (void)flow_sensor_rpi_deinit(sensor);
    return 0;
}
