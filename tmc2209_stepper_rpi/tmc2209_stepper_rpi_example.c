/*
 * Install libgpiod development files:
 *   sudo apt update
 *   sudo apt install libgpiod-dev gpiod
 *
 * Compile:
 *   gcc -Wall -Wextra -pedantic tmc2209_stepper_rpi_example.c tmc2209_stepper_rpi.c -o tmc2209_test -lgpiod -pthread
 *
 * Compile from this repository root:
 *   gcc -Wall -Wextra -pedantic -I. tmc2209_stepper_rpi/tmc2209_stepper_rpi_example.c tmc2209_stepper_rpi/tmc2209_stepper_rpi.c -o tmc2209_test -lgpiod -pthread
 *
 * Run:
 *   sudo ./tmc2209_test
 */

#include "tmc2209_stepper_rpi.h"

#include <stdio.h>
#include <unistd.h>

/*
 * Use BCM GPIO numbering, not physical header pin numbering.
 * This driver controls STEP only. DIR and EN are controlled manually by switches.
 * Linux GPIO timing is not real-time, so high step rates may have jitter.
 * For high-speed precision stepping, use a microcontroller or pigpio DMA-based
 * waveform generation instead of simple software toggling.
 */
#define TMC2209_STEP_GPIO 18
#define TMC2209_DUTY_PCT  50

int main(void)
{
    tmc2209_stepper_rpi_handle_t stepper = NULL;
    tmc2209_stepper_rpi_config_t cfg = {
        .step_gpio = TMC2209_STEP_GPIO,
        .duty_percent = TMC2209_DUTY_PCT,
    };
    int64_t count = 0;
    int err;

    err = tmc2209_stepper_rpi_init(&cfg, &stepper);
    if (err < 0) {
        fprintf(stderr, "Failed to initialize TMC2209 STEP driver: %d\n", err);
        return 1;
    }

    err = tmc2209_stepper_rpi_move_steps(stepper, 200, 500, 0);
    if (err < 0) {
        fprintf(stderr, "Move failed: %d\n", err);
        (void)tmc2209_stepper_rpi_deinit(stepper);
        return 1;
    }

    err = tmc2209_stepper_rpi_get_count(stepper, &count);
    if (err == TMC2209_STEPPER_RPI_OK) {
        printf("Step count after exact move: %lld\n", (long long)count);
    }

    sleep(1);

    err = tmc2209_stepper_rpi_start_continuous(stepper, 1000);
    if (err < 0) {
        fprintf(stderr, "Continuous start failed: %d\n", err);
        (void)tmc2209_stepper_rpi_deinit(stepper);
        return 1;
    }

    sleep(3);

    err = tmc2209_stepper_rpi_stop(stepper);
    if (err < 0) {
        fprintf(stderr, "Stop failed: %d\n", err);
        (void)tmc2209_stepper_rpi_deinit(stepper);
        return 1;
    }

    err = tmc2209_stepper_rpi_get_count(stepper, &count);
    if (err == TMC2209_STEPPER_RPI_OK) {
        printf("Step count after continuous run: %lld\n", (long long)count);
    }

    err = tmc2209_stepper_rpi_deinit(stepper);
    if (err < 0) {
        fprintf(stderr, "Deinit failed: %d\n", err);
        return 1;
    }

    return 0;
}
