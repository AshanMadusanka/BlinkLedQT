/*
 * Compile when both XL9535 files are in one flat directory:
 *   gcc -Wall -Wextra -pedantic xl9535_rpi_example.c xl9535_rpi.c -o xl9535_test -pthread
 *
 * Compile from this repository root:
 *   gcc -Wall -Wextra -pedantic -I. xl9535_rpi/xl9535_rpi_example.c xl9535_rpi/xl9535_rpi.c -o xl9535_test -pthread
 *
 * Run:
 *   sudo ./xl9535_test
 */

#include "xl9535_rpi.h"

#include <signal.h>
#include <stdio.h>
#include <unistd.h>

/*
 * Enable I2C using:
 *   sudo raspi-config
 *
 * Check the device address using:
 *   i2cdetect -y 1
 *
 * /dev/i2c-1 is normally used on Raspberry Pi 3.
 * 0 in an XL9535 CONFIG register means output.
 * 1 in an XL9535 CONFIG register means input.
 */
#define XL9535_I2C_DEVICE "/dev/i2c-1"
#define XL9535_I2C_ADDR   0x20

static volatile sig_atomic_t keep_running = 1;

static void handle_signal(int signum)
{
    (void)signum;
    keep_running = 0;
}

int main(void)
{
    xl9535_rpi_t dev;
    int err;

    signal(SIGINT, handle_signal);
    signal(SIGTERM, handle_signal);

    err = xl9535_rpi_open(&dev, XL9535_I2C_DEVICE, XL9535_I2C_ADDR);
    if (err < 0) {
        fprintf(stderr,
                "Failed to open XL9535 on %s at address 0x%02X: %d\n",
                XL9535_I2C_DEVICE,
                XL9535_I2C_ADDR,
                err);
        return 1;
    }

    err = xl9535_rpi_set_pin_dir(&dev,
                                 XL9535_RPI_PIN_P0_0,
                                 XL9535_RPI_PIN_DIR_OUTPUT);
    if (err < 0) {
        fprintf(stderr, "Failed to configure P0_0 as output: %d\n", err);
        xl9535_rpi_close(&dev);
        return 1;
    }

    err = xl9535_rpi_set_pin_dir(&dev,
                                 XL9535_RPI_PIN_P1_0,
                                 XL9535_RPI_PIN_DIR_INPUT);
    if (err < 0) {
        fprintf(stderr, "Failed to configure P1_0 as input: %d\n", err);
        xl9535_rpi_close(&dev);
        return 1;
    }

    while (keep_running) {
        xl9535_rpi_pin_level_t input_level;

        (void)xl9535_rpi_write_pin(&dev,
                                   XL9535_RPI_PIN_P0_0,
                                   XL9535_RPI_PIN_LEVEL_HIGH);
        usleep(500 * 1000);

        err = xl9535_rpi_read_pin(&dev, XL9535_RPI_PIN_P1_0, &input_level);
        if (err == XL9535_RPI_OK) {
            printf("P1_0: %s\n",
                   input_level == XL9535_RPI_PIN_LEVEL_HIGH ? "HIGH" : "LOW");
        } else {
            fprintf(stderr, "Failed to read P1_0: %d\n", err);
        }

        (void)xl9535_rpi_write_pin(&dev,
                                   XL9535_RPI_PIN_P0_0,
                                   XL9535_RPI_PIN_LEVEL_LOW);
        usleep(500 * 1000);
    }

    (void)xl9535_rpi_write_pin(&dev,
                               XL9535_RPI_PIN_P0_0,
                               XL9535_RPI_PIN_LEVEL_LOW);
    xl9535_rpi_close(&dev);

    return 0;
}
