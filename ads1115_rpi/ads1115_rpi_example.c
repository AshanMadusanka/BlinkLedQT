/*
 * Raspberry Pi setup:
 *
 *   sudo raspi-config
 *   # Interface Options > I2C > Enable
 *
 *   sudo apt update
 *   sudo apt install i2c-tools
 *
 *   i2cdetect -y 1
 *
 * Compile:
 *
 *   gcc -Wall -Wextra -pedantic ads1115_rpi_example.c ads1115_rpi.c -o ads1115_test
 *
 * Run:
 *
 *   sudo ./ads1115_test
 *
 * Raspberry Pi 3 wiring:
 *
 *   ADS1115 VDD  -> Raspberry Pi 3.3V
 *   ADS1115 GND  -> Raspberry Pi GND
 *   ADS1115 SCL  -> Raspberry Pi GPIO3 / SCL1 / physical pin 5
 *   ADS1115 SDA  -> Raspberry Pi GPIO2 / SDA1 / physical pin 3
 *   ADS1115 ADDR -> GND for address 0x48
 *
 * Safety note: even with the +/-6.144 V ADS1115 range selected, analog input
 * voltage must not exceed VDD. With 3.3 V power, keep single-ended inputs
 * within 0 V to 3.3 V.
 */

#include "ads1115_rpi.h"

#include <signal.h>
#include <stdio.h>
#include <unistd.h>

#define ADS1115_I2C_DEVICE "/dev/i2c-1"
#define ADS1115_I2C_ADDR   0x48

static volatile sig_atomic_t keep_running = 1;

static void handle_signal(int signum)
{
    (void)signum;
    keep_running = 0;
}

static const char *channel_name(ads1115_rpi_channel_t channel)
{
    switch (channel) {
    case ADS1115_RPI_CHANNEL_A0:
        return "A0";
    case ADS1115_RPI_CHANNEL_A1:
        return "A1";
    case ADS1115_RPI_CHANNEL_A2:
        return "A2";
    case ADS1115_RPI_CHANNEL_A3:
        return "A3";
    default:
        return "?";
    }
}

static int print_channel(ads1115_rpi_t *dev, ads1115_rpi_channel_t channel)
{
    int16_t raw = 0;
    ads1115_rpi_gain_t gain;
    float voltage = 0.0f;
    int err;

    err = ads1115_rpi_read_raw(dev, channel, &raw);
    if (err < 0) {
        fprintf(stderr, "%s: read raw failed: %d\n", channel_name(channel), err);
        return err;
    }

    err = ads1115_rpi_get_gain(dev, &gain);
    if (err < 0) {
        fprintf(stderr, "%s: get gain failed: %d\n", channel_name(channel), err);
        return err;
    }

    voltage = ((float)raw * ads1115_rpi_gain_full_scale_voltage(gain)) /
              32768.0f;

    printf("%s: raw=%d, voltage=%.3f V\n",
           channel_name(channel),
           raw,
           voltage);

    return ADS1115_RPI_OK;
}

int main(void)
{
    ads1115_rpi_t dev;
    int err;

    signal(SIGINT, handle_signal);
    signal(SIGTERM, handle_signal);

    err = ads1115_rpi_open(&dev, ADS1115_I2C_DEVICE, ADS1115_I2C_ADDR);
    if (err < 0) {
        fprintf(stderr,
                "Failed to open ADS1115 on %s at address 0x%02X: %d\n",
                ADS1115_I2C_DEVICE,
                ADS1115_I2C_ADDR,
                err);
        return 1;
    }

    err = ads1115_rpi_set_gain(&dev, ADS1115_RPI_GAIN_4_096V);
    if (err < 0) {
        fprintf(stderr, "Failed to set ADS1115 gain: %d\n", err);
        ads1115_rpi_close(&dev);
        return 1;
    }

    err = ads1115_rpi_set_data_rate(&dev, ADS1115_RPI_DATA_RATE_128SPS);
    if (err < 0) {
        fprintf(stderr, "Failed to set ADS1115 data rate: %d\n", err);
        ads1115_rpi_close(&dev);
        return 1;
    }

    while (keep_running) {
        (void)print_channel(&dev, ADS1115_RPI_CHANNEL_A0);
        (void)print_channel(&dev, ADS1115_RPI_CHANNEL_A1);
        (void)print_channel(&dev, ADS1115_RPI_CHANNEL_A2);
        (void)print_channel(&dev, ADS1115_RPI_CHANNEL_A3);
        printf("\n");
        sleep(1);
    }

    ads1115_rpi_close(&dev);
    return 0;
}
