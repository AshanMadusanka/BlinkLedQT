#ifndef ADS1115_RPI_H
#define ADS1115_RPI_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ADS1115_RPI_OK                 0
#define ADS1115_RPI_ERR_INVALID_ARG   -1
#define ADS1115_RPI_ERR_OPEN          -2
#define ADS1115_RPI_ERR_IOCTL         -3
#define ADS1115_RPI_ERR_READ          -4
#define ADS1115_RPI_ERR_WRITE         -5
#define ADS1115_RPI_ERR_TIMEOUT       -6

#define ADS1115_RPI_REG_CONVERSION    0x00
#define ADS1115_RPI_REG_CONFIG        0x01
#define ADS1115_RPI_REG_LO_THRESH     0x02
#define ADS1115_RPI_REG_HI_THRESH     0x03

typedef enum {
    ADS1115_RPI_CHANNEL_A0 = 0,
    ADS1115_RPI_CHANNEL_A1,
    ADS1115_RPI_CHANNEL_A2,
    ADS1115_RPI_CHANNEL_A3
} ads1115_rpi_channel_t;

typedef enum {
    ADS1115_RPI_GAIN_6_144V = 0,
    ADS1115_RPI_GAIN_4_096V,
    ADS1115_RPI_GAIN_2_048V,
    ADS1115_RPI_GAIN_1_024V,
    ADS1115_RPI_GAIN_0_512V,
    ADS1115_RPI_GAIN_0_256V
} ads1115_rpi_gain_t;

typedef enum {
    ADS1115_RPI_DATA_RATE_8SPS = 0,
    ADS1115_RPI_DATA_RATE_16SPS,
    ADS1115_RPI_DATA_RATE_32SPS,
    ADS1115_RPI_DATA_RATE_64SPS,
    ADS1115_RPI_DATA_RATE_128SPS,
    ADS1115_RPI_DATA_RATE_250SPS,
    ADS1115_RPI_DATA_RATE_475SPS,
    ADS1115_RPI_DATA_RATE_860SPS
} ads1115_rpi_data_rate_t;

typedef struct {
    int fd;
    uint8_t i2c_addr;
    ads1115_rpi_gain_t gain;
    ads1115_rpi_data_rate_t data_rate;
} ads1115_rpi_t;

/*
 * Open a Linux I2C device, select the ADS1115 address, and set default
 * gain/data-rate settings. The usual Raspberry Pi 3 bus is /dev/i2c-1 and the
 * default ADS1115 address is 0x48.
 */
int ads1115_rpi_open(ads1115_rpi_t *dev,
                     const char *i2c_device,
                     uint8_t i2c_addr);

/* Close the Linux I2C file descriptor and reset the device handle. */
void ads1115_rpi_close(ads1115_rpi_t *dev);

/* Set the programmable gain/range used for later conversions. */
int ads1115_rpi_set_gain(ads1115_rpi_t *dev,
                         ads1115_rpi_gain_t gain);

/* Get the currently configured programmable gain/range. */
int ads1115_rpi_get_gain(ads1115_rpi_t *dev,
                         ads1115_rpi_gain_t *gain);

/* Set the data rate used for later single-shot conversions. */
int ads1115_rpi_set_data_rate(ads1115_rpi_t *dev,
                              ads1115_rpi_data_rate_t data_rate);

/* Get the currently configured data rate. */
int ads1115_rpi_get_data_rate(ads1115_rpi_t *dev,
                              ads1115_rpi_data_rate_t *data_rate);

/*
 * Read one raw single-ended ADC code from A0, A1, A2, or A3 using single-shot
 * mode. ADS1115 register values are signed 16-bit two's-complement values.
 */
int ads1115_rpi_read_raw(ads1115_rpi_t *dev,
                         ads1115_rpi_channel_t channel,
                         int16_t *raw);

/*
 * Read one single-ended voltage from A0, A1, A2, or A3.
 *
 * Safety note: even though the ADS1115 supports a +/-6.144 V measurement
 * range, analog input voltage must not exceed VDD. If the ADS1115 is powered
 * by 3.3 V, single-ended analog inputs must stay within 0 V to 3.3 V.
 */
int ads1115_rpi_read_voltage(ads1115_rpi_t *dev,
                             ads1115_rpi_channel_t channel,
                             float *voltage);

/* Return the full-scale voltage for a gain enum, or 0.0f for an invalid gain. */
float ads1115_rpi_gain_full_scale_voltage(ads1115_rpi_gain_t gain);

/* Write one 16-bit ADS1115 register value using big-endian register order. */
int ads1115_rpi_write_register(ads1115_rpi_t *dev,
                               uint8_t reg,
                               uint16_t value);

/* Read one 16-bit ADS1115 register value using big-endian register order. */
int ads1115_rpi_read_register(ads1115_rpi_t *dev,
                              uint8_t reg,
                              uint16_t *value);

#ifdef __cplusplus
}
#endif

#endif /* ADS1115_RPI_H */
