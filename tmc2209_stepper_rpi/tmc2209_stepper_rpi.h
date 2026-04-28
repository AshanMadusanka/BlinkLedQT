#ifndef TMC2209_STEPPER_RPI_H
#define TMC2209_STEPPER_RPI_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define TMC2209_STEPPER_RPI_OK              0
#define TMC2209_STEPPER_RPI_ERR_INVALID_ARG -1
#define TMC2209_STEPPER_RPI_ERR_NO_MEMORY   -2
#define TMC2209_STEPPER_RPI_ERR_GPIO        -3
#define TMC2209_STEPPER_RPI_ERR_BUSY        -4
#define TMC2209_STEPPER_RPI_ERR_THREAD      -5

typedef struct tmc2209_stepper_rpi_dev *tmc2209_stepper_rpi_handle_t;

typedef struct {
    int step_gpio;          /* BCM GPIO number for STEP pin. */
    uint8_t duty_percent;   /* 1 to 90; 50 is recommended. */
} tmc2209_stepper_rpi_config_t;

/*
 * Initialize the STEP pulse driver using libgpiod on gpiochip0.
 * DIR and EN are intentionally not controlled by this driver.
 */
int tmc2209_stepper_rpi_init(const tmc2209_stepper_rpi_config_t *cfg,
                             tmc2209_stepper_rpi_handle_t *out_handle);

/* Stop any active stepping, release GPIO resources, and free the handle. */
int tmc2209_stepper_rpi_deinit(tmc2209_stepper_rpi_handle_t handle);

/*
 * Blocking exact-step move. One step is one STEP rising edge.
 * Linux software GPIO timing is not real-time; high step rates may jitter.
 */
int tmc2209_stepper_rpi_move_steps(tmc2209_stepper_rpi_handle_t handle,
                                   uint32_t steps,
                                   uint32_t step_hz,
                                   uint32_t timeout_ms);

/* Start a non-blocking worker thread that continuously toggles STEP. */
int tmc2209_stepper_rpi_start_continuous(tmc2209_stepper_rpi_handle_t handle,
                                         uint32_t step_hz);

/* Stop continuous stepping if active and force STEP low. */
int tmc2209_stepper_rpi_stop(tmc2209_stepper_rpi_handle_t handle);

/* Return the software pulse count, incremented on every STEP rising edge. */
int tmc2209_stepper_rpi_get_count(tmc2209_stepper_rpi_handle_t handle,
                                  int64_t *out_count);

/* Reset the software pulse count to zero. */
int tmc2209_stepper_rpi_reset_count(tmc2209_stepper_rpi_handle_t handle);

#ifdef __cplusplus
}
#endif

#endif /* TMC2209_STEPPER_RPI_H */
