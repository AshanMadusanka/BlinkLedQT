#ifndef TMC2209_STEPPER_H
#define TMC2209_STEPPER_H


#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/pcnt.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct tmc2209_stepper_dev *tmc2209_stepper_handle_t;

typedef struct {
    gpio_num_t step_gpio;              // STEP pin (your case: GPIO32)

    // LEDC (pulse generator)
    ledc_mode_t        ledc_speed_mode; // LEDC_HIGH_SPEED_MODE recommended
    ledc_timer_t       ledc_timer;
    ledc_channel_t     ledc_channel;

    // PCNT (pulse counter)
    pcnt_unit_t        pcnt_unit;
    pcnt_channel_t     pcnt_channel;

    // Duty for step pulse (0..100). 50% recommended.
    uint8_t            duty_percent;
} tmc2209_stepper_config_t;

/**
 * @brief Create/init step generator (LEDC) + pulse counter (PCNT).
 *        DIR/EN are not controlled here (you use manual switches).
 */
esp_err_t tmc2209_stepper_init(const tmc2209_stepper_config_t *cfg,
                               tmc2209_stepper_handle_t *out_handle);

/**
 * @brief Deinit (does not uninstall global LEDC/PCNT drivers, just frees handle).
 */
esp_err_t tmc2209_stepper_deinit(tmc2209_stepper_handle_t handle);

/**
 * @brief Move exact number of steps at a constant step frequency (blocking).
 *
 * @param steps       number of STEP pulses (rising edges) to output
 * @param step_hz     pulse frequency in Hz (e.g., 500, 1000, 2000)
 * @param timeout_ms  max time to wait; if 0 => compute automatically with margin
 */
esp_err_t tmc2209_stepper_move_steps(tmc2209_stepper_handle_t handle,
                                     uint32_t steps,
                                     uint32_t step_hz,
                                     uint32_t timeout_ms);

/**
 * @brief Start continuous stepping at step_hz (non-blocking).
 *        Use tmc2209_stepper_stop() to stop.
 */
esp_err_t tmc2209_stepper_start_continuous(tmc2209_stepper_handle_t handle,
                                           uint32_t step_hz);

/**
 * @brief Stop stepping (for both move and continuous).
 */
esp_err_t tmc2209_stepper_stop(tmc2209_stepper_handle_t handle);

/**
 * @brief Read how many pulses PCNT has counted since last reset.
 */
esp_err_t tmc2209_stepper_get_count(tmc2209_stepper_handle_t handle,
                                   int16_t *out_count);

/**
 * @brief Reset PCNT count to 0.
 */
esp_err_t tmc2209_stepper_reset_count(tmc2209_stepper_handle_t handle);

#ifdef __cplusplus
}
#endif

#endif // TMC2209_STEPPER_H
