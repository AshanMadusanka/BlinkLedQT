#include "tmc2209_stepper.h"
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "TMC2209_STEP";

struct tmc2209_stepper_dev {
    tmc2209_stepper_config_t cfg;
    bool running;
};

static esp_err_t ledc_set_step_freq(const tmc2209_stepper_config_t *c, uint32_t step_hz)
{
    if (step_hz == 0) return ESP_ERR_INVALID_ARG;

    // Use a fixed resolution that works well for typical step rates.
    // 10-bit gives 0..1023 duty levels.
    ledc_timer_config_t tcfg = {
        .speed_mode       = c->ledc_speed_mode,
        .timer_num        = c->ledc_timer,
        .duty_resolution  = LEDC_TIMER_10_BIT,
        .freq_hz          = (uint32_t)step_hz,
        .clk_cfg          = LEDC_AUTO_CLK,
    };
    esp_err_t err = ledc_timer_config(&tcfg);
    return err;
}

static esp_err_t step_ledc_start(const tmc2209_stepper_config_t *c)
{
    uint32_t max_duty = (1u << LEDC_TIMER_10_BIT) - 1u;
    uint32_t duty = (max_duty * (uint32_t)c->duty_percent) / 100u;

    ledc_channel_config_t ch = {
        .gpio_num   = c->step_gpio,
        .speed_mode = c->ledc_speed_mode,
        .channel    = c->ledc_channel,
        .intr_type  = LEDC_INTR_DISABLE,
        .timer_sel  = c->ledc_timer,
        .duty       = duty,
        .hpoint     = 0,
    };
    esp_err_t err = ledc_channel_config(&ch);
    if (err != ESP_OK) return err;

    err = ledc_update_duty(c->ledc_speed_mode, c->ledc_channel);
    return err;
}

static esp_err_t step_ledc_stop(const tmc2209_stepper_config_t *c)
{
    // Stop PWM output and force STEP low
    esp_err_t err = ledc_stop(c->ledc_speed_mode, c->ledc_channel, 0);
    return err;
}

static esp_err_t pcnt_init(const tmc2209_stepper_config_t *c)
{
    pcnt_config_t pcfg = {
        .pulse_gpio_num = c->step_gpio,
        .ctrl_gpio_num  = PCNT_PIN_NOT_USED,
        .unit           = c->pcnt_unit,
        .channel        = c->pcnt_channel,
        .pos_mode       = PCNT_COUNT_INC,   // count rising edges
        .neg_mode       = PCNT_COUNT_DIS,   // ignore falling edges
        .lctrl_mode     = PCNT_MODE_KEEP,
        .hctrl_mode     = PCNT_MODE_KEEP,
        .counter_h_lim  = 32767,
        .counter_l_lim  = -32768,
    };

    esp_err_t err = pcnt_unit_config(&pcfg);
    if (err != ESP_OK) return err;

    // pcnt_unit_config may override GPIO direction; ensure INPUT_OUTPUT
    // so LEDC output is looped back through the pad for PCNT to read.
    gpio_set_direction(c->step_gpio, GPIO_MODE_INPUT_OUTPUT);

    // Filter to reject very short glitches (APB clk 80MHz, filter is in APB cycles)
    // 1000 cycles ~ 12.5 us. Tune if needed.
    pcnt_set_filter_value(c->pcnt_unit, 1000);
    pcnt_filter_enable(c->pcnt_unit);

    pcnt_counter_pause(c->pcnt_unit);
    pcnt_counter_clear(c->pcnt_unit);
    pcnt_counter_resume(c->pcnt_unit);

    return ESP_OK;
}

esp_err_t tmc2209_stepper_init(const tmc2209_stepper_config_t *cfg,
                               tmc2209_stepper_handle_t *out_handle)
{
    if (!cfg || !out_handle) return ESP_ERR_INVALID_ARG;
    if (cfg->step_gpio < 0) return ESP_ERR_INVALID_ARG;
    if (cfg->duty_percent == 0 || cfg->duty_percent > 90) {
        // 50% recommended. 0% invalid. >90% sometimes problematic for some drivers.
        return ESP_ERR_INVALID_ARG;
    }

    struct tmc2209_stepper_dev *dev = calloc(1, sizeof(*dev));
    if (!dev) return ESP_ERR_NO_MEM;
    dev->cfg = *cfg;
    dev->running = false;

    esp_err_t err = pcnt_init(&dev->cfg);
    if (err != ESP_OK) {
        free(dev);
        return err;
    }
    

    // Set STEP low initially
    gpio_set_level(dev->cfg.step_gpio, 0);

    *out_handle = dev;
    ESP_LOGI(TAG, "Init OK (STEP GPIO=%d)", (int)dev->cfg.step_gpio);
    return ESP_OK;
}

esp_err_t tmc2209_stepper_deinit(tmc2209_stepper_handle_t handle)
{
    if (!handle) return ESP_ERR_INVALID_ARG;
    // Stop if running
    (void)tmc2209_stepper_stop(handle);
    free(handle);
    return ESP_OK;
}

esp_err_t tmc2209_stepper_reset_count(tmc2209_stepper_handle_t handle)
{
    if (!handle) return ESP_ERR_INVALID_ARG;
    struct tmc2209_stepper_dev *dev = handle;
    pcnt_counter_pause(dev->cfg.pcnt_unit);
    pcnt_counter_clear(dev->cfg.pcnt_unit);
    pcnt_counter_resume(dev->cfg.pcnt_unit);
    return ESP_OK;
}

esp_err_t tmc2209_stepper_get_count(tmc2209_stepper_handle_t handle,
                                   int16_t *out_count)
{
    if (!handle || !out_count) return ESP_ERR_INVALID_ARG;
    struct tmc2209_stepper_dev *dev = handle;
    int16_t c = 0;
    esp_err_t err = pcnt_get_counter_value(dev->cfg.pcnt_unit, &c);
    if (err != ESP_OK) return err;
    *out_count = c;
    return ESP_OK;
}

esp_err_t tmc2209_stepper_start_continuous(tmc2209_stepper_handle_t handle,
                                           uint32_t step_hz)
{
    if (!handle) return ESP_ERR_INVALID_ARG;
    struct tmc2209_stepper_dev *dev = handle;

    ESP_ERROR_CHECK(tmc2209_stepper_reset_count(handle));

    esp_err_t err = ledc_set_step_freq(&dev->cfg, step_hz);
    if (err != ESP_OK) return err;

    err = step_ledc_start(&dev->cfg);
    if (err != ESP_OK) return err;

    dev->running = true;
    return ESP_OK;
}

esp_err_t tmc2209_stepper_stop(tmc2209_stepper_handle_t handle)
{
    if (!handle) return ESP_ERR_INVALID_ARG;
    struct tmc2209_stepper_dev *dev = handle;

    if (!dev->running) {
        // Still ensure STEP is low
        (void)step_ledc_stop(&dev->cfg);
        return ESP_OK;
    }

    dev->running = false;
    return step_ledc_stop(&dev->cfg);
}

esp_err_t tmc2209_stepper_move_steps(tmc2209_stepper_handle_t handle,
                                     uint32_t steps,
                                     uint32_t step_hz,
                                     uint32_t timeout_ms)
{
    if (!handle) return ESP_ERR_INVALID_ARG;
    if (steps == 0) return ESP_OK;
    if (step_hz == 0) return ESP_ERR_INVALID_ARG;

    struct tmc2209_stepper_dev *dev = handle;

    // Calculate exact duration: LEDC generates precise hardware pulses,
    // so (steps / step_hz) seconds gives us the exact run time.
    uint32_t duration_ms = (uint32_t)(((uint64_t)steps * 1000ULL) / (uint64_t)step_hz);
    if (duration_ms == 0) duration_ms = 1;

    esp_err_t err = ledc_set_step_freq(&dev->cfg, step_hz);
    if (err != ESP_OK) return err;

    err = step_ledc_start(&dev->cfg);
    if (err != ESP_OK) return err;

    dev->running = true;

    ESP_LOGI(TAG, "move_steps: %u steps @ %u Hz => %u ms",
             (unsigned)steps, (unsigned)step_hz, (unsigned)duration_ms);

    // Non-blocking delay — FreeRTOS schedules other tasks during this time
    vTaskDelay(pdMS_TO_TICKS(duration_ms));

    err = tmc2209_stepper_stop(handle);
    return err;
}

