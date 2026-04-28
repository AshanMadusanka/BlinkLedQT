#ifndef _DEFAULT_SOURCE
#define _DEFAULT_SOURCE
#endif

#include "tmc2209_stepper_rpi.h"

#include <errno.h>
#include <gpiod.h>
#include <pthread.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <time.h>
#include <unistd.h>

#define TMC2209_STEPPER_RPI_GPIOCHIP "gpiochip0"
#define TMC2209_STEPPER_RPI_CONSUMER "tmc2209-stepper-rpi"

struct tmc2209_stepper_rpi_dev {
    tmc2209_stepper_rpi_config_t cfg;
    struct gpiod_chip *chip;
    struct gpiod_line *step_line;
    pthread_mutex_t lock;
    pthread_t thread;
    bool lock_initialized;
    bool step_line_requested;
    bool thread_active;
    bool running;
    bool moving;
    bool stop_requested;
    uint32_t continuous_step_hz;
    int64_t count;
};

static int step_gpio_set(struct tmc2209_stepper_rpi_dev *dev, int level)
{
    if (dev == NULL || dev->step_line == NULL) {
        return TMC2209_STEPPER_RPI_ERR_INVALID_ARG;
    }

    return (gpiod_line_set_value(dev->step_line, level ? 1 : 0) == 0)
               ? TMC2209_STEPPER_RPI_OK
               : TMC2209_STEPPER_RPI_ERR_GPIO;
}

static void sleep_us(uint32_t us)
{
    struct timespec req;
    struct timespec rem;

    req.tv_sec = us / 1000000u;
    req.tv_nsec = (long)(us % 1000000u) * 1000L;

    while (nanosleep(&req, &rem) < 0 && errno == EINTR) {
        req = rem;
    }
}

static int elapsed_ms_since(const struct timespec *start, uint32_t timeout_ms)
{
    struct timespec now;
    uint64_t elapsed_ms;

    if (timeout_ms == 0) {
        return 0;
    }

    if (clock_gettime(CLOCK_MONOTONIC, &now) < 0) {
        return 0;
    }

    elapsed_ms = (uint64_t)(now.tv_sec - start->tv_sec) * 1000ULL;
    if (now.tv_nsec >= start->tv_nsec) {
        elapsed_ms += (uint64_t)(now.tv_nsec - start->tv_nsec) / 1000000ULL;
    } else {
        elapsed_ms -= 1000ULL;
        elapsed_ms += (uint64_t)(1000000000L + now.tv_nsec - start->tv_nsec) /
                      1000000ULL;
    }

    return elapsed_ms >= timeout_ms;
}

static int calculate_timing(const struct tmc2209_stepper_rpi_dev *dev,
                            uint32_t step_hz,
                            uint32_t *high_time_us,
                            uint32_t *low_time_us)
{
    uint32_t period_us;
    uint32_t high_us;
    uint32_t low_us;

    if (dev == NULL || high_time_us == NULL || low_time_us == NULL ||
        step_hz == 0) {
        return TMC2209_STEPPER_RPI_ERR_INVALID_ARG;
    }

    period_us = 1000000u / step_hz;
    high_us = (period_us * (uint32_t)dev->cfg.duty_percent) / 100u;
    low_us = period_us - high_us;

    if (period_us == 0 || high_us == 0 || low_us == 0) {
        return TMC2209_STEPPER_RPI_ERR_INVALID_ARG;
    }

    *high_time_us = high_us;
    *low_time_us = low_us;
    return TMC2209_STEPPER_RPI_OK;
}

static int should_stop(struct tmc2209_stepper_rpi_dev *dev)
{
    int stop;

    (void)pthread_mutex_lock(&dev->lock);
    stop = dev->stop_requested ? 1 : 0;
    (void)pthread_mutex_unlock(&dev->lock);

    return stop;
}

static void increment_count(struct tmc2209_stepper_rpi_dev *dev)
{
    (void)pthread_mutex_lock(&dev->lock);
    dev->count++;
    (void)pthread_mutex_unlock(&dev->lock);
}

static void *continuous_thread_func(void *arg)
{
    struct tmc2209_stepper_rpi_dev *dev = (struct tmc2209_stepper_rpi_dev *)arg;
    uint32_t high_time_us;
    uint32_t low_time_us;

    if (calculate_timing(dev, dev->continuous_step_hz, &high_time_us, &low_time_us) < 0) {
        (void)pthread_mutex_lock(&dev->lock);
        dev->running = false;
        (void)pthread_mutex_unlock(&dev->lock);
        return NULL;
    }

    while (!should_stop(dev)) {
        if (step_gpio_set(dev, 1) < 0) {
            break;
        }
        increment_count(dev);
        sleep_us(high_time_us);

        if (step_gpio_set(dev, 0) < 0) {
            break;
        }
        sleep_us(low_time_us);
    }

    (void)step_gpio_set(dev, 0);

    (void)pthread_mutex_lock(&dev->lock);
    dev->running = false;
    (void)pthread_mutex_unlock(&dev->lock);

    return NULL;
}

int tmc2209_stepper_rpi_init(const tmc2209_stepper_rpi_config_t *cfg,
                             tmc2209_stepper_rpi_handle_t *out_handle)
{
    struct tmc2209_stepper_rpi_dev *dev;
    int err;

    if (cfg == NULL || out_handle == NULL || cfg->step_gpio < 0 ||
        cfg->duty_percent == 0 || cfg->duty_percent > 90) {
        return TMC2209_STEPPER_RPI_ERR_INVALID_ARG;
    }

    *out_handle = NULL;

    dev = (struct tmc2209_stepper_rpi_dev *)calloc(1, sizeof(*dev));
    if (dev == NULL) {
        return TMC2209_STEPPER_RPI_ERR_NO_MEMORY;
    }

    dev->cfg = *cfg;

    err = pthread_mutex_init(&dev->lock, NULL);
    if (err != 0) {
        free(dev);
        return TMC2209_STEPPER_RPI_ERR_THREAD;
    }
    dev->lock_initialized = true;

    dev->chip = gpiod_chip_open_by_name(TMC2209_STEPPER_RPI_GPIOCHIP);
    if (dev->chip == NULL) {
        tmc2209_stepper_rpi_deinit(dev);
        return TMC2209_STEPPER_RPI_ERR_GPIO;
    }

    dev->step_line = gpiod_chip_get_line(dev->chip, (unsigned int)cfg->step_gpio);
    if (dev->step_line == NULL) {
        tmc2209_stepper_rpi_deinit(dev);
        return TMC2209_STEPPER_RPI_ERR_GPIO;
    }

    if (gpiod_line_request_output(dev->step_line,
                                  TMC2209_STEPPER_RPI_CONSUMER,
                                  0) < 0) {
        tmc2209_stepper_rpi_deinit(dev);
        return TMC2209_STEPPER_RPI_ERR_GPIO;
    }
    dev->step_line_requested = true;

    if (step_gpio_set(dev, 0) < 0) {
        tmc2209_stepper_rpi_deinit(dev);
        return TMC2209_STEPPER_RPI_ERR_GPIO;
    }

    *out_handle = dev;
    return TMC2209_STEPPER_RPI_OK;
}

int tmc2209_stepper_rpi_deinit(tmc2209_stepper_rpi_handle_t handle)
{
    if (handle == NULL) {
        return TMC2209_STEPPER_RPI_ERR_INVALID_ARG;
    }

    (void)tmc2209_stepper_rpi_stop(handle);
    (void)step_gpio_set(handle, 0);

    if (handle->step_line != NULL && handle->step_line_requested) {
        gpiod_line_release(handle->step_line);
    }
    handle->step_line = NULL;
    handle->step_line_requested = false;

    if (handle->chip != NULL) {
        gpiod_chip_close(handle->chip);
        handle->chip = NULL;
    }

    if (handle->lock_initialized) {
        (void)pthread_mutex_destroy(&handle->lock);
        handle->lock_initialized = false;
    }

    free(handle);
    return TMC2209_STEPPER_RPI_OK;
}

int tmc2209_stepper_rpi_move_steps(tmc2209_stepper_rpi_handle_t handle,
                                   uint32_t steps,
                                   uint32_t step_hz,
                                   uint32_t timeout_ms)
{
    uint32_t high_time_us;
    uint32_t low_time_us;
    struct timespec start;
    int err;
    int result = TMC2209_STEPPER_RPI_OK;

    if (handle == NULL || step_hz == 0) {
        return TMC2209_STEPPER_RPI_ERR_INVALID_ARG;
    }

    if (steps == 0) {
        return TMC2209_STEPPER_RPI_OK;
    }

    err = calculate_timing(handle, step_hz, &high_time_us, &low_time_us);
    if (err < 0) {
        return err;
    }

    err = pthread_mutex_lock(&handle->lock);
    if (err != 0) {
        return TMC2209_STEPPER_RPI_ERR_THREAD;
    }

    if (handle->running || handle->moving) {
        (void)pthread_mutex_unlock(&handle->lock);
        return TMC2209_STEPPER_RPI_ERR_BUSY;
    }

    handle->moving = true;
    handle->stop_requested = false;
    (void)pthread_mutex_unlock(&handle->lock);

    if (timeout_ms != 0 && clock_gettime(CLOCK_MONOTONIC, &start) < 0) {
        result = TMC2209_STEPPER_RPI_ERR_THREAD;
    }

    for (uint32_t step = 0; result == TMC2209_STEPPER_RPI_OK && step < steps; ++step) {
        if (timeout_ms != 0 && elapsed_ms_since(&start, timeout_ms)) {
            result = TMC2209_STEPPER_RPI_ERR_BUSY;
            break;
        }

        if (should_stop(handle)) {
            result = TMC2209_STEPPER_RPI_ERR_BUSY;
            break;
        }

        result = step_gpio_set(handle, 1);
        if (result < 0) {
            break;
        }

        increment_count(handle);
        sleep_us(high_time_us);

        result = step_gpio_set(handle, 0);
        if (result < 0) {
            break;
        }

        sleep_us(low_time_us);
    }

    (void)step_gpio_set(handle, 0);

    (void)pthread_mutex_lock(&handle->lock);
    handle->moving = false;
    handle->stop_requested = false;
    (void)pthread_mutex_unlock(&handle->lock);

    return result;
}

int tmc2209_stepper_rpi_start_continuous(tmc2209_stepper_rpi_handle_t handle,
                                         uint32_t step_hz)
{
    uint32_t high_time_us;
    uint32_t low_time_us;
    int err;

    if (handle == NULL || step_hz == 0) {
        return TMC2209_STEPPER_RPI_ERR_INVALID_ARG;
    }

    err = calculate_timing(handle, step_hz, &high_time_us, &low_time_us);
    if (err < 0) {
        return err;
    }

    (void)high_time_us;
    (void)low_time_us;

    err = pthread_mutex_lock(&handle->lock);
    if (err != 0) {
        return TMC2209_STEPPER_RPI_ERR_THREAD;
    }

    if (handle->running || handle->moving) {
        (void)pthread_mutex_unlock(&handle->lock);
        return TMC2209_STEPPER_RPI_ERR_BUSY;
    }

    handle->stop_requested = false;
    handle->continuous_step_hz = step_hz;
    handle->running = true;
    handle->thread_active = true;
    (void)pthread_mutex_unlock(&handle->lock);

    err = pthread_create(&handle->thread, NULL, continuous_thread_func, handle);
    if (err != 0) {
        (void)pthread_mutex_lock(&handle->lock);
        handle->running = false;
        handle->thread_active = false;
        (void)pthread_mutex_unlock(&handle->lock);
        return TMC2209_STEPPER_RPI_ERR_THREAD;
    }

    return TMC2209_STEPPER_RPI_OK;
}

int tmc2209_stepper_rpi_stop(tmc2209_stepper_rpi_handle_t handle)
{
    bool join_needed;

    if (handle == NULL) {
        return TMC2209_STEPPER_RPI_ERR_INVALID_ARG;
    }

    if (pthread_mutex_lock(&handle->lock) != 0) {
        return TMC2209_STEPPER_RPI_ERR_THREAD;
    }

    handle->stop_requested = true;
    join_needed = handle->thread_active;
    (void)pthread_mutex_unlock(&handle->lock);

    if (join_needed) {
        if (pthread_join(handle->thread, NULL) != 0) {
            return TMC2209_STEPPER_RPI_ERR_THREAD;
        }
    }

    (void)step_gpio_set(handle, 0);

    if (pthread_mutex_lock(&handle->lock) != 0) {
        return TMC2209_STEPPER_RPI_ERR_THREAD;
    }
    handle->running = false;
    handle->thread_active = false;
    if (!handle->moving) {
        handle->stop_requested = false;
    }
    (void)pthread_mutex_unlock(&handle->lock);

    return TMC2209_STEPPER_RPI_OK;
}

int tmc2209_stepper_rpi_get_count(tmc2209_stepper_rpi_handle_t handle,
                                  int64_t *out_count)
{
    if (handle == NULL || out_count == NULL) {
        return TMC2209_STEPPER_RPI_ERR_INVALID_ARG;
    }

    if (pthread_mutex_lock(&handle->lock) != 0) {
        return TMC2209_STEPPER_RPI_ERR_THREAD;
    }

    *out_count = handle->count;
    (void)pthread_mutex_unlock(&handle->lock);

    return TMC2209_STEPPER_RPI_OK;
}

int tmc2209_stepper_rpi_reset_count(tmc2209_stepper_rpi_handle_t handle)
{
    if (handle == NULL) {
        return TMC2209_STEPPER_RPI_ERR_INVALID_ARG;
    }

    if (pthread_mutex_lock(&handle->lock) != 0) {
        return TMC2209_STEPPER_RPI_ERR_THREAD;
    }

    handle->count = 0;
    (void)pthread_mutex_unlock(&handle->lock);

    return TMC2209_STEPPER_RPI_OK;
}
