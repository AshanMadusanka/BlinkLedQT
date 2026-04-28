#include "flow_sensor_rpi.h"

#include <errno.h>
#include <gpiod.h>
#include <pthread.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#define FLOW_SENSOR_RPI_CONSUMER "flow-sensor-rpi"
#define FLOW_SENSOR_RPI_EVENT_WAIT_NS (100 * 1000 * 1000LL)
#define FLOW_SENSOR_RPI_EDGE_BUFFER_SIZE 16

struct flow_sensor_rpi_dev {
    int gpio;
    uint8_t sensor_id;
    float calibration_factor;

    struct gpiod_chip *chip;
    struct gpiod_line_request *request;
    struct gpiod_edge_event_buffer *event_buffer;

    pthread_t thread;
    pthread_mutex_t lock;
    bool lock_initialized;
    bool thread_started;
    bool running;

    uint32_t pulse_count_1s;
    uint64_t total_pulses;

    float frequency_hz;
    float flow_rate_l_min;
};

static flow_sensor_rpi_handle_t sensors[FLOW_SENSOR_RPI_MAX_SENSORS];
static pthread_mutex_t sensors_lock = PTHREAD_MUTEX_INITIALIZER;

static double elapsed_seconds(const struct timespec *start,
                              const struct timespec *end)
{
    double seconds = (double)(end->tv_sec - start->tv_sec);
    double nanoseconds = (double)(end->tv_nsec - start->tv_nsec) / 1000000000.0;

    return seconds + nanoseconds;
}

static int add_sensor_to_list(flow_sensor_rpi_handle_t handle)
{
    int err;
    int result = FLOW_SENSOR_RPI_ERR_NOT_FOUND;

    err = pthread_mutex_lock(&sensors_lock);
    if (err != 0) {
        return FLOW_SENSOR_RPI_ERR_THREAD;
    }

    for (size_t i = 0; i < FLOW_SENSOR_RPI_MAX_SENSORS; ++i) {
        if (sensors[i] == NULL) {
            sensors[i] = handle;
            result = FLOW_SENSOR_RPI_OK;
            break;
        }
    }

    (void)pthread_mutex_unlock(&sensors_lock);
    return result;
}

static void remove_sensor_from_list(flow_sensor_rpi_handle_t handle)
{
    if (pthread_mutex_lock(&sensors_lock) != 0) {
        return;
    }

    for (size_t i = 0; i < FLOW_SENSOR_RPI_MAX_SENSORS; ++i) {
        if (sensors[i] == handle) {
            sensors[i] = NULL;
            break;
        }
    }

    (void)pthread_mutex_unlock(&sensors_lock);
}

static flow_sensor_rpi_handle_t find_sensor_by_id(uint8_t sensor_id)
{
    flow_sensor_rpi_handle_t found = NULL;

    if (pthread_mutex_lock(&sensors_lock) != 0) {
        return NULL;
    }

    for (size_t i = 0; i < FLOW_SENSOR_RPI_MAX_SENSORS; ++i) {
        if (sensors[i] != NULL && sensors[i]->sensor_id == sensor_id) {
            found = sensors[i];
            break;
        }
    }

    (void)pthread_mutex_unlock(&sensors_lock);
    return found;
}

static int request_rising_edge_line(struct flow_sensor_rpi_dev *dev,
                                    const char *gpiochip)
{
    struct gpiod_line_settings *settings = NULL;
    struct gpiod_line_config *line_config = NULL;
    struct gpiod_request_config *request_config = NULL;
    unsigned int offset;
    int result = FLOW_SENSOR_RPI_ERR_GPIO;

    dev->chip = gpiod_chip_open(gpiochip);
    if (dev->chip == NULL) {
        return FLOW_SENSOR_RPI_ERR_GPIO;
    }

    settings = gpiod_line_settings_new();
    line_config = gpiod_line_config_new();
    request_config = gpiod_request_config_new();
    if (settings == NULL || line_config == NULL || request_config == NULL) {
        goto cleanup;
    }

    if (gpiod_line_settings_set_direction(settings, GPIOD_LINE_DIRECTION_INPUT) < 0) {
        goto cleanup;
    }
    if (gpiod_line_settings_set_edge_detection(settings, GPIOD_LINE_EDGE_RISING) < 0) {
        goto cleanup;
    }

    offset = (unsigned int)dev->gpio;
    if (gpiod_line_config_add_line_settings(line_config, &offset, 1, settings) < 0) {
        goto cleanup;
    }

    gpiod_request_config_set_consumer(request_config, FLOW_SENSOR_RPI_CONSUMER);
    gpiod_request_config_set_event_buffer_size(request_config,
                                               FLOW_SENSOR_RPI_EDGE_BUFFER_SIZE);

    dev->request = gpiod_chip_request_lines(dev->chip, request_config, line_config);
    if (dev->request == NULL) {
        goto cleanup;
    }

    dev->event_buffer = gpiod_edge_event_buffer_new(FLOW_SENSOR_RPI_EDGE_BUFFER_SIZE);
    if (dev->event_buffer == NULL) {
        goto cleanup;
    }

    result = FLOW_SENSOR_RPI_OK;

cleanup:
    if (request_config != NULL) {
        gpiod_request_config_free(request_config);
    }
    if (line_config != NULL) {
        gpiod_line_config_free(line_config);
    }
    if (settings != NULL) {
        gpiod_line_settings_free(settings);
    }

    return result;
}

static int handle_edge_events(struct flow_sensor_rpi_dev *dev)
{
    int num_events;

    num_events = gpiod_line_request_read_edge_events(dev->request,
                                                     dev->event_buffer,
                                                     FLOW_SENSOR_RPI_EDGE_BUFFER_SIZE);
    if (num_events < 0) {
        return FLOW_SENSOR_RPI_ERR_GPIO;
    }

    if (pthread_mutex_lock(&dev->lock) != 0) {
        return FLOW_SENSOR_RPI_ERR_THREAD;
    }

    for (int i = 0; i < num_events; ++i) {
        struct gpiod_edge_event *event;

        event = gpiod_edge_event_buffer_get_event(dev->event_buffer,
                                                  (unsigned long)i);
        if (event != NULL &&
            gpiod_edge_event_get_event_type(event) == GPIOD_EDGE_EVENT_RISING_EDGE) {
            dev->pulse_count_1s++;
            dev->total_pulses++;
        }
    }

    (void)pthread_mutex_unlock(&dev->lock);
    return FLOW_SENSOR_RPI_OK;
}

static void update_frequency_if_due(struct flow_sensor_rpi_dev *dev,
                                    struct timespec *last_update)
{
    struct timespec now;
    double elapsed;

    if (clock_gettime(CLOCK_MONOTONIC, &now) < 0) {
        return;
    }

    elapsed = elapsed_seconds(last_update, &now);
    if (elapsed < 1.0) {
        return;
    }

    if (pthread_mutex_lock(&dev->lock) != 0) {
        return;
    }

    dev->frequency_hz = (float)((double)dev->pulse_count_1s / elapsed);
    dev->flow_rate_l_min = dev->frequency_hz / dev->calibration_factor;
    dev->pulse_count_1s = 0;

    (void)pthread_mutex_unlock(&dev->lock);
    *last_update = now;
}

static bool is_running(struct flow_sensor_rpi_dev *dev)
{
    bool running;

    if (pthread_mutex_lock(&dev->lock) != 0) {
        return false;
    }

    running = dev->running;
    (void)pthread_mutex_unlock(&dev->lock);

    return running;
}

static void *flow_sensor_thread(void *arg)
{
    struct flow_sensor_rpi_dev *dev = (struct flow_sensor_rpi_dev *)arg;
    struct timespec last_update;

    (void)clock_gettime(CLOCK_MONOTONIC, &last_update);

    while (is_running(dev)) {
        int wait_result;

        wait_result = gpiod_line_request_wait_edge_events(dev->request,
                                                          FLOW_SENSOR_RPI_EVENT_WAIT_NS);
        if (wait_result < 0) {
            break;
        }

        if (wait_result > 0) {
            (void)handle_edge_events(dev);
        }

        update_frequency_if_due(dev, &last_update);
    }

    update_frequency_if_due(dev, &last_update);
    return NULL;
}

int flow_sensor_rpi_init(const flow_sensor_rpi_config_t *config,
                         flow_sensor_rpi_handle_t *out_handle)
{
    struct flow_sensor_rpi_dev *dev;
    const char *gpiochip;
    int err;

    if (config == NULL || out_handle == NULL || config->gpio < 0 ||
        config->calibration_factor <= 0.0f) {
        return FLOW_SENSOR_RPI_ERR_INVALID_ARG;
    }

    *out_handle = NULL;

    dev = (struct flow_sensor_rpi_dev *)calloc(1, sizeof(*dev));
    if (dev == NULL) {
        return FLOW_SENSOR_RPI_ERR_NO_MEMORY;
    }

    dev->gpio = config->gpio;
    dev->sensor_id = config->sensor_id;
    dev->calibration_factor = config->calibration_factor;
    gpiochip = (config->gpiochip != NULL) ? config->gpiochip
                                          : FLOW_SENSOR_RPI_DEFAULT_GPIOCHIP;

    err = pthread_mutex_init(&dev->lock, NULL);
    if (err != 0) {
        flow_sensor_rpi_deinit(dev);
        return FLOW_SENSOR_RPI_ERR_THREAD;
    }
    dev->lock_initialized = true;

    err = request_rising_edge_line(dev, gpiochip);
    if (err < 0) {
        flow_sensor_rpi_deinit(dev);
        return err;
    }

    err = add_sensor_to_list(dev);
    if (err < 0) {
        flow_sensor_rpi_deinit(dev);
        return err;
    }

    dev->running = true;
    err = pthread_create(&dev->thread, NULL, flow_sensor_thread, dev);
    if (err != 0) {
        flow_sensor_rpi_deinit(dev);
        return FLOW_SENSOR_RPI_ERR_THREAD;
    }
    dev->thread_started = true;

    *out_handle = dev;
    return FLOW_SENSOR_RPI_OK;
}

int flow_sensor_rpi_deinit(flow_sensor_rpi_handle_t handle)
{
    if (handle == NULL) {
        return FLOW_SENSOR_RPI_ERR_INVALID_ARG;
    }

    if (handle->lock_initialized) {
        if (pthread_mutex_lock(&handle->lock) == 0) {
            handle->running = false;
            (void)pthread_mutex_unlock(&handle->lock);
        }
    }

    if (handle->thread_started) {
        if (pthread_join(handle->thread, NULL) != 0) {
            return FLOW_SENSOR_RPI_ERR_THREAD;
        }
        handle->thread_started = false;
    }

    remove_sensor_from_list(handle);

    if (handle->event_buffer != NULL) {
        gpiod_edge_event_buffer_free(handle->event_buffer);
        handle->event_buffer = NULL;
    }

    if (handle->request != NULL) {
        gpiod_line_request_release(handle->request);
        handle->request = NULL;
    }

    if (handle->chip != NULL) {
        gpiod_chip_close(handle->chip);
        handle->chip = NULL;
    }

    if (handle->lock_initialized) {
        (void)pthread_mutex_destroy(&handle->lock);
        handle->lock_initialized = false;
    }

    free(handle);
    return FLOW_SENSOR_RPI_OK;
}

int flow_sensor_rpi_get_frequency(flow_sensor_rpi_handle_t handle,
                                  float *frequency_hz)
{
    if (handle == NULL || frequency_hz == NULL) {
        return FLOW_SENSOR_RPI_ERR_INVALID_ARG;
    }

    if (pthread_mutex_lock(&handle->lock) != 0) {
        return FLOW_SENSOR_RPI_ERR_THREAD;
    }

    *frequency_hz = handle->frequency_hz;
    (void)pthread_mutex_unlock(&handle->lock);

    return FLOW_SENSOR_RPI_OK;
}

int flow_sensor_rpi_get_flow_rate_l_min(flow_sensor_rpi_handle_t handle,
                                        float *flow_rate_l_min)
{
    if (handle == NULL || flow_rate_l_min == NULL) {
        return FLOW_SENSOR_RPI_ERR_INVALID_ARG;
    }

    if (pthread_mutex_lock(&handle->lock) != 0) {
        return FLOW_SENSOR_RPI_ERR_THREAD;
    }

    *flow_rate_l_min = handle->flow_rate_l_min;
    (void)pthread_mutex_unlock(&handle->lock);

    return FLOW_SENSOR_RPI_OK;
}

int flow_sensor_rpi_get_total_pulses(flow_sensor_rpi_handle_t handle,
                                     uint64_t *total_pulses)
{
    if (handle == NULL || total_pulses == NULL) {
        return FLOW_SENSOR_RPI_ERR_INVALID_ARG;
    }

    if (pthread_mutex_lock(&handle->lock) != 0) {
        return FLOW_SENSOR_RPI_ERR_THREAD;
    }

    *total_pulses = handle->total_pulses;
    (void)pthread_mutex_unlock(&handle->lock);

    return FLOW_SENSOR_RPI_OK;
}

int flow_sensor_rpi_reset_total_pulses(flow_sensor_rpi_handle_t handle)
{
    if (handle == NULL) {
        return FLOW_SENSOR_RPI_ERR_INVALID_ARG;
    }

    if (pthread_mutex_lock(&handle->lock) != 0) {
        return FLOW_SENSOR_RPI_ERR_THREAD;
    }

    handle->total_pulses = 0;
    (void)pthread_mutex_unlock(&handle->lock);

    return FLOW_SENSOR_RPI_OK;
}

int flow_sensor_rpi_get_frequency_by_sensor_id(uint8_t sensor_id,
                                               float *frequency_hz)
{
    flow_sensor_rpi_handle_t handle;

    if (frequency_hz == NULL) {
        return FLOW_SENSOR_RPI_ERR_INVALID_ARG;
    }

    handle = find_sensor_by_id(sensor_id);
    if (handle == NULL) {
        return FLOW_SENSOR_RPI_ERR_NOT_FOUND;
    }

    return flow_sensor_rpi_get_frequency(handle, frequency_hz);
}

int flow_sensor_rpi_get_flow_rate_l_min_by_sensor_id(uint8_t sensor_id,
                                                     float *flow_rate_l_min)
{
    flow_sensor_rpi_handle_t handle;

    if (flow_rate_l_min == NULL) {
        return FLOW_SENSOR_RPI_ERR_INVALID_ARG;
    }

    handle = find_sensor_by_id(sensor_id);
    if (handle == NULL) {
        return FLOW_SENSOR_RPI_ERR_NOT_FOUND;
    }

    return flow_sensor_rpi_get_flow_rate_l_min(handle, flow_rate_l_min);
}

int flow_sensor_rpi_set_calibration_factor(flow_sensor_rpi_handle_t handle,
                                           float calibration_factor)
{
    if (handle == NULL || calibration_factor <= 0.0f) {
        return FLOW_SENSOR_RPI_ERR_INVALID_ARG;
    }

    if (pthread_mutex_lock(&handle->lock) != 0) {
        return FLOW_SENSOR_RPI_ERR_THREAD;
    }

    handle->calibration_factor = calibration_factor;
    handle->flow_rate_l_min = handle->frequency_hz / handle->calibration_factor;
    (void)pthread_mutex_unlock(&handle->lock);

    return FLOW_SENSOR_RPI_OK;
}

int flow_sensor_rpi_get_calibration_factor(flow_sensor_rpi_handle_t handle,
                                           float *calibration_factor)
{
    if (handle == NULL || calibration_factor == NULL) {
        return FLOW_SENSOR_RPI_ERR_INVALID_ARG;
    }

    if (pthread_mutex_lock(&handle->lock) != 0) {
        return FLOW_SENSOR_RPI_ERR_THREAD;
    }

    *calibration_factor = handle->calibration_factor;
    (void)pthread_mutex_unlock(&handle->lock);

    return FLOW_SENSOR_RPI_OK;
}
