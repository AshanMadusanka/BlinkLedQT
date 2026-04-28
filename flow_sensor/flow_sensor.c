#include "flow_sensor.h"

static const char *TAG = "FlowSensor";  // Tag for logging
static flow_sensor_handle_t* sensors[MAX_FLOW_SENSORS] = {0};
static uint8_t sensor_tracker = 0;
static uint32_t test_count= 0 ;

// ISR to handle pulses from the flow sensor (called on rising edge of pulse)
static void IRAM_ATTR flow_sensor_isr_handler(void* arg) {
    flow_sensor_handle_t *handle = (flow_sensor_handle_t *)arg;
    if (handle) {
        handle->pulse_count++;
    }
	//test_count++;
}

// Timer callback to calculate flow frequency once every second
static void flow_timer_callback(TimerHandle_t xTimer) {
    flow_sensor_handle_t *handle = (flow_sensor_handle_t *)pvTimerGetTimerID(xTimer);
    if (handle) {
        handle->flow_frequency = handle->pulse_count;  // Copy pulse count as flow frequency (Hz)
        handle->pulse_count = 0;                      // Reset pulse count for the next interval
    }
	//test_count=0;
}

// Initialization function to configure the GPIO, ISR, and FreeRTOS timer
esp_err_t flow_sensor_init(flow_sensor_handle_t *handle) {
	static bool isr_service_installed = false;
    if (!handle) {
        ESP_LOGE(TAG, "Invalid flow sensor handle");
        return;
    }

    // Configure GPIO for flow sensor input (with pull-up) and interrupt on rising edge
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_POSEDGE,      // Interrupt on rising edge
        .mode = GPIO_MODE_INPUT,             // Input mode
        .pin_bit_mask = (1ULL << handle->flow_sensor_pin),  // Bit mask for the pin
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_ENABLE
    };
    gpio_config(&io_conf);

    // Install the ISR service and add the ISR handler for the flow 
	if(isr_service_installed == false){
		gpio_install_isr_service(0);
		isr_service_installed = true;
	}
	
    gpio_isr_handler_add(handle->flow_sensor_pin, flow_sensor_isr_handler, (void*)handle);

    // Create and start a FreeRTOS timer with a 1-second period to calculate flow rate
    handle->flow_timer = xTimerCreate("flow_rate_timer", pdMS_TO_TICKS(1000), pdTRUE, (void *)handle, flow_timer_callback);

    if (handle->flow_timer != NULL) {
        if (sensor_tracker < MAX_FLOW_SENSORS) {
            sensors[sensor_tracker++] = handle;
			xTimerStart(handle->flow_timer, 0);
        } else {
            ESP_LOGE(TAG, "Maximum number of flow sensors reached");
        }
    } else {
        ESP_LOGE(TAG, "Failed to create flow rate timer");
    }
	return ESP_OK;
}

// Function to get the current flow frequency (in pulses per second, Hz) by sensor ID
uint32_t get_flow_frequency_by_sensor_id(uint8_t sensor_id) {
	//return test_count;
    for (uint8_t i = 0; i < sensor_tracker; i++) {
        if (sensors[i] && sensors[i]->sensor_id == sensor_id) {
            return sensors[i]->flow_frequency;
        }
    }
    return 0;
}

// Function to get the current flow frequency (in pulses per second, Hz)
uint32_t get_flow_frequency(flow_sensor_handle_t *handle) {
    return (handle) ? handle->flow_frequency : 0;
}
