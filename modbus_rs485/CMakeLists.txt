idf_component_register(SRCS "modbus_rs485.c"
                    INCLUDE_DIRS .
                    REQUIRES freertos esp_common driver)
