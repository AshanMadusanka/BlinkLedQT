idf_component_register(SRCS "sn3002_co2.c"
                    INCLUDE_DIRS .
                    REQUIRES freertos esp_common modbus_rs485)
