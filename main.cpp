#include <QCoreApplication>
#include <QDebug>
#include <QThread>
#include <gpiod.h>
#include <iostream>
#include <unistd.h>

#include <cerrno>
#include <csignal>
#include <cstdio>
#include <cstring>

#include "mcp23017_rpi.h"
#include "modbus_rs485_rpi/modbus_rs485_rpi.h"
#include "cwt_sl_lth_rpi/cwt_sl_lth_rpi.h"

static volatile sig_atomic_t keepRunning = 1;

// modbus_rs485_rpi_bus_t bus = {};
// cwt_sl_lth_rpi_handle_t sensor = nullptr;

// static void read_cwt_sl_lth_temp_humidity(cwt_sl_lth_rpi_handle_t sensor);
// class SensorThread : public QThread
// {
// protected:
//     void run() override
//     {
//         while (true)
//         {
//             qDebug() << "Reading sensor...";
//             read_cwt_sl_lth_temp_humidity(sensor);
//             QThread::sleep(5); // Sleep for 5 seconds
//         }
//     }
// };

static void handleSignal(int signalNumber)
{
    Q_UNUSED(signalNumber);
    keepRunning = 0;
}

// static QString errorText(int errorCode)
// {
//     return QString::fromLocal8Bit(std::strerror(-errorCode));
// }

// static bool configureLed(mcp23017_rpi_t *device, quint8 address)
// {
//     const int err = mcp23017_rpi_set_pin_dir(device,
//                                              MCP23017_RPI_PIN_B7,
//                                              MCP23017_RPI_PIN_DIR_OUTPUT);
//     if (err < 0) {
//         qCritical() << "Failed to configure GPB7 as output on expander"
//                     << Qt::hex << address << Qt::dec
//                     << "error" << err << errorText(err);
//         return false;
//     }

//     return true;
// }

// static bool writeLed(mcp23017_rpi_t *device,
//                      quint8 address,
//                      mcp23017_rpi_pin_level_t level)
// {
//     const int err = mcp23017_rpi_write_pin(device, MCP23017_RPI_PIN_B7, level);
//     if (err < 0) {
//         qCritical() << "Failed to write GPB7 on expander"
//                     << Qt::hex << address << Qt::dec
//                     << "error" << err << errorText(err);
//         return false;
//     }

//     return true;
// }

// static bool init_cwt_sl_lth_sensor(modbus_rs485_rpi_bus_t *bus,
//                                    cwt_sl_lth_rpi_handle_t *sensor)
// {
//     int err = modbus_rs485_rpi_open(bus, "/dev/serial0", 4800);
//     if (err != MODBUS_RS485_RPI_OK) {
//         const int openErrno = errno;
//         std::printf("Failed to open RS485 bus on /dev/serial0, error %d, errno %d (%s)\n",
//                     err,
//                     openErrno,
//                     std::strerror(openErrno));
//         std::fflush(stdout);
//         return false;
//     }

//     cwt_sl_lth_rpi_config_t config = {};
//     config.bus = bus;
//     config.slave_id = 1;
//     config.lux_mode = CWT_SL_LTH_RPI_LUX_SIMPLE_1X;

//     err = cwt_sl_lth_rpi_create(&config, sensor);
//     if (err != CWT_SL_LTH_RPI_OK) {
//         std::printf("Failed to create CWT-SL-LTH sensor handle, error %d\n", err);
//         std::fflush(stdout);
//         modbus_rs485_rpi_close(bus);
//         return false;
//     }

//     return true;
// }

// static void read_cwt_sl_lth_temp_humidity(cwt_sl_lth_rpi_handle_t sensor)
// {
//     float temperature = 0.0f;
//     float humidity = 0.0f;

//     const int err = cwt_sl_lth_rpi_read_temp_hum(sensor, &temperature, &humidity);
//     if (err == CWT_SL_LTH_RPI_OK) {
//         std::printf("CWT-SL-LTH Temperature: %.1f C\n", temperature);
//         std::printf("CWT-SL-LTH Humidity: %.1f %%\n", humidity);
//     } else {
//         std::printf("Failed to read CWT-SL-LTH temperature/humidity, error %d\n", err);
//     }

//     std::fflush(stdout);
// }


int main(int argc, char *argv[])
{
    QCoreApplication app(argc, argv);
    const char *chipname = "/dev/gpiochip0";
    const unsigned int line_num = 17; // BCM GPIO17

    std::signal(SIGINT, handleSignal);
    std::signal(SIGTERM, handleSignal);

    gpiod_chip *chip = gpiod_chip_open(chipname);
    if (!chip) {
        std::cerr << "Failed to open GPIO chip\n";
        return 1;
    }

    gpiod_line_settings *settings = gpiod_line_settings_new();
    if (!settings) {
        std::cerr << "Failed to create GPIO line settings\n";
        gpiod_chip_close(chip);
        return 1;
    }

    int ret = gpiod_line_settings_set_direction(settings, GPIOD_LINE_DIRECTION_OUTPUT);
    if (ret < 0) {
        std::cerr << "Failed to set GPIO line direction\n";
        gpiod_line_settings_free(settings);
        gpiod_chip_close(chip);
        return 1;
    }

    ret = gpiod_line_settings_set_output_value(settings, GPIOD_LINE_VALUE_INACTIVE);
    if (ret < 0) {
        std::cerr << "Failed to set initial GPIO line value\n";
        gpiod_line_settings_free(settings);
        gpiod_chip_close(chip);
        return 1;
    }

    gpiod_line_config *line_config = gpiod_line_config_new();
    if (!line_config) {
        std::cerr << "Failed to create GPIO line config\n";
        gpiod_line_settings_free(settings);
        gpiod_chip_close(chip);
        return 1;
    }

    ret = gpiod_line_config_add_line_settings(line_config, &line_num, 1, settings);
    if (ret < 0) {
        std::cerr << "Failed to add GPIO line settings\n";
        gpiod_line_config_free(line_config);
        gpiod_line_settings_free(settings);
        gpiod_chip_close(chip);
        return 1;
    }

    gpiod_request_config *request_config = gpiod_request_config_new();
    if (!request_config) {
        std::cerr << "Failed to create GPIO request config\n";
        gpiod_line_config_free(line_config);
        gpiod_line_settings_free(settings);
        gpiod_chip_close(chip);
        return 1;
    }

    gpiod_request_config_set_consumer(request_config, "blink-led");

    gpiod_line_request *request = gpiod_chip_request_lines(chip, request_config, line_config);
    gpiod_request_config_free(request_config);
    gpiod_line_config_free(line_config);
    gpiod_line_settings_free(settings);

    if (!request) {
        std::cerr << "Failed to request GPIO line as output\n";
        gpiod_chip_close(chip);
        return 1;
    }

    qInfo() << "BlinkLed started";
    qInfo() << "Using GPIO17";

    int exit_code = 0;
    while (keepRunning) {
        if (gpiod_line_request_set_value(request, line_num, GPIOD_LINE_VALUE_ACTIVE) < 0) {
            std::cerr << "Failed to set GPIO line active\n";
            exit_code = 1;
            break;
        }
        std::cout << "LED ON\n";
        sleep(1);

        if (!keepRunning) {
            break;
        }

        if (gpiod_line_request_set_value(request, line_num, GPIOD_LINE_VALUE_INACTIVE) < 0) {
            std::cerr << "Failed to set GPIO line inactive\n";
            exit_code = 1;
            break;
        }
        std::cout << "LED OFF\n";
        sleep(1);
    }

    (void)gpiod_line_request_set_value(request, line_num, GPIOD_LINE_VALUE_INACTIVE);
    gpiod_line_request_release(request);
    gpiod_chip_close(chip);

    qInfo() << "BlinkLed stopped";
    return exit_code;
}
    
    // if (!init_cwt_sl_lth_sensor(&bus, &sensor)) {
    //     return 1;
    // }

    // SensorThread *sensor = new SensorThread();
    // sensor->start();
    

    // constexpr const char *i2cDevice = MCP23017_RPI_DEFAULT_I2C_DEV;
    // constexpr quint8 expander1Address = 0x22;
    // constexpr quint8 expander2Address = 0x20;

    // mcp23017_rpi_t expander1;
    // mcp23017_rpi_t expander2;
    // bool expander1Open = false;
    // bool expander2Open = false;
    // int exitCode = 0;

    // std::signal(SIGINT, handleSignal);
    // std::signal(SIGTERM, handleSignal);

    // qInfo() << "BlinkLed started";
    // qInfo() << "Using MCP23017 expanders at 0x22 and 0x20 on" << i2cDevice;
    // qInfo() << "Blinking GPB7 on both expanders";

    // int err = mcp23017_rpi_open(&expander1, i2cDevice, expander1Address);
    // if (err < 0) {
    //     qCritical() << "Failed to open expander 1 at 0x22, error" << err << errorText(err);
    //     return 1;
    // }
    // expander1Open = true;

    // err = mcp23017_rpi_open(&expander2, i2cDevice, expander2Address);
    // if (err < 0) {
    //     qCritical() << "Failed to open expander 2 at 0x20, error" << err << errorText(err);
    //     mcp23017_rpi_close(&expander1);
    //     return 1;
    // }
    // expander2Open = true;

    // if (!configureLed(&expander1, expander1Address) ||
    //     !configureLed(&expander2, expander2Address)) {
    //     exitCode = 1;
    //     keepRunning = 0;
    // }

    // while (keepRunning) {
    //     if (!writeLed(&expander1, expander1Address, MCP23017_RPI_PIN_LEVEL_HIGH) ||
    //         !writeLed(&expander2, expander2Address, MCP23017_RPI_PIN_LEVEL_HIGH)) {
    //         exitCode = 1;
    //         break;
    //     }

    //     qInfo() << "LEDs ON";

    //     QThread::msleep(500);
    //     if (!keepRunning) {
    //         break;
    //     }

    //     if (!writeLed(&expander1, expander1Address, MCP23017_RPI_PIN_LEVEL_LOW) ||
    //         !writeLed(&expander2, expander2Address, MCP23017_RPI_PIN_LEVEL_LOW)) {
    //         exitCode = 1;
    //         break;
    //     }

    //     qInfo() << "LEDs OFF";

    //     QThread::msleep(500);
    // }

    // if (expander1Open) {
    //     (void)mcp23017_rpi_write_pin(&expander1,
    //                                  MCP23017_RPI_PIN_B7,
    //                                  MCP23017_RPI_PIN_LEVEL_LOW);
    //     mcp23017_rpi_close(&expander1);
    // }

    // if (expander2Open) {
    //     (void)mcp23017_rpi_write_pin(&expander2,
    //                                  MCP23017_RPI_PIN_B7,
    //                                  MCP23017_RPI_PIN_LEVEL_LOW);
    //     mcp23017_rpi_close(&expander2);
    // }

    // qInfo() << "BlinkLed stopped";
    // return exitCode;
