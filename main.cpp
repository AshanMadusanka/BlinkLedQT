#include <QCoreApplication>
#include <QDebug>
#include <QMutex>
#include <QMutexLocker>
#include <QThread>
#include <gpiod.h>
#include <iostream>
#include <unistd.h>

#include <cerrno>
#include <csignal>
#include <cstdio>
#include <cstdint>
#include <cstring>

#include "mcp23017_rpi.h"
#include "modbus_rs485_rpi/modbus_rs485_rpi.h"
#include "cwt_sl_lth_rpi/cwt_sl_lth_rpi.h"
#include "sn3002_co2_rpi/sn3002_co2_rpi.h"
#include "xl9535_rpi/xl9535_rpi.h"
#include "flow_sensor_rpi/flow_sensor_rpi.h"

static volatile sig_atomic_t keepRunning = 1;

modbus_rs485_rpi_bus_t bus = {};
cwt_sl_lth_rpi_handle_t sensor = nullptr;
sn3002_co2_rpi_handle_t co2Sensor = nullptr;
static QMutex modbusReadMutex;

static void read_cwt_sl_lth_temp_humidity(cwt_sl_lth_rpi_handle_t sensor);
static void read_sn3002_co2(sn3002_co2_rpi_handle_t sensor);
static bool cycle_xl9535_relays(xl9535_rpi_t *relayPanel);
static void test_flow_sensor(void);

class SensorThread : public QThread
{
protected:
    void run() override
    {
        while (keepRunning)
        {
            qDebug() << "Reading sensor...";
           read_cwt_sl_lth_temp_humidity(sensor);
            QThread::sleep(5); // Sleep for 5 seconds
        }
    }
};

class Co2SensorThread : public QThread
{
protected:
    void run() override
    {
        while (keepRunning)
        {
            qDebug() << "Reading CO2 sensor...";
            read_sn3002_co2(co2Sensor);
            QThread::sleep(5); // Sleep for 5 seconds
        }
    }
};

class RelayTestThread : public QThread
{
public:
    explicit RelayTestThread(xl9535_rpi_t *relayPanel)
        : relayPanel(relayPanel)
    {
    }

protected:
    void run() override
    {
        while (keepRunning)
        {
            if (!cycle_xl9535_relays(relayPanel)) {
                keepRunning = 0;
                break;
            }
        }
    }

private:
    xl9535_rpi_t *relayPanel;
};

class FlowSensorThread : public QThread
{
protected:
    void run() override
    {
        test_flow_sensor();
    }
};

static void handleSignal(int signalNumber)
{
    Q_UNUSED(signalNumber);
    keepRunning = 0;
}

static QString errorText(int errorCode)
{
    return QString::fromLocal8Bit(std::strerror(-errorCode));
}

static bool configureLed(mcp23017_rpi_t *device, quint8 address)
{
    const int err = mcp23017_rpi_set_pin_dir(device,
                                             MCP23017_RPI_PIN_B7,
                                             MCP23017_RPI_PIN_DIR_OUTPUT);
    if (err < 0) {
        qCritical() << "Failed to configure GPB7 as output on expander"
                    << Qt::hex << address << Qt::dec
                    << "error" << err << errorText(err);
        return false;
    }

    return true;
}

static bool writeLed(mcp23017_rpi_t *device,
                     quint8 address,
                     mcp23017_rpi_pin_level_t level)
{
    const int err = mcp23017_rpi_write_pin(device, MCP23017_RPI_PIN_B7, level);
    if (err < 0) {
        qCritical() << "Failed to write GPB7 on expander"
                    << Qt::hex << address << Qt::dec
                    << "error" << err << errorText(err);
        return false;
    }

    return true;
}

static bool configure_xl9535_relay_panel(xl9535_rpi_t *relayPanel, quint8 address)
{
    int err = xl9535_rpi_write_port(relayPanel, XL9535_RPI_PORT_0, 0x00);
    if (err < 0) {
        qCritical() << "Failed to set all XL9535 relays OFF at"
                    << Qt::hex << address << Qt::dec
                    << "error" << err << errorText(err);
        return false;
    }

    err = xl9535_rpi_write_register(relayPanel, XL9535_REGISTER_CONFIG_0, 0x00);
    if (err < 0) {
        qCritical() << "Failed to configure XL9535 port 0 as relay outputs at"
                    << Qt::hex << address << Qt::dec
                    << "error" << err << errorText(err);
        return false;
    }

    return true;
}

static bool cycle_xl9535_relays(xl9535_rpi_t *relayPanel)
{
    int err = xl9535_rpi_write_port(relayPanel, XL9535_RPI_PORT_0, 0x01);
    if (err < 0) {
        qCritical() << "Failed to turn XL9535 relay channel 0 ON, error"
                    << err << errorText(err);
        return false;
    }

    qInfo() << "XL9535 relay channel 0 ON";
    QThread::msleep(500);

    // err = xl9535_rpi_write_port(relayPanel, XL9535_RPI_PORT_0, 0x00);
    // if (err < 0) {
    //     qCritical() << "Failed to turn XL9535 relay channel 0 OFF, error"
    //                 << err << errorText(err);
    //     return false;
    // }

    qInfo() << "XL9535 relay channel 0 OFF";
    QThread::msleep(500);
    return true;
}

static bool init_cwt_sl_lth_sensor(modbus_rs485_rpi_bus_t *bus,
                                   cwt_sl_lth_rpi_handle_t *sensor)
{
    int err = modbus_rs485_rpi_open(bus, "/dev/serial0", 4800);
    if (err != MODBUS_RS485_RPI_OK) {
        const int openErrno = errno;
        std::printf("Failed to open RS485 bus on /dev/serial0, error %d, errno %d (%s)\n",
                    err,
                    openErrno,
                    std::strerror(openErrno));
        std::fflush(stdout);
        return false;
    }

    cwt_sl_lth_rpi_config_t config = {};
    config.bus = bus;
    config.slave_id = 1;
    config.lux_mode = CWT_SL_LTH_RPI_LUX_SIMPLE_1X;

    err = cwt_sl_lth_rpi_create(&config, sensor);
    if (err != CWT_SL_LTH_RPI_OK) {
        std::printf("Failed to create CWT-SL-LTH sensor handle, error %d\n", err);
        std::fflush(stdout);
        modbus_rs485_rpi_close(bus);
        return false;
    }

    return true;
}

static bool init_sn3002_co2_sensor(modbus_rs485_rpi_bus_t *bus,
                                   sn3002_co2_rpi_handle_t *sensor)
{
    sn3002_co2_rpi_config_t config = {};
    config.bus = bus;
    config.slave_id = 3;

    const int err = sn3002_co2_rpi_create(&config, sensor);
    if (err != SN3002_CO2_RPI_OK) {
        std::printf("Failed to create SN3002 CO2 sensor handle, error %d\n", err);
        std::fflush(stdout);
        return false;
    }

    return true;
}

static void cleanup_rs485_sensors()
{
    if (co2Sensor != nullptr) {
        sn3002_co2_rpi_destroy(co2Sensor);
        co2Sensor = nullptr;
    }

    if (sensor != nullptr) {
        cwt_sl_lth_rpi_destroy(sensor);
        sensor = nullptr;
    }

    modbus_rs485_rpi_close(&bus);
}

static void read_cwt_sl_lth_temp_humidity(cwt_sl_lth_rpi_handle_t sensor)
{
    float temperature = 0.0f;
    float humidity = 0.0f;

    QMutexLocker locker(&modbusReadMutex);
    const int err = cwt_sl_lth_rpi_read_temp_hum(sensor, &temperature, &humidity);
    if (err == CWT_SL_LTH_RPI_OK) {
        std::printf("CWT-SL-LTH Temperature: %.1f C\n", temperature);
        std::printf("CWT-SL-LTH Humidity: %.1f %%\n", humidity);
    } else {
        std::printf("Failed to read CWT-SL-LTH temperature/humidity, error %d\n", err);
    }

    std::fflush(stdout);
}

static void read_sn3002_co2(sn3002_co2_rpi_handle_t sensor)
{
    uint32_t co2Ppm = 0;

    QMutexLocker locker(&modbusReadMutex);
    const int err = sn3002_co2_rpi_read(sensor, &co2Ppm);
    if (err == SN3002_CO2_RPI_OK) {
        std::printf("SN3002 CO2: %lu ppm\n", static_cast<unsigned long>(co2Ppm));
    } else {
        std::printf("Failed to read SN3002 CO2, error %d\n", err);
    }

    std::fflush(stdout);
}

static void test_flow_sensor(void)
{
    constexpr int flowGpio = 17;
    constexpr uint8_t flowSensorId = 1;
    flow_sensor_rpi_handle_t flow = nullptr;
    flow_sensor_rpi_config_t cfg = {};

    cfg.gpio = flowGpio;
    cfg.sensor_id = flowSensorId;
    cfg.calibration_factor = 7.5f;
    cfg.gpiochip = FLOW_SENSOR_RPI_DEFAULT_GPIOCHIP;

    int err = flow_sensor_rpi_init(&cfg, &flow);
    if (err != FLOW_SENSOR_RPI_OK) {
        std::printf("Failed to init flow sensor on BCM GPIO%d, error %d\n",
                    flowGpio,
                    err);
        std::fflush(stdout);
        return;
    }

    while (keepRunning) {
        float hz = 0.0f;
        float lmin = 0.0f;
        uint64_t totalPulses = 0;

        sleep(1);

        err = flow_sensor_rpi_get_frequency(flow, &hz);
        if (err != FLOW_SENSOR_RPI_OK) {
            std::printf("Failed to read flow frequency, error %d\n", err);
            std::fflush(stdout);
            break;
        }

        err = flow_sensor_rpi_get_flow_rate_l_min(flow, &lmin);
        if (err != FLOW_SENSOR_RPI_OK) {
            std::printf("Failed to read flow rate, error %d\n", err);
            std::fflush(stdout);
            break;
        }

        err = flow_sensor_rpi_get_total_pulses(flow, &totalPulses);
        if (err != FLOW_SENSOR_RPI_OK) {
            std::printf("Failed to read flow total pulses, error %d\n", err);
            std::fflush(stdout);
            break;
        }

        std::printf("Flow sensor %u: %.2f Hz, %.2f L/min, total pulses %llu\n",
                    static_cast<unsigned>(flowSensorId),
                    hz,
                    lmin,
                    static_cast<unsigned long long>(totalPulses));
        std::fflush(stdout);
    }

    flow_sensor_rpi_deinit(flow);
}


int main(int argc, char *argv[])
{
    QCoreApplication app(argc, argv);
    const char *chipname = "/dev/gpiochip0";
    const unsigned int line_num = 17; // BCM GPIO17

    std::signal(SIGINT, handleSignal);
    std::signal(SIGTERM, handleSignal);

    // gpiod_chip *chip = gpiod_chip_open(chipname);
    // if (!chip) {
    //     std::cerr << "Failed to open GPIO chip\n";
    //     return 1;
    // }

    // gpiod_line_settings *settings = gpiod_line_settings_new();
    // if (!settings) {
    //     std::cerr << "Failed to create GPIO line settings\n";
    //     gpiod_chip_close(chip);
    //     return 1;
    // }

    // int ret = gpiod_line_settings_set_direction(settings, GPIOD_LINE_DIRECTION_OUTPUT);
    // if (ret < 0) {
    //     std::cerr << "Failed to set GPIO line direction\n";
    //     gpiod_line_settings_free(settings);
    //     gpiod_chip_close(chip);
    //     return 1;
    // }

    // ret = gpiod_line_settings_set_output_value(settings, GPIOD_LINE_VALUE_INACTIVE);
    // if (ret < 0) {
    //     std::cerr << "Failed to set initial GPIO line value\n";
    //     gpiod_line_settings_free(settings);
    //     gpiod_chip_close(chip);
    //     return 1;
    // }

    // gpiod_line_config *line_config = gpiod_line_config_new();
    // if (!line_config) {
    //     std::cerr << "Failed to create GPIO line config\n";
    //     gpiod_line_settings_free(settings);
    //     gpiod_chip_close(chip);
    //     return 1;
    // }

    // ret = gpiod_line_config_add_line_settings(line_config, &line_num, 1, settings);
    // if (ret < 0) {
    //     std::cerr << "Failed to add GPIO line settings\n";
    //     gpiod_line_config_free(line_config);
    //     gpiod_line_settings_free(settings);
    //     gpiod_chip_close(chip);
    //     return 1;
    // }

    // gpiod_request_config *request_config = gpiod_request_config_new();
    // if (!request_config) {
    //     std::cerr << "Failed to create GPIO request config\n";
    //     gpiod_line_config_free(line_config);
    //     gpiod_line_settings_free(settings);
    //     gpiod_chip_close(chip);
    //     return 1;
    // }

    // gpiod_request_config_set_consumer(request_config, "blink-led");

    // gpiod_line_request *request = gpiod_chip_request_lines(chip, request_config, line_config);
    // gpiod_request_config_free(request_config);
    // gpiod_line_config_free(line_config);
    // gpiod_line_settings_free(settings);

    // if (!request) {
    //     std::cerr << "Failed to request GPIO line as output\n";
    //     gpiod_chip_close(chip);
    //     return 1;
    // }

    // qInfo() << "BlinkLed started";
    // qInfo() << "Using GPIO17";

    // int exit_code = 0;
    // while (keepRunning) {
    //     if (gpiod_line_request_set_value(request, line_num, GPIOD_LINE_VALUE_ACTIVE) < 0) {
    //         std::cerr << "Failed to set GPIO line active\n";
    //         exit_code = 1;
    //         break;
    //     }
    //     std::cout << "LED ON\n";
    //     sleep(1);

    //     if (!keepRunning) {
    //         break;
    //     }

    //     if (gpiod_line_request_set_value(request, line_num, GPIOD_LINE_VALUE_INACTIVE) < 0) {
    //         std::cerr << "Failed to set GPIO line inactive\n";
    //         exit_code = 1;
    //         break;
    //     }
    //     std::cout << "LED OFF\n";
    //     sleep(1);
    // }

    // (void)gpiod_line_request_set_value(request, line_num, GPIOD_LINE_VALUE_INACTIVE);
    // gpiod_line_request_release(request);
    // gpiod_chip_close(chip);

    // qInfo() << "BlinkLed stopped";
    // return exit_code;

    if (!init_cwt_sl_lth_sensor(&bus, &sensor)) {
        return 1;
    }

    if (!init_sn3002_co2_sensor(&bus, &co2Sensor)) {
        cleanup_rs485_sensors();
        return 1;
    }

    SensorThread sensorThread;
    Co2SensorThread co2SensorThread;
    FlowSensorThread flowSensorThread;

    constexpr const char *i2cDevice = MCP23017_RPI_DEFAULT_I2C_DEV;
    constexpr quint8 expander1Address = 0x22;
    constexpr quint8 expander2Address = 0x20;
    constexpr quint8 relayPanelAddress = 0x21;

    mcp23017_rpi_t expander1;
    mcp23017_rpi_t expander2;
    xl9535_rpi_t relayPanel;
    bool expander1Open = false;
    bool expander2Open = false;
    bool relayPanelOpen = false;
    int exitCode = 0;

    std::signal(SIGINT, handleSignal);
    std::signal(SIGTERM, handleSignal);

    qInfo() << "BlinkLed started";
    qInfo() << "Using MCP23017 expanders at 0x22 and 0x20 on" << i2cDevice;
    qInfo() << "Blinking GPB7 on both expanders";
    qInfo() << "Testing XL9535 relay channel 0 at 0x21 on" << i2cDevice;

    int err = mcp23017_rpi_open(&expander1, i2cDevice, expander1Address);
    if (err < 0) {
        qCritical() << "Failed to open expander 1 at 0x22, error" << err << errorText(err);
        cleanup_rs485_sensors();
        return 1;
    }
    expander1Open = true;

    err = mcp23017_rpi_open(&expander2, i2cDevice, expander2Address);
    if (err < 0) {
        qCritical() << "Failed to open expander 2 at 0x20, error" << err << errorText(err);
        mcp23017_rpi_close(&expander1);
        cleanup_rs485_sensors();
        return 1;
    }
    expander2Open = true;

    err = xl9535_rpi_open(&relayPanel, i2cDevice, relayPanelAddress);
    if (err < 0) {
        qCritical() << "Failed to open XL9535 relay panel at 0x21, error"
                    << err << errorText(err);
        mcp23017_rpi_close(&expander2);
        mcp23017_rpi_close(&expander1);
        cleanup_rs485_sensors();
        return 1;
    }
    relayPanelOpen = true;

    if (!configureLed(&expander1, expander1Address) ||
        !configureLed(&expander2, expander2Address) ||
        !configure_xl9535_relay_panel(&relayPanel, relayPanelAddress)) {
        exitCode = 1;
        keepRunning = 0;
    }

    RelayTestThread relayTestThread(&relayPanel);

    if (exitCode == 0) {
        sensorThread.start();
        co2SensorThread.start();
        flowSensorThread.start();
        relayTestThread.start();
    }

    while (keepRunning) {
        if (!writeLed(&expander1, expander1Address, MCP23017_RPI_PIN_LEVEL_HIGH) ||
            !writeLed(&expander2, expander2Address, MCP23017_RPI_PIN_LEVEL_HIGH)) {
            exitCode = 1;
            break;
        }

        qInfo() << "LEDs ON";

        QThread::msleep(500);
        if (!keepRunning) {
            break;
        }

        if (!writeLed(&expander1, expander1Address, MCP23017_RPI_PIN_LEVEL_LOW) ||
            !writeLed(&expander2, expander2Address, MCP23017_RPI_PIN_LEVEL_LOW)) {
            exitCode = 1;
            break;
        }

        qInfo() << "LEDs OFF";

        QThread::msleep(500);
    }

    if (expander1Open) {
        (void)mcp23017_rpi_write_pin(&expander1,
                                     MCP23017_RPI_PIN_B7,
                                     MCP23017_RPI_PIN_LEVEL_LOW);
        mcp23017_rpi_close(&expander1);
    }

    if (expander2Open) {
        (void)mcp23017_rpi_write_pin(&expander2,
                                     MCP23017_RPI_PIN_B7,
                                     MCP23017_RPI_PIN_LEVEL_LOW);
        mcp23017_rpi_close(&expander2);
    }

    keepRunning = 0;
    if (relayTestThread.isRunning()) {
        relayTestThread.wait();
    }
    if (sensorThread.isRunning()) {
        sensorThread.wait();
    }
    if (co2SensorThread.isRunning()) {
        co2SensorThread.wait();
    }
    if (flowSensorThread.isRunning()) {
        flowSensorThread.wait();
    }

    if (relayPanelOpen) {
        (void)xl9535_rpi_write_port(&relayPanel, XL9535_RPI_PORT_0, 0x00);
        xl9535_rpi_close(&relayPanel);
    }

    cleanup_rs485_sensors();

    qInfo() << "BlinkLed stopped";
    return exitCode;
}
