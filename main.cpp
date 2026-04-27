#include <QCoreApplication>
#include <QDebug>
#include <QThread>

#include <cerrno>
#include <csignal>
#include <cstdio>
#include <cstring>

#include "mcp23017_rpi.h"
#include "modbus_rs485_rpi/modbus_rs485_rpi.h"
#include "cwt_sl_lth_rpi/cwt_sl_lth_rpi.h"

static volatile sig_atomic_t keepRunning = 1;

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

static void test_cwt_sl_lth_temp_humidity(void)
{
    qInfo() << "Testing CWT-SL-LTH temperature/humidity on /dev/serial0";

    modbus_rs485_rpi_bus_t bus = {};
    int err = modbus_rs485_rpi_open(&bus, "/dev/serial0", 4800);
    if (err != MODBUS_RS485_RPI_OK) {
        const int openErrno = errno;
        std::printf("Failed to open RS485 bus on /dev/serial0, error %d, errno %d (%s)\n",
                    err,
                    openErrno,
                    std::strerror(openErrno));
        std::fflush(stdout);
        return;
    }

    cwt_sl_lth_rpi_config_t config = {};
    config.bus = &bus;
    config.slave_id = 1;
    config.lux_mode = CWT_SL_LTH_RPI_LUX_SIMPLE_1X;

    cwt_sl_lth_rpi_handle_t sensor = nullptr;
    err = cwt_sl_lth_rpi_create(&config, &sensor);
    if (err != CWT_SL_LTH_RPI_OK) {
        std::printf("Failed to create CWT-SL-LTH sensor handle, error %d\n", err);
        std::fflush(stdout);
        modbus_rs485_rpi_close(&bus);
        return;
    }

    float temperature = 0.0f;
    float humidity = 0.0f;

    err = cwt_sl_lth_rpi_read_temp_hum(sensor, &temperature, &humidity);
    if (err == CWT_SL_LTH_RPI_OK) {
        std::printf("CWT-SL-LTH Temperature: %.1f C\n", temperature);
        std::printf("CWT-SL-LTH Humidity: %.1f %%\n", humidity);
    } else {
        std::printf("Failed to read CWT-SL-LTH temperature/humidity, error %d\n", err);

        uint8_t resp[9] = {};
        const int modbusErr = modbus_rs485_rpi_read_holding(&bus,
                                                            1,
                                                            0x0000,
                                                            2,
                                                            resp,
                                                            sizeof(resp),
                                                            2000);
        std::printf("Direct Modbus temp/humidity read error %d\n", modbusErr);
    }
    std::fflush(stdout);

    cwt_sl_lth_rpi_destroy(sensor);
    modbus_rs485_rpi_close(&bus);
}

int main(int argc, char *argv[])
{
    QCoreApplication app(argc, argv);

    test_cwt_sl_lth_temp_humidity();

    constexpr const char *i2cDevice = MCP23017_RPI_DEFAULT_I2C_DEV;
    constexpr quint8 expander1Address = 0x22;
    constexpr quint8 expander2Address = 0x20;

    mcp23017_rpi_t expander1;
    mcp23017_rpi_t expander2;
    bool expander1Open = false;
    bool expander2Open = false;
    int exitCode = 0;

    std::signal(SIGINT, handleSignal);
    std::signal(SIGTERM, handleSignal);

    qInfo() << "BlinkLed started";
    qInfo() << "Using MCP23017 expanders at 0x22 and 0x20 on" << i2cDevice;
    qInfo() << "Blinking GPB7 on both expanders";

    int err = mcp23017_rpi_open(&expander1, i2cDevice, expander1Address);
    if (err < 0) {
        qCritical() << "Failed to open expander 1 at 0x22, error" << err << errorText(err);
        return 1;
    }
    expander1Open = true;

    err = mcp23017_rpi_open(&expander2, i2cDevice, expander2Address);
    if (err < 0) {
        qCritical() << "Failed to open expander 2 at 0x20, error" << err << errorText(err);
        mcp23017_rpi_close(&expander1);
        return 1;
    }
    expander2Open = true;

    if (!configureLed(&expander1, expander1Address) ||
        !configureLed(&expander2, expander2Address)) {
        exitCode = 1;
        keepRunning = 0;
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

    qInfo() << "BlinkLed stopped";
    return exitCode;
}
