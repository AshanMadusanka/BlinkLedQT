#include <QCoreApplication>
#include <QDebug>
#include <QThread>

#include <csignal>
#include <cstring>

#include "mcp23017_rpi.h"

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

int main(int argc, char *argv[])
{
    QCoreApplication app(argc, argv);

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
