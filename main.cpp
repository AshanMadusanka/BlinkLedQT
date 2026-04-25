#include <QCoreApplication>
#include <QProcess>
#include <QThread>
#include <QDebug>

bool runCommand(const QStringList &args)
{
    QProcess process;
    process.start("pinctrl", args);
    process.waitForFinished();

    QString output = process.readAllStandardOutput();
    QString error = process.readAllStandardError();

    if (!output.isEmpty())
        qDebug() << output;

    if (!error.isEmpty())
        qDebug() << error;

    return process.exitCode() == 0;
}

int main(int argc, char *argv[])
{
    QCoreApplication app(argc, argv);

    qDebug() << "BlinkLed started";
    qDebug() << "Using GPIO17";

    if (!runCommand({"set", "17", "op"})) {
        qDebug() << "Failed to set GPIO17 as output";
        return -1;
    }

    while (true) {
        qDebug() << "LED ON";
        runCommand({"set", "17", "dh"});
        QThread::msleep(500);

        qDebug() << "LED OFF";
        runCommand({"set", "17", "dl"});
        QThread::msleep(500);
    }

    return app.exec();
}
