#include "vmacontroller.h"
#include <QSerialPort>
#include <QIODevice>
#include <QDebug>
#include "math.h"

VMAController::VMAController(const QString& portName,
                             const qint32 baud,
                             const QSerialPort::DataBits data,
                             const QSerialPort::StopBits stop,
                             const QSerialPort::Parity parity,
                             const QSerialPort::FlowControl flow,
                             QObject* parent) :
    mPortName(portName),
    mBaud(baud),
    mDataBits(data),
    mStopBits(stop),
    mParity(parity),
    mFlow(flow),
    mTimer(this) {
    connect(&mTimer, &QTimer::timeout, this, &VMAController::tick);
}

void VMAController::start() {
    QMutexLocker lock(&mGuard);
    if (mSerialPort->isOpen()) {
        qDebug() << "Duplicate open request";
        return;
    }
    mSerialPort->setPortName(mPortName);
    mSerialPort->setBaudRate(mBaud);
    mSerialPort->setDataBits(mDataBits);
    mSerialPort->setStopBits(mStopBits);
    mSerialPort->setParity(mParity);
    mSerialPort->setFlowControl(mFlow);
    if (!mSerialPort->open(QIODevice::ReadWrite)) {
        qDebug() << "Error connecting to MCU" << mSerialPort->errorString();
        return;
    }
    mTimer.start(REQUEST_TIME);
    qDebug() << "Port opened";
    isConnected = true;
    emit started();
    return;
}

void VMAController::tick() {
    sendData();
    X[55][0]++;
}

bool VMAController::sendData() {
    bool ret  = true;
    QByteArray packet;

    packet.append(0xff);
    packet.append(0xfd);
    for (int i = 0; i < 9; ++i) {
        packet.append((const char*)(vmaVector + i), sizeof(qint16));
    }
    qint16  crc = calculate_crc(packet);
    qint8 crc_low = crc & 0xff;
    qint8 crc_high = (crc >> 8);
    packet.append(crc_low);
    packet.append(crc_high);
    qint64 bytesWritten = mSerialPort->write(packet);
    if ((bytesWritten != packet.size())) {
        qDebug() << mSerialPort->errorString();
        ret = false;
    }
    packet.clear();
    return ret;
}

quint16 VMAController::calculate_crc(QByteArray array) {

    int len = array.size();
    quint16 wcrc = 0xFFFF; // preset 16 position crc register , The initial values are all 1
    quint8 temp;// Define intermediate variables
    int i = 0, j = 0; // Define count
    for (i = 0; i < len; i++) { // Cycle through each data

        temp = array.at(i);
        wcrc ^= temp;
        for (j = 0; j < 8; j++) {

            // Judge whether what is moved to the right is 1, If it is 1 XOR with polynomials .
            if (wcrc & 0x0001) {

                wcrc >>= 1; // First move the data one bit to the right
                wcrc ^= 0xA001; // XOR with the polynomial above
            } else // If not 1, Then directly remove
                wcrc >>= 1; // Direct removal
        }
    }
    temp = wcrc; //crc Value
    return wcrc;
}

void VMAController::stop() {
    QMutexLocker lock(&mGuard);
    if (mSerialPort && mSerialPort->isOpen())
        mSerialPort->close();
    emit finished();
}

void VMAController::setValues(const float Upnp, const float Upnl, const float Uznl, const float Uznp, const float Upvp, const float Upvl, const float Uzvl, const float Uzvp, bool powerFlag)
{
    double scale=2;
    if (K[99]!=0) scale = K[99];
    X[80][0]=vmaVector[0] = round(Upnp/scale)+K[98];
    X[81][0]=vmaVector[1] = round(Upnl/scale)+K[98];
    X[82][0]=vmaVector[2] = round(Uznl/scale)+K[98];
    X[83][0]=vmaVector[3] = round(Uznp/scale)+K[98];
    X[84][0]=vmaVector[4] = round(Upvp/scale)+K[98];
    X[85][0]=vmaVector[5] = round(Upvl/scale)+K[98];
    X[86][0]=vmaVector[6] = round(Uzvl/scale)+K[98];
    X[87][0]=vmaVector[7] = round(Uzvp/scale)+K[98];
    (powerFlag)? vmaVector[8] = 0b10000000 : vmaVector[8]=0;
    X[88][0]=vmaVector[8];
}

VMAController::~VMAController() {
    stop();
}
