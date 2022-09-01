#pragma once
#include <QSerialPort>
#include <QByteArray>
#include <QObject>
#include <QMutex>
#include <QTimer>

//для использования kx-pult
extern double X[2000][2];
extern QVector<double> K;


class VMAController: public QObject {
    Q_OBJECT

public:
    VMAController(const QString& portName,
                  const qint32 baud,
                  const QSerialPort::DataBits data = QSerialPort::Data8,
                  const QSerialPort::StopBits stop = QSerialPort::OneStop,
                  const QSerialPort::Parity parity = QSerialPort::EvenParity,
                  const QSerialPort::FlowControl flow = QSerialPort::NoFlowControl,
                  QObject* parent = nullptr);
    virtual ~VMAController();

public slots:
    virtual void start();
    virtual void stop();
    void setValues(const float Upnp,const float Upnl,const float Uznl,const float Uznp, const float Upvp, const float Upvl, const float Uzvl, const float Uzvp, bool powerFlag);
signals:
    void started();
    void finished();

private:
    QSerialPort* mSerialPort = new QSerialPort(this);
    QString mPortName;
    qint32 mBaud;
    QSerialPort::DataBits mDataBits;
    QSerialPort::StopBits mStopBits;
    QSerialPort::Parity mParity;
    QSerialPort::FlowControl mFlow;
    qint16 vmaVector[9];
    QTimer mTimer;
    bool isConnected = false;
    mutable QMutex mGuard;
    static constexpr int REQUEST_TIME = 100;
    static constexpr int PACKET_SIZE = 22;
    bool sendData();
    quint16 calculate_crc(QByteArray array);

private slots:
    void tick();
};
