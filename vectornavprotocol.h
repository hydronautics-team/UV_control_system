#ifndef VECTORNAVPROTOCOL_H
#define VECTORNAVPROTOCOL_H

#include <QObject>
#include <QTimer>
#include <stdio.h>
#include <unistd.h>
#include "include_vn100/vectornav.h"
#include <QThread>
#include <QDebug>

const char* const COM_PORT = "COM5";
const int BAUD_RATE = 115200;

//для использования kx-pult
extern double X[2000][2];
extern QVector<double> K;

class VectorNavProtocol : public QObject
{
    Q_OBJECT
public:
    VectorNavProtocol(QObject * prnt = nullptr):QObject(prnt){
        QThread * thread = new QThread();
        moveToThread(thread);
        thread->start();
        vn100_connect(&vn100, COM_PORT, BAUD_RATE);
        Sleep(1000);
        connect(&timer, &QTimer::timeout, this, &VectorNavProtocol::tick);
    }
    virtual ~VectorNavProtocol(){
        vn100_disconnect(&vn100);
    }
public slots:
    void start (int dt){
        timer.start(dt);
    }
    void stop(){
        timer.stop();
    }
    void tick(){
       //Vn100CompositeData data;
       //vn100_getCurrentAsyncData(&vn100, &data);
       vn100_getYawPitchRollTrueInertialAcclerationAngularRate(&vn100,&ypr,&inertialAcceleration,&angularRate);
       //vn100_getCalibratedImuMeasurements(&vn100,&magnetic,&inertialAcceleration,&angularRate, &temp);

       //kx-pult
        X[301][0] = ypr.yaw;
        X[302][0] = ypr.pitch;
        X[303][0] = ypr.roll;
        X[304][0] = inertialAcceleration.c0;
        X[305][0] = inertialAcceleration.c1;
        X[306][0] = inertialAcceleration.c2;
        X[307][0] = angularRate.c0;
        X[308][0] = angularRate.c1;
        X[309][0] = angularRate.c2;
        X[310][0] = K[1]; //теперь значение этой переменной будет таким же как принятый коэффициент K1
    }
protected:
    QTimer timer;
    Vn100 vn100;
    VnYpr ypr;
    VnVector3 inertialAcceleration;
    VnVector3 angularRate;
    VnVector3 magnetic;
    double temp;
};

#endif // VECTORNAVPROTOCOL_H
