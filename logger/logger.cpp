#include "logger.h"
#include "vectornav/vectornavprotocol.h"
#include <QObject>
#include <QDebug>
#include <QFile>
#include <QDateTime>
extern double X[2000][2];
Logger::Logger(QObject *parent)
    : QObject{parent}{
}


void Logger::logStart() {
    if (writeLog == false) {
        QString fileName = "dataVectorNAV"+QString(" ")+QDate::currentDate().toString("yy-MM-dd")+QString(" ")+QTime::currentTime().toString("hh-mm-ss")+".txt";
        qDebug()<<fileName;
        file.setFileName(fileName);

        if (file.open(QIODevice::ReadWrite | QIODevice::Text)) {
            qDebug()<<"file is opened";
            writeLog = true;
        }
        else {
            //qDebug()<<"file open error : "<<QDate::currentDate().toString()+QTime::currentTime().toString()+".txt";
            qDebug()<<file.errorString() <<" " << file.error();
        }
        QTextStream stream2(&file);
        stream2 <<"k310 = " << K[310] << ", k311 = " << K[311] << ", k314 = " << K[314] << ", k315 = " << K[315] << ", k316 = " << K[316] << ", k317 = " << K[317] << ", k318 = " << K[318] << ", k322 = " << K[322] << ", k323 = " << K[323] << ", k325 = " << K[325] << ", k326 = " << K[326] << ", k328 = " << K[328] << ", k329 = " << K[329] << ", k330 = " << K[330] << "\n";
        stream2 <<"Time2, TimeStartup,  msg.yaw, msg.pitch, msg.roll, msg.X_rate, msg.Y_rate,  msg.Z_rate,  msg.X_accel,  msg.Y_accel,  msg.Z_accel, VMA1, VMA2, VMA3, VMA4, VMA5, VMA6, VMA7, VMA8, ControlSignalPsi, ControlSignalRoll, X[313][0], X[316][0], X[317][0], X[319][0], X[323][0], X[327][0]\n";

    }
}

void Logger::logTick(DataFromVectorNav msg) {
    if (writeLog) {
        QTextStream stream2(&file);
        stream2 << QTime::currentTime().toString("hh-mm-ss-zz") << " , " << msg.TimeStartup <<" , " <<  msg.yaw <<" , "<< msg.pitch<< " , " << msg.roll <<" , "<< msg.X_rate <<" , "<< \
        msg.Y_rate << " , "<< msg.Z_rate << " , " << msg.X_accel << " , "<< msg.Y_accel << " , " << msg.Z_accel<<" , " \
                << X[111][0]<<" , "<<X[121][0]<<" , "<<X[131][0]<<" , "<<X[141][0]<<" , "<<X[151][0]<<" , "<<X[161][0]<<" , "<<X[171][0]<<" , "<<X[181][0]<<" , "<<X[101][0]<<" , "<<X[103][0]<< " , "<<X[313][0]<< " , "<<X[316][0]<< " , "<<X[317][0]<< " , "<<X[319][0]<< " , "<<X[323][0]<< " , "<<X[327][0]<<  "\n";
//        qDebug()<<"stream2 << QTime::currentTime().toString";
    }
}

void Logger::logStop() {
    if (writeLog == true){
        writeLog = false;
        file.close();
    }

}