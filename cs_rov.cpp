#include "cs_rov.h"

CS_ROV::CS_ROV(QObject *parent)
{
    //logger.logStart();
    vn100Proto = new VectorNavProtocol("ttyUSB0");
    QObject::connect(vn100Proto, &VectorNavProtocol::newMessageDetected,
                     &logger, &Logger::logTick);

    QSettings settings("settings/settings.ini", QSettings::IniFormat);
    settings.beginGroup("Port");
    QString port = settings.value("portname", "/dev/tty.usbserial-AK06UI59").toString();
    qint32 baudrate = settings.value("baudrate", 230400).toInt();
    settings.endGroup();
    vmaProtocol = new VMAController(port, baudrate);
    vmaProtocol->moveToThread(&vmaThread);
    QObject::connect(&vmaThread, &QThread::started, vmaProtocol, &VMAController::start);
    vmaThread.start();

    pultProtocol = new ControlSystem::PC_Protocol(ConfigFile,"rov_pult");

    qDebug() << "-----start exchange";
    pultProtocol->startExchange();

    connect(&timer, &QTimer::timeout, this, &CS_ROV::tick);
    timer.start(10);
    X[207][0] =X[209][0] = 0; //нулевые НУ для интегрирующего звена
}

void CS_ROV::tick()
{
    readDataFromPult();
    readDataFromSensors();
    regulators();
    BFS_DRK(X[101][0], X[102][0], X[103][0] , X[104][0], X[105][0], X[106][0]);
    writeDataToVMA();
    writeDataToPult();

}

void CS_ROV::resetValues()
{
    vmaProtocol->setValues(0, 0, 0, 0, 0,0,0,0,true);

}

void CS_ROV::readDataFromPult()
{
    X[91][0] = pultProtocol->rec_data.controlData.yaw;
    X[92][0] = pultProtocol->rec_data.controlData.pitch;
    X[93][0] = pultProtocol->rec_data.controlData.roll;
    X[94][0] = pultProtocol->rec_data.controlData.march;
    X[95][0] = pultProtocol->rec_data.controlData.lag;
    X[96][0] = pultProtocol->rec_data.controlData.depth;
    X[97][0] = pultProtocol->rec_data.thrusterPower;

    X[201][0] = pultProtocol->rec_data.sinTest.u0;
    X[202][0] = pultProtocol->rec_data.sinTest.a;
    X[203][0] = pultProtocol->rec_data.sinTest.w;
    X[204][0] = pultProtocol->rec_data.sinTest.k;
    X[205][0] = pultProtocol->rec_data.sinTest.h;

    changePowerOffFlag(pultProtocol->rec_data.thrusterPower);
    changeSinSignalFlag(pultProtocol->rec_data.sinTest.sinSignal);
    if (K[0] > 0) setModellingFlag(true);
    else setModellingFlag(false);
    if (pultProtocol->rec_data.experimentTypicalInput) logger.logStart();
    else logger.logStop();
}

void CS_ROV::readDataFromSensors()
{
    //kx-pult
     X[301][0] = vn100Proto->data.yaw;
     X[302][0] = vn100Proto->data.pitch;
     X[303][0] = vn100Proto->data.roll;
     X[304][0] = vn100Proto->data.X_rate; //wx
     X[305][0] = vn100Proto->data.Y_rate; //wy
     X[306][0] = vn100Proto->data.Z_rate; //wz

}

void CS_ROV::regulators()
{

    X[102][0] = K[102]*X[92][0];
    X[103][0] = K[103]*X[93][0];
    X[104][0] = K[104]*X[94][0];
    X[105][0] = K[105]*X[95][0];
    X[106][0] = K[106]*X[96][0];

    if (pultProtocol->rec_data.sinTest.sinSignal) {
        X[101][0] = K[101] * (X[201][0] + X[202][0]*sin(X[203][0]*timeForSinus.elapsed()*0.001));
                //X[101][0] = U0 + A*sin(w*k*h);
    }
    else {
        int t_shag=10;
        X[202][0] = X[91][0]-X[201][0]; //ошибка по углу
        X[203][0] = K[101]*X[202][0];
        X[204][0] = K[103]*X[91][0];
        if (K[105] == 0) {  //K[105] - флаг регулятора
           X[206][0] = K[102]*X[205][0];
           X[101][0] = X[203][0]+X[204][0]-X[206][0];
        }
        if (K[105] == 1) {
           //K[111]=121; коэффициент усиления звена
           //K[112]=0,1; //постоянная времени для апериодического звена
           //K[113]=0,126; //постоянная времени для дифференцирования
                X[206][0] = X[207][0]+ 10*0,001*(1/K[112])*(X[205][0]*K[111]-X[207][0]);
                X[207][0]= X[206][0];//запоминаем значение с этого шага для апериодич. звена
                X[208][0]= (K[113]*(X[206][0]-X[209][0])/(t_shag*0.001));
                X[209][0]= X[208][0];//запоминаем значение с этого шага для дифференцирования
                X[210][0]= X[208][0] + X[206][0];
           X[101][0] = X[203][0]+X[204][0]-X[210][0];
        }

    }

//    X[101][0] = K[1];//Upsi
//    X[102][0] = K[2];//Uteta
//    X[103][0] = K[3];//Ugamma
//    X[104][0] = K[4];//Ux
//    X[105][0] = K[5];//Uy
//    X[106][0] = K[6];//Uz
}

void CS_ROV::BFS_DRK(double Upsi, double Uteta, double Ugamma, double Ux, double Uy, double Uz)
{
    X[110][0] = (K[10]*Ux + K[11]*Uy + K[12]*Uz + K[13]*Ugamma + K[14]*Uteta + K[15]*Upsi)*K[16];//U1
    X[120][0] = (K[20]*Ux + K[21]*Uy + K[22]*Uz + K[23]*Ugamma + K[24]*Uteta + K[25]*Upsi)*K[26];//U2
    X[130][0] = (K[30]*Ux + K[31]*Uy + K[32]*Uz + K[33]*Ugamma + K[34]*Uteta + K[35]*Upsi)*K[36];//U3
    X[140][0] = (K[40]*Ux + K[41]*Uy + K[42]*Uz + K[43]*Ugamma + K[44]*Uteta + K[45]*Upsi)*K[46];//U4
    X[150][0] = (K[50]*Ux + K[51]*Uy + K[52]*Uz + K[53]*Ugamma + K[54]*Uteta + K[55]*Upsi)*K[56];//U5
    X[160][0] = (K[60]*Ux + K[61]*Uy + K[62]*Uz + K[63]*Ugamma + K[64]*Uteta + K[65]*Upsi)*K[66];//U6
    X[170][0] = (K[70]*Ux + K[71]*Uy + K[72]*Uz + K[73]*Ugamma + K[74]*Uteta + K[75]*Upsi)*K[76];//U7
    X[180][0] = (K[80]*Ux + K[81]*Uy + K[82]*Uz + K[83]*Ugamma + K[84]*Uteta + K[85]*Upsi)*K[86];//U8

    X[111][0] = limit(X[110][0],K[100]);
    X[121][0] = limit(X[120][0],K[100]);
    X[131][0] = limit(X[130][0],K[100]);
    X[141][0] = limit(X[140][0],K[100]);
    X[151][0] = limit(X[150][0],K[100]);
    X[161][0] = limit(X[160][0],K[100]);
    X[171][0] = limit(X[170][0],K[100]);
    X[181][0] = limit(X[180][0],K[100]);


}

void CS_ROV::writeDataToPult()
{
    pultProtocol->send_data.imuData.psi = X[301][0];
    pultProtocol->send_data.imuData.teta = X[302][0];
    pultProtocol->send_data.imuData.gamma = X[303][0];
    pultProtocol->send_data.imuData.wx = X[304][0];
    pultProtocol->send_data.imuData.wy = X[305][0];
    pultProtocol->send_data.imuData.wz = X[306][0];
}

void CS_ROV::changePowerOffFlag(qint8 flag)
{
    if (vmaPowerOffFlag!=static_cast<bool>(flag)) {
        vmaPowerOffFlag = static_cast<bool>(flag);
        resetValues();
    }

}

void CS_ROV::changeSinSignalFlag(qint8 sinflag)
{
    if (generationSinFlag!=sinflag){
        if (sinflag) timeForSinus.start();
        generationSinFlag = sinflag;
    }
}

void CS_ROV::setModellingFlag(bool flag)
{
    if (modellingFlag!=flag) {
        if (modellingFlag == false) resetValues();
        modellingFlag = flag;
    }
}

void CS_ROV::writeDataToVMA()
{
    if (modellingFlag) {//режим модели
        model.tick(X[111][0], X[121][0], X[131][0], X[141][0], X[151][0], X[161][0], X[171][0], X[181][0], 0.01);
    }
    else {
      vmaProtocol->setValues(X[111][0], X[161][0], X[121][0], X[151][0], X[131][0], X[181][0], X[171][0], X[141][0], vmaPowerOffFlag);
//      qDebug() << X[111][0];
//      qDebug() << vmaPowerOffFlag;
    }
}

