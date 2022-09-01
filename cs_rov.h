#ifndef CS_ROV_H
#define CS_ROV_H

#include "rov_model.h"
#include "include_vn100/vectornavprotocol.h"
#include <QThread>
#include <QSettings>
#include "pult_connection/pultcontrolsystemprotocols.h"
#include "vma/vmacontroller.h"
#include "math.h"
#include <qmath.h>

const QString ConfigFile = "protocols.conf";
const QString XI = "xi";
const QString KI = "ki";

class CS_ROV : public QObject
{
    Q_OBJECT
public:
    CS_ROV(QObject * parent = nullptr);

    void start(int dt){
        timer.start(dt);
    }
public slots:
    void tick();
public:
    double limit (double value, double limit){
        if(fabs(value)>limit) return (limit*sgn(value));
        else return value;
    }
    template <typename T> int sgn(T val) {
        return (T(0) < val) - (val < T(0));
    }
protected:
    void readDataFromPult();
    void readDataFromSensors();
    void regulators();
    void BFS_DRK(double Upsi, double Uteta, double Ugamma, double Ux, double Uy, double Uz);
    void writeDataForVMA();
    VectorNavProtocol *vn100Proto = nullptr;
    VMAController* vmaProtocol = nullptr;
    //обмен с пультом
    ControlSystem::PC_Protocol *pultProtocol = nullptr;
    ROV_Model model;
    QTimer timer;
    bool vmaPowerFlag;
};

#endif // CS_ROV_H