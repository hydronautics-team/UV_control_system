#include <QCoreApplication>
#include "vectornavprotocol.h"
#include <QThread>
#include "kx_pult/kx_protocol.h"
#include "kx_pult/qkx_coeffs.h"
#include "pultcontrolsystemprotocols.h"

const QString ConfigFile = "protocols.conf";
const QString XI = "xi";
const QString KI = "ki";

double X[2000][2];

int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);
    //передача K
    Qkx_coeffs * kProtocol = new Qkx_coeffs(ConfigFile, KI);
    //передача X
    x_protocol * xProtocol = new x_protocol(ConfigFile,XI,X);
    //обмен с VectorNav (там внутри пример с примененем X-ов)
    VectorNavProtocol vn100Proto;
    vn100Proto.start(100);
    //обмен с пультом
    ControlSystem::PC_Protocol *pultProtocol = new ControlSystem::PC_Protocol(ConfigFile,"rov_pult");
    pultProtocol->send_data.imuData.ax = 1;
    pultProtocol->send_data.imuData.ay = 2;
    pultProtocol->send_data.imuData.az = 3;
    pultProtocol->send_data.imuData.gamma = 4;
    pultProtocol->send_data.imuData.psi = 5;
    pultProtocol->send_data.imuData.teta = 6;
    pultProtocol->send_data.imuData.q0 = 7;
    pultProtocol->send_data.imuData.q1 = 8;
    pultProtocol->send_data.imuData.q2 = 9;
    pultProtocol->send_data.imuData.q3 = 10;
    pultProtocol->send_data.depth = 11;
    pultProtocol->send_data.connectionFlags.controlSystem = 0;
    pultProtocol->send_data.connectionFlags.joystick = 1;
    pultProtocol->send_data.connectionFlags.thrusterController = 0;
    pultProtocol->send_data.connectionFlags.vectorNav = 1;
    return a.exec();
}
