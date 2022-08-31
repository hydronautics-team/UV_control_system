#include <QCoreApplication>
#include "include_vn100/vectornavprotocol.h"
#include <QThread>
#include "kx_pult/kx_protocol.h"
#include "kx_pult/qkx_coeffs.h"
#include "pult_connection/pultcontrolsystemprotocols.h"

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
    ControlSystem::PC_Protocol *controlProtocol = new ControlSystem::PC_Protocol(QHostAddress("127.0.0.1"), 13020,
                                                                                 QHostAddress::LocalHost, 13021, 10);

    controlProtocol->send_data.imuData.ax = 1;
    controlProtocol->send_data.imuData.ay = 2;
    controlProtocol->send_data.imuData.az = 3;
    controlProtocol->send_data.imuData.gamma = 4;
    controlProtocol->send_data.imuData.psi = 5;
    controlProtocol->send_data.imuData.teta = 6;
    controlProtocol->send_data.imuData.q0 = 7;
    controlProtocol->send_data.imuData.q1 = 8;
    controlProtocol->send_data.imuData.q2 = 9;
    controlProtocol->send_data.imuData.q3 = 10;
    controlProtocol->send_data.depth = 11;
    controlProtocol->send_data.connectionFlags.controlSystem = 0;
    controlProtocol->send_data.connectionFlags.joystick = 1;
    controlProtocol->send_data.connectionFlags.thrusterController = 0;
    controlProtocol->send_data.connectionFlags.vectorNav = 1;

    qDebug() << "-----start exchange";
    controlProtocol->startExchange();

    return a.exec();
}
