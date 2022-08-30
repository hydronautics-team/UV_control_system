#ifndef PULTCONTROLSYSTEMPROTOCOLS_H
#define PULTCONTROLSYSTEMPROTOCOLS_H
#include "uv_state.h"
#include "udp_protocol.h"
extern double X[2000][2];

//класс обмена АНПА- планировщик, который создается в АНПА
namespace ControlSystem {
class PC_Protocol : public QObject, public MetaUdpProtocol {
    Q_OBJECT
public:
    explicit PC_Protocol(QHostAddress _receiverIP, int _receiverPort, QHostAddress _senderIP,\
                         int _senderPort,int freq, QObject *parent = 0){
        udpProtocol = new UdpProtocol <FromPult, ToPult> (_receiverIP,_receiverPort, _senderIP,_senderPort,\
                                                          freq, parent);
        connect(timer,SIGNAL(timeout()),SLOT(sendData()));
        connect(udpProtocol->getReceiveSocket(),SIGNAL(readyRead()),SLOT(receiveData()));
        set_ip_receiver(udpProtocol->ip_receiver());
        set_ip_sender (udpProtocol->ip_sender());
        set_port_receiver(udpProtocol->port_receiver());
        set_port_sender (udpProtocol->port_sender());
    }

    explicit PC_Protocol(const QString & config = "protocols.conf",
                           const QString & name = "rov_pult", QObject *parent = 0){
        udpProtocol = new UdpProtocol <FromPult, ToPult> (config, name, parent);
        connect(timer,SIGNAL(timeout()),SLOT(sendData()));
        connect(udpProtocol->getReceiveSocket(),SIGNAL(readyRead()),SLOT(receiveData()));
        set_ip_receiver(udpProtocol->ip_receiver());
        set_ip_sender (udpProtocol->ip_sender());
        set_port_receiver(udpProtocol->port_receiver());
        set_port_sender (udpProtocol->port_sender());
    }
signals:
    void dataReceived();
public slots:
    //запуск обмена
    void startExchange(){
        timer->start(1000/udpProtocol->getFrequency());
    }
    //остановить обмен
    void stopExhange(){
        timer->stop();
    }
    void sendData(){
 //       qDebug()<<"send data";
        udpProtocol->send_data = send_data;
        udpProtocol->sendData();
    }
    void receiveData(){
        udpProtocol->receiveData();
        rec_data = udpProtocol->rec_data;
        emit dataReceived();
        X[400][0] = rec_data.controlData.depth;
        X[401][0] = rec_data.controlData.lag;
        X[402][0] = rec_data.controlData.march;
        X[403][0] = rec_data.controlData.pitch;
        X[404][0] = rec_data.controlData.roll;
        X[405][0] = rec_data.controlData.yaw;
        X[406][0] = rec_data.controlContoursFlags.depth;
        X[407][0] = rec_data.controlContoursFlags.lag;
        X[408][0] = rec_data.controlContoursFlags.march;
        X[409][0] = rec_data.controlContoursFlags.pitch;
        X[410][0] = rec_data.controlContoursFlags.roll;
        X[411][0] = rec_data.controlContoursFlags.yaw;
        X[412][0] = static_cast<unsigned char>(rec_data.cSMode);
        X[413][0] ++;

    }
public:
    ToPult send_data;
    FromPult rec_data;
    UdpProtocol <FromPult, ToPult> *udpProtocol;

    bool bindState(){return udpProtocol->bindState();}
};
} //namespace ControlSystem


#endif // PULTCONTROLSYSTEMPROTOCOLS_H
