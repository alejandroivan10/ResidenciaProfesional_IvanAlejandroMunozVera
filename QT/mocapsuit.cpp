#include <QApplication>
#include <QUdpSocket>
#include "arduino.h"
#include "matlab.h"
#include "mocapsuit.h"
#include "wiringPi.h"
#include "RegistersAndMask.h"

extern QUdpSocket* mUdpSocket;

MocapSuit::MocapSuit(QObject* parent):
     QObject(parent)
{
    mTimer = new QTimer(this);
    connect(mTimer, SIGNAL(timeout()), this, SLOT(onTimeout()));
    connect(mUdpSocket, SIGNAL(readyRead()), this, SLOT(onRecieveUDP()));
    mUdpSocket->bind(63608, QUdpSocket::ShareAddress);
}

void MocapSuit::all_CS_toHigh(){
    Mpu9250* mpu;
    QVector<Mpu9250*>::iterator j;
    for(j=m_Sensores.begin(); j!=m_Sensores.end(); j++){
        mpu = *j;
        digitalWrite(mpu->CS, HIGH);
    }
}

void MocapSuit::addSensor(Mpu9250* _mpu)
{
    m_Sensores.append(_mpu);
}

void MocapSuit::setup()
{
    all_CS_toHigh();

//    while(true){
    fprintf(stderr, "WHO_AM_I: Si es igual a 0x71(113), es MPU9250; si es 0x73, es MPU9255; si es 0x70, es MPU6500 \n");
    Mpu9250* mpu;
    QVector<Mpu9250*>::iterator i;
    delay(10);
    for(i=m_Sensores.begin(); i!=m_Sensores.end(); i++){
        mpu = *i;
        uint8_t data = 0;
        MARGread(MPU9250_ADDR, mpu->CS, WHO_AM_I, 1, &data);
        fprintf(stderr, "WHO_AM_I sensor #%d = %d \n", mpu->id, data);
    }
//    delay(1000);
//    }

    while(wait4Start){
        QApplication::processEvents(QEventLoop::AllEvents);
        fprintf(stderr, "wait4Start is true...\n");
        delay(2000);
    }

    align_Tpose();//wait4Start = false; REVISAR

}

void MocapSuit::align_Tpose()
{
    Mpu9250* mpu;
    QVector<Mpu9250*>::iterator i;
    for(i=m_Sensores.begin(); i!=m_Sensores.end(); i++){
        mpu = *i;
        mpu->isAligned = false;
    }
    bool isSuitAligned = false;
    while(!isSuitAligned){
        for(i=m_Sensores.begin(); i!=m_Sensores.end(); i++){
            mpu = *i;
            if(mpu->isAligned == false){
                fprintf(stderr, "Sensor #%d: \n", mpu->id);
                mpu->loop(Mpu9250::Align);
            }
//            else{
//                fprintf(stderr, "Sensor #%d: \n", mpu->id);
//                mpu->loop(Mpu9250::NoImprimir);
//            }
            all_CS_toHigh();
        }
        fprintf(stderr, "\n\n");

        isSuitAligned = true;
        for(i=m_Sensores.begin(); i!=m_Sensores.end(); i++){
            mpu = *i;
            isSuitAligned = isSuitAligned && mpu->isAligned;
        }
        delay(10);
    }

    // Once wait4Start is false, then:
    QString c2 = "Start recived";
    QByteArray dataSnd(c2.toUtf8());
    mUdpSocket->writeDatagram(dataSnd, QHostAddress("192.168.1.126"), 63604);
    fprintf(stderr, "UPD recibido: Comienza alineación, comienza el programa");

}

void MocapSuit::loop()
{
    mTimer->start(500);
    while(true){
        QVector<Mpu9250*>::iterator i;
        Mpu9250* mpu;
        for(i=m_Sensores.begin(); i!=m_Sensores.end(); i++){
            mpu = *i;
            fprintf(stderr, "Sensor #%d: \n", mpu->id);
            mpu->loop();
            all_CS_toHigh();
        }
        QApplication::processEvents(QEventLoop::AllEvents);
        fprintf(stderr, "\n\n");
        delay(10);
    }
}

void MocapSuit::onTimeout()
{
    Vector SEeuler(3);
    QString datSEND = QString::number(m_Sensores.size())+"=";
    QVector<Mpu9250*>::iterator i;
    for(i=m_Sensores.begin(); i!=m_Sensores.end(); i++){
        Mpu9250* mpu = *i;
        SEeuler = RotMat2Euler(mpu->SErotM.t() * mpu->Align_Ajd);
        SEeuler = (SEeuler < 0.0f)*360.0f + SEeuler;
        SEeuler(1) = 360 - SEeuler(1);
        SEeuler(2) = 360 - SEeuler(2);

        datSEND = datSEND + String(SEeuler(0))+","+String(SEeuler(1))+","+String(SEeuler(2))+",";
    }
    datSEND = datSEND+" ";
    QByteArray prueba = QByteArray(datSEND.toUtf8());
    mUdpSocket->writeDatagram(prueba, QHostAddress("192.168.1.126"), 63604);
    fprintf(stderr, "%s\n", prueba.data());
}

void MocapSuit::onRecieveUDP(){
    fprintf(stderr, "UDP package Recieved...\n");
    while(mUdpSocket->hasPendingDatagrams()){
        QByteArray datagramaUDP;
        datagramaUDP.resize(mUdpSocket->pendingDatagramSize());
        mUdpSocket->readDatagram(datagramaUDP.data(), datagramaUDP.size());
        QString c1 = "Start program";
        if(QString(datagramaUDP.data()) == c1){
            wait4Start = false;
        }
    }
}
