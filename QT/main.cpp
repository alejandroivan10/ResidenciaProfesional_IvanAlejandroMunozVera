#include "mainwindow.h"
#include <QApplication>

#include <typeinfo>
#include "stdio.h"

#include <cmath>
#include <QString>
#include <QDebug>
#include <QUdpSocket>

#include "arduino.h"
#include "Complementos.h"
#include "matlab.h"
#include "mocapsuit.h"
#include "mpu9250.h"
#include "RegistersAndMask.h"
#include "SPI.h"


#include "mpu9250.h"

#include "wiringPi.h" //Borrar libreria

/**********************   CONEXIONES ARDUINO   *******************/
//GND - GND
//VCC - VCC
//SDA - Pin A4
//SCL - Pin A5

//SDA(MOSI) - Pin D11
//AD0(MISO) - Pin D12
//SCL(SCK)  - Pin D13
//NCS(CS)   - Pin D7 (arbitrario)




QUdpSocket* mUdpSocket;

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    MainWindow w;

    mUdpSocket = new QUdpSocket();
    MocapSuit MainSuit;
//    Mpu9250 Esternon(2, MainSuit, {9.819, 9.584, 10.071, 0.3, 0.131, -0.317}, {-1.6558, 2.0798, -0.3839},
//    {39.6496, 43.5980, 40.8667, -9.53, 11.987, 47.4591}, {-90, 80.52, -90}); /// Sensor #2
//    Mpu9250 ClaviculaIzq(3, MainSuit, {10.132, 9.990, 10.441, 0.736, 0.4, -1.93}, {3.5523,  0.6303,  2.2918},
//    {45.687, 44.505, 46.507, 22.771, 0.5005, 12.820}, {-3.94, -1.82, -26.13});  /// Sensor #3  {-1.39,0,0}
//    Mpu9250 BrazoIzq(21, MainSuit, {9.624, 9.603, 9.934, -6.046, -3.138, -6.3}, {0.0, 0.0, 0.0},
//    {40.687, 40.505, 40.507, -11.0, 12.0, 15.0}, {-1.39, 0.0, 0.0}); /// Sensor #5
//    Mpu9250 AntBrazoIzq(22, MainSuit, {9.624, 9.603, 9.934, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0},
//    {40.687, 40.505, 40.507, 0.0, 0.0, 0.0}, {-7.83, 0.0, 0.0}); /// Sensor #10 (Ant: MuñecaDer)
    Mpu9250 BrazoDer(23, MainSuit, {9.624, 9.603, 9.934, 0.15, -0.2, -0.6}, {0.0, 0.0, 0.0},
    {40.687, 40.505, 40.507, 4, 30.0, 17.5}, {178.61, 0.0, 0.0}); /// Sensor #11
//    Mpu9250 AntBrazoDer(24, MainSuit, {9.624, 9.603, 9.934, 0.0, -0.1, 0.0}, {0.0, 0.0, 0.0},
//    {40.687, 40.505, 40.507, -22.5, 23.5, 25.5}, {172.161, 0.0, 0.0}); /// Sensor #12

//    Mpu9250 sinDef4(4, MainSuit, {9.624, 9.603, 9.934, 0.046, -0.35, -0.55}, {1.2490, -1.9366, -1.2892},
//    {40.687, 40.505, 40.507, -6.5, 3.5, 31.5}, {178.61, 0.0, 0.0}); /// Sensor #4 (Ant: ClaviculaIzq)
//    Mpu9250 sinDef6(6, MainSuit, {9.624, 9.603, 9.934, 0.146, -0.138, 0.580}, {0.0, 0.0, 0.0},
//    {40.687, 40.505, 40.507, 10.5, 10.0, 8.0}, {-83.22, 5.67, 3.34}); /// Sensor #6 (Ant: AntBrazoIzq)
//    Mpu9250 sinDef7(7, MainSuit, {9.624, 9.603, 9.934, 0.046, -0.138, -0.480}, {0.0, 0.0, 0.0},
//    {40.687, 40.505, 40.507, 0.0, 0.0, 0.0}, {178.61, 0.0, 0.0});  /// Sensor #7 (Ant: MuñecaIzq)
//    Mpu9250 SinDef(21, MainSuit, {9.624, 9.603, 9.934, 0.0, 0.7, -0.5}, {0.0, 0.0, 0.0},
//    {40.687, 40.505, 40.507, 0.00, 0.0, 0.0}, {-7.83, 0.0, 0.0}); /// Sensor #8 (Ant: BrazoDer)
//    Mpu9250 sinDef9(7, MainSuit, {9.624, 9.603, 9.934, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0},
//    {40.687, 40.505, 40.507, 0.0, 0.0, 0.0}, {-83.22, 5.67, 3.34}); /// Sensor #9 (Ant: AntBrazoDer, luego Cabeza)


    wiringPiSetup();
    QVector<uint8_t> Unused = {2,3,4, 5,6,7, 21,22,23,24};
    for(int i = 0; i < Unused.size(); i++){
        pinMode(Unused[i], OUTPUT);
        digitalWrite(Unused[i], HIGH);
    }
    if (SPI.setClockDivider(SPI_CLOCK_DIV16) == -1){
        fprintf(stderr, "FATAL ERROR: SPI.setUp Failed \n");
        exit(0);
    }

//    Esternon.setup();
//    ClaviculaIzq.setup();
//    BrazoIzq.setup();
//    AntBrazoIzq.setup();
    BrazoDer.setup();
//    AntBrazoDer.setup();

    //    ClaviculaDer.setup();
//    MunecaIzq.setup();
//    MunecaDer.setup();
//    Back.setup();
//    sinDef9.setup();

//    sinDef.setup();

    MainSuit.setup();    
    delay(5000);
    MainSuit.loop();

    w.show();
    a.exec();

    delete mUdpSocket;
    return 0;
}








