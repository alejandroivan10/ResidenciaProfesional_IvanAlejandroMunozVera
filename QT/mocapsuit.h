#ifndef MOCAPSUIT_H
#define MOCAPSUIT_H

#include "mpu9250.h"
#include <QVector>
#include <QObject>
#include <QTimer>

class MocapSuit : public QObject
{
    Q_OBJECT
public:
    explicit MocapSuit(QObject* parent = NULL);

    void all_CS_toHigh(void);
    void addSensor(Mpu9250* _mpu);
    void setup();
    void align_Tpose();
    void loop();

public slots:
    void onTimeout();
    void onRecieveUDP();

private:
    QVector<Mpu9250*> m_Sensores;
    QTimer* mTimer = NULL;
    bool wait4Start = true;
};

#endif // MOCAPSUIT_H
