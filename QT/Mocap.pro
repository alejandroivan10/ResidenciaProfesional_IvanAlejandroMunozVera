#-------------------------------------------------
#
# Project created by QtCreator 2021-05-11T15:45:04
#
#-------------------------------------------------

QT       += core gui network



lessThan(QT_VERSION, 5.7) {
    message("Cannot use Qt $${QT_VERSION}")
    error("Use Qt 5.7 or newer")
}

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

QMAKE_CXXFLAGS += -std=c++17

TARGET = Mocap
TEMPLATE = app

# The following define makes your compiler emit warnings if you use
# any feature of Qt which as been marked as deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0


SOURCES += main.cpp\
        mainwindow.cpp \
    matlab.cpp \
    mpu9250.cpp \
    arduino.cpp \
    mocapsuit.cpp

HEADERS  += mainwindow.h \
    Complementos.h \
    RegistersAndMask.h \
    SPI.h \
    matlab.h \
    mpu9250.h \
    arduino.h \
    mocapsuit.h

FORMS    += mainwindow.ui

LIBS += -L/usr/local/lib -lwiringPi
