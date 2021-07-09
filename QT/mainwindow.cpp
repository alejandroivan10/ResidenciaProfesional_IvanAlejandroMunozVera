#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <QUdpSocket>

extern QUdpSocket* mUdpSocket;

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_pushButton_clicked()
{
    QString sAUX = ui->lineEdit->text();
    QByteArray prueba = QByteArray(sAUX.toUtf8());
    mUdpSocket->writeDatagram(prueba, QHostAddress("192.168.1.126"), 63604);
}
