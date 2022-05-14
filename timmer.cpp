#include "mainwindow.h"


void MainWindow::timmer_init()
{
    pTimer = new QTimer;
    pTimer2 = new QTimer;
    connect(pTimer,SIGNAL(timeout()),this,SLOT(timerIRQ())); //连接槽函数
    connect(pTimer2,SIGNAL(timeout()),this,SLOT(timer2IRQ()));
    pTimer->start(100);
    pTimer2->start(50);
    printLog("Start Timmer.");

}

void MainWindow::timerIRQ()
{

    readCanReceiveBuffer();
    can_boot_handler();
    unified_upgrade();
    uart_boot_handler();

    //updateStatusBar();

}

void MainWindow::timer2IRQ()
{


    uart_can_upgrade_handler();
    //updateStatusBar();

}
