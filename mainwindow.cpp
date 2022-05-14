#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "canbus.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    ui_init();
    uart_init();
}

MainWindow::~MainWindow()
{
    delete ui;
}


void MainWindow::ui_init(void)
{
    //设置定时器
     timmer_init();

    //设置窗口名
    setWindowTitle("Yorozuya IAP tool V1.0");
    //刷新串口;
    setFixedSize(930,600);
    ui->freshenButton->setFixedSize(30,23);
    qDebug() << "debug " ;
    QIcon btn_icon;
    btn_icon.addFile("icon/freshen.PNG");
    ui->freshenButton->setIcon(btn_icon);
    open_can_flag=0;
    //ui->saveButton->setEnabled(false);
    connect(ui->uart_names,SIGNAL(clicked()),this,SLOT(uart_init()));
    isConnectedFlag=false;
   // uartFirstRecflag = true;
    //overReceiveFlag = false;
    //fsm_status=FSM_INIT;

    connect(&serial,&QSerialPort::readyRead,this,&MainWindow::dataProcess); //连接槽函数
    //初始化参数

    ui->BaudRatecomboBox->setCurrentText("57600");
    //Button_en(false);
    progressBar[0] = ui->UP1;
    progressBar[1] = ui->UP2;
    progressBar[2] = ui->UP3;
    progressBar[3] = ui->UP4;
    progressBar[4] = ui->UP5;
    progressBar[5] = ui->UP6;
    progressBar[6] = ui->UP7;
    progressBar[7] = ui->UP8;
    progressBar[8] = ui->UP9;
    progressBar[9] = ui->UP10;
    progressBar[10] = ui->UP11;
    progressBar[11] = ui->UP12;
    progressBar[12] = ui->UP13;
    progressBar[13] = ui->UP14;
    progressBar[14] = ui->UP15;
    progressBar[15] = ui->UP16;

}

void MainWindow::on_openUartButton_clicked()
{
    openPort();

}

void MainWindow::on_fileSelectButton_clicked()
{
    filePath = QFileDialog::getOpenFileName(
            this,
            tr("open a file."),
            "D:/",
            tr("fw_file(*.bin *.hex)"));
        if (filePath.isEmpty())
        {

        }
        else  //获取数据指针
        {
            QFileInfo file;
            QFile fp(filePath);
            if(fp.open(QFile::ReadOnly))
            {
                temp_file = fp.readAll();
                fp.close();
                uart.file_pointer = (uint8_t *)temp_file.data();
                uart.file_lenth = temp_file.length();
                printLog("open file:"+filePath);
                printLog("数据长度 = "+QString::number(uart.file_lenth) + " Byte");
            }
            file = QFileInfo(filePath);
            ui->fileNameTextLable->setText(file.fileName());
            uart.pack_size=256; //包大小
            if(uart.file_lenth%256)
            {
                uart.pack_lenth = uart.file_lenth/256+1;
            }
            else
            {
                uart.pack_lenth = uart.file_lenth/256;
            }
            printLog("分为 ["+QString::number(uart.pack_lenth) + "] 包数据发送");


        }
}

void MainWindow::on_writeFileButton_clicked()
{
    uartCmdTypedef cmd={0};
    cmd.pgn = EBD;
    cmd.ps = 0xF9;
    cmd.send_lenth=0;
    if(uart.pack_lenth>0)
    {
        uart.fsm_status = UART_FSM_INIT;
        uart_send_cmd(&cmd);

        uart.pack_cnt =0;
        uart.wait_cnt_en = true;
        printLog("开始写入数据");
    }
    else
    {
       QMessageBox::warning(NULL,"警告","未选中文件！");

    }
}

void MainWindow::on_freshenButton_clicked()
{
      uart_init();
}

void MainWindow::on_run_app_clicked()
{
    RecStatus = RUN_APP;
    uint8_t cmd[8] = {0x41,0x01,0x00,0x00,0x00,0x00,0x32,0xCA};
    serial.write((const char*)cmd,8);
    serial.waitForBytesWritten(1000);
}

void MainWindow::on_openPortButton_clicked()
{
    if(open_can_flag == false)
        {
           openCanDevice();
           if(open_can_flag)
           {
              // pTimer2->start(500);
           }

        }
        else
        {
            closeCanDevice();
            if(open_can_flag == false)
            {
               // pTimer2->stop();
            }

        }

}

void MainWindow::on_can_fileSelectButton_clicked()
{
    filePath = QFileDialog::getOpenFileName(
            this,
            tr("open a file."),
            "D:/",
            tr("fw_file(*.bin *.hex)"));
        if (filePath.isEmpty())
        {

        }
        else  //获取数据指针
        {
            QFileInfo file;
            QFile fp(filePath);
            if(fp.open(QFile::ReadOnly))
            {
                can_temp_file = fp.readAll();
                fp.close();
                can_write_data = (uint8_t *)can_temp_file.data();
                can_file_lenth = can_temp_file.length();
                printLog("open file:"+filePath);
                printLog("数据长度 = "+QString::number(can_file_lenth) + " Byte");
            }
            file = QFileInfo(filePath);
            ui->can_fileNameTextLable->setText(file.fileName());
            canBoot.file_lenth = can_file_lenth;
            canBoot.pack_size=256; //包大小
            if(can_file_lenth%256)
            {
                canBoot.pack_lenth = can_file_lenth/256+1;
            }
            else
            {
                canBoot.pack_lenth = can_file_lenth/256;
            }
            printLog("分为 ["+QString::number(canBoot.pack_lenth) + "] 包数据发送");

        }

}


void MainWindow::on_can_run_app_clicked()
{
    uint8_t temp[8] = {0};
    can_boot_send_data(RUN,temp);
    canBoot.fsm_status = CAN_FSM_RUN;
    printLog("开始运行...");
}


void MainWindow::on_can_writeFileButton_clicked()
{
    uint8_t temp[8] = {0};
    canBoot.pack_cnt=0;
    if(canBoot.pack_lenth>0)
    {
      can_boot_send_data(EBD,temp);
      canBoot.fsm_status = CAN_FSM_INIT;
      canBoot.wait_cnt_en = true;
    }
    else
    {
        QMessageBox::warning(this,"Error","文件载入失败");
    }


}


void MainWindow::on_pushButton_3_clicked()
{
    ui->logBrowser->clear();
    ui->HvLable->clear();
    ui->AvLable->clear();
    ui->BvLable->clear();
    ui->UART_HvLable->clear();
    ui->UART_AvLable->clear();
    ui->UART_BvLable->clear();
}


void MainWindow::on_readDevButton_clicked()
{
    uint8_t temp[8] = {12};

    can_boot_send_data(HBT,temp);
    canBoot.fsm_status = CAN_FSM_READ_INF;
    printLog("读取设备型号...");


}


void MainWindow::on_rebootDevButton_clicked()
{
    uint8_t temp[8] = {12};
    temp[0] = 0xFF;
    canSendData(0x18017F00+ui->ps_spinBox->value(),temp);
    //canBoot.fsm_status = CAN_FSM_READ_INF;
    printLog("复位设备...");

}


void MainWindow::on_all_update_clicked()
{
    unified.en = true;
    unified.target_cnt=0;
    unified.fsm_status = UNI_FSM_INIT;

}


void MainWindow::on_UART_read_clicked()
{
    uartCmdTypedef cmd={0};
    cmd.pgn = HBT;
    cmd.ps = 0xF9;
    cmd.send_lenth=0;
    uart_send_cmd(&cmd);
    uart.fsm_status = UART_FSM_IDLE;
    printLog("读取设备型号...");

}


void MainWindow::on_checkBox_stateChanged(int arg1)
{

}


void MainWindow::on_checkBox_clicked(bool checked)
{
    uart.mode =checked;
    if(checked)
    {

        printLog("Bootloader 模式");
    }
    else
    {
        printLog("自由运行模式");
    }

}


void MainWindow::on_UART_reboot_clicked()
{

    uint8_t cmd[8] = {0x57,0xDD,0xBB,0x00,0x00,0x00,0xC5,0x0B};
    serial.write((const char*)cmd,8);
    uart.fsm_status = UART_FSM_IDLE;
    serial.waitForBytesWritten(1000);

}


void MainWindow::on_writeAll_clicked()
{
    if(uart.pack_lenth>0)
    {
        if(uart_can.fsm_status != UART_CAN_FSM_IDLE)
        {
            QMessageBox::warning(this,"Error","正在升级...");
            return;

        }
       if(ui->END_ID->value() - ui->START_ID->value()>=0)
       {
           uart_can.fsm_status = UART_CAN_FSM_INIT;
           uart_can.file_pointer = uart.file_pointer; // 文件指针
           uart_can.file_lenth = uart.file_lenth;
           uart_can.pack_lenth = uart.pack_lenth;
           uart_can.target_id = ui->START_ID->value();
           uart_can.target_num = ui->END_ID->value() - ui->START_ID->value();

           printLog("复位目标设备...");
           for(int i=0;i<16;i++)
           {
               progressBar[i]->setValue(0);
           }

       }

       else
       {

          QMessageBox::warning(this,"Error","目标编号范围错误。");

       }

    }
    else
    {
        QMessageBox::warning(this,"Error","文件载入失败");
    }

}

