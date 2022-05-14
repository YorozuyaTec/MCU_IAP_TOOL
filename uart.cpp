#include "uart.h"
#include <time.h>
#include <string.h>
#include "canbus.h"

uartTypedef uart;
uartCanTypedef uart_can;

void MainWindow::uart_init(void)

{
    const auto infos = QSerialPortInfo::availablePorts();
    ui->uart_names->clear();
    for(const QSerialPortInfo &info : infos)
    {


        serial.setPort(info);
        if(serial.open(QIODevice::ReadWrite))
        {

            ui->uart_names->addItem(info.portName()+' '+info.description());
            //QDebug()<<info.portName();
            qDebug() << "Name : " << info.portName();
            qDebug() << "Description : " << info.description();
            qDebug() << "Manufacturer: " << info.manufacturer();
            qDebug() << "Serial Number: " << info.serialNumber();
            qDebug() << "System Location: " << info.systemLocation();
            serial.close();
        }
    }


}

void MainWindow::openPort()
{

    QString text;
    int baudrate;
    baudrate = ui->BaudRatecomboBox->currentText().toInt();
 //   qDebug("open Port");
    text = ui->BaudRatecomboBox->currentText();
    if(isConnectedFlag == false)
    {
        if(ui->uart_names->currentText()!="") //有串口
        {

            QStringList list = ui->uart_names->currentText().split(" ");
            serial.setPortName(list[0]);
            if(serial.open(QIODevice::ReadWrite))
            {
                if(baudrate == 9600)
                {
                  serial.setBaudRate(QSerialPort::Baud9600);//波特率
                  printLog("baud = 9600");
                }
                else if(baudrate == 57600)
                {
                  serial.setBaudRate(QSerialPort::Baud57600);//波特率
                  printLog("baud = 57600");
                }
                else if(baudrate == 115200)
                {
                  serial.setBaudRate(QSerialPort::Baud115200);//波特率
                  printLog("baud = 115200");
                }

                serial.setDataBits(QSerialPort::Data8);//8位数据位
                serial.setParity(QSerialPort:: NoParity );//无校验
                serial.setStopBits(QSerialPort::OneStop );//1个停止位
                serial.setFlowControl(QSerialPort::NoFlowControl);
                serial.setReadBufferSize(4096);

                ui->uart_names->setEnabled(false);
                ui->freshenButton->setEnabled(false);
                ui->BaudRatecomboBox->setEnabled(false);


               // ui->setZeroButton->setEnabled(true);
                ui->openUartButton->setText("关闭连接");
                printLog(ui->uart_names->currentText() + "打开成功");


               // printLog("RS485 Addr = " + QString::number(rs485_addr) );

                //pTimer->start(400);
                isConnectedFlag = true;
                RecStatus=CHECK;
                on_UART_read_clicked();
                //uint8_t init_sendbuff[8] = {0x49,0x00,0x00,0x00,0x00,0x00,0x0E,0x42};
                //serial.write((const char*)init_sendbuff,8);
            }

            else //打开失败
            {
                ui->uart_names->setEnabled(true);//打开失败
                ui->freshenButton->setEnabled(true);
                ui->BaudRatecomboBox->setEnabled(true);

                ui->openUartButton->setText("连接设备");
                printLog(ui->uart_names->currentText() + "打开失败");
                QMessageBox::warning(this,"Error","串口打开失败！"); //串口打开失败
               // pTimer->stop();
                uartFirstRecflag = true;
                isConnectedFlag = false;
            }


        }
        else
        {
            printLog(ui->uart_names->currentText() + "无可用串口");
            QMessageBox::warning(this,"Error","无可用串口！"); //串口打开失败
            //pTimer->stop();
            uartFirstRecflag = true;
            isConnectedFlag = false;


        }

    }
    else  //关闭串口
    {
        serial.clear();
        serial.close();
        ui->uart_names->setEnabled(true);
        ui->freshenButton->setEnabled(true);
        ui->BaudRatecomboBox->setEnabled(true);
        ui->openUartButton->setText("连接设备");
        printLog( "关闭串口" + ui->uart_names->currentText());
       // pTimer->stop();
        uartFirstRecflag = true;
        isConnectedFlag = false;
        fsm_status = FSM_INIT;
        Button_en(false);

    }

}

void MainWindow::uartReceiveHandle()
{
    QByteArray rec_buffer = serial.readAll();
    QByteArray log = rec_buffer;
    int len = rec_buffer.length();
    printLog("接收" + QString::number(rec_buffer.length()) + "Byte  =  "+ log.toHex());
    char *data;
    data = rec_buffer.data();

    switch (RecStatus)
    {
    case CHECK:
        if(data[len-2] == 'O' && data[len-1] == 'K')
        {
            ui->logBrowser->setStyleSheet("color:green;");
            printLog(rec_buffer);
            printLog("设备连接成功！");
           // isCheckFlag = false;
            serial.clear();
           // RecStatus = INIT;
           // on_ReadCfgButton_clicked();
        }
        break;
     }



}




QString MainWindow::getTime() //获取当前时间
{
   QString nowtime;
   QString shour,sminute,ssecond;
   QTime current_time =QTime::currentTime();
   int hour = current_time.hour();//当前的小时
   int minute = current_time.minute();//当前的分
   int second = current_time.second();//当前的秒
   shour=QString::number(hour);sminute=QString::number(minute);ssecond=QString::number(second);

   shour=shour.asprintf("%02d",hour);
   sminute=sminute.asprintf("%02d",minute);
   ssecond=ssecond.asprintf("%02d",second);
   nowtime = shour+":"+sminute+":"+ssecond;
   qDebug()<<nowtime;
   return nowtime;
}

void MainWindow::setUnit(QString unit)
{
    uint16_t crc_check;
    uint8_t send_cmd[8] = {0x00,0x03,0x03,0x02,0x00,0x00,0x00,0x00};
    send_cmd[0] = rs485_addr;
    if(unit == "Gs")
    {
        send_cmd[5] = 0x00;
    }
    else if(unit == "mT")
    {
       send_cmd[5] = 0x01;
    }
    else if(unit == "Oe")
    {
       send_cmd[5] = 0x02;
    }
    else if(unit == "Am")
    {
       send_cmd[5] = 0x03;
    }
    crc_check = getCrcCheck(send_cmd,6);
    send_cmd[6] = (uint8_t) crc_check;
    send_cmd[7] = (uint8_t) (crc_check >> 8);

    serial.write((const char *)send_cmd,8);
    printLog("Ste unit = "+unit);

}

void MainWindow::WriteDateToDSRAM(uint8_t *data, uint16_t lenth)
{
    uint16_t crc16;
    uint8_t temp[1024];
    memcpy(temp,data,lenth);
    crc16 = CRC16_MODBUS(data,lenth);

    temp[lenth] = (uint8_t) crc16;
    temp[lenth+1] = (uint8_t) (crc16 >> 8);
    serial.write((const char*)temp,lenth+2);
    serial.waitForBytesWritten(5000);
    Delay_MSec_Suspend(50);


}

void MainWindow::printLog(QString log) //打印LOG
{
   QString LOG;
   LOG=getTime()+"  >>"+log;
   ui->logBrowser->append(LOG);
}

uint16_t MainWindow::getCrcCheck(uint8_t *puchMsg, uint16_t usDataLen) //获取CRC16 Modbus 检验位
{
    return CRC16_MODBUS(puchMsg, usDataLen);
}

bool MainWindow::dataProcess()
{
    QByteArray data = serial.readAll();
    QString str;
    //uint8_t temp_data[256];
    char *P = data.data();
    int len = data.length();
    uint16_t pack_num;
    uint8_t pgn;  //参数组编号
    uint8_t sa;   //数据源地址
    uint16_t data_lenth;
    uartCmdTypedef uart_cmd = {0};

    uint8_t data_area[512];  //数据域

  // printLog("接收" + QString::number(len) + "Byte  =  "+ data.toHex());
    uart_cmd.ps = 0xF9; // 源地址
    //printLog("receive:"+data);
    if(checkCRC((uint8_t *)P,len) == 0)  //检查CRC
    {
        serial.clear();
        printLog("CRC校验失败");
        printLog("接收" + QString::number(len) + "Byte  =  "+ data.toHex());
        return 0;
    }

    sa= P[0];
    pgn = P[1];
    memcpy(&data_lenth,&P[2],2);  //获取数据长度
    memcpy(&pack_num, &P[4],2);   //获取数据包编号
    if(data_lenth>500)
    {
        serial.clear();
        return 0;
    }

    memcpy(data_area,&P[6],data_lenth);   //获取数据

    switch(uart.fsm_status)
    {
    case UART_FSM_IDLE:
        if(pgn == RHB)
        {
            memcpy(canBoot.boot_version,&data_area[4],2);
            memcpy(canBoot.hw_model,data_area,4);
            memcpy(canBoot.app_version,&data_area[6],2);
            ui->UART_BvLable->setText(QString(canBoot.boot_version));
            ui->UART_AvLable->setText(QString(canBoot.app_version));
            ui->UART_HvLable->setText(QString(canBoot.hw_model));
            ui->logBrowser->setStyleSheet("color:green;");
            printLog("心跳  设备型号读取成功...");

        }


        break;
    case UART_FSM_INIT:
        if(pgn == RHB)
        {
            memcpy(canBoot.boot_version,&data_area[4],2);
            memcpy(canBoot.hw_model,data_area,4);
            memcpy(canBoot.app_version,&data_area[6],2);
            ui->UART_BvLable->setText(QString(canBoot.boot_version));
            ui->UART_AvLable->setText(QString(canBoot.app_version));
            ui->UART_HvLable->setText(QString(canBoot.hw_model));
            ui->logBrowser->setStyleSheet("color:green;");
            printLog("设备型号读取成功...");

        }

        else if(pgn == BOT)  //可以通知设备进行升级
        {
            memcpy(canBoot.boot_version,&data_area[4],2);

            uart_cmd.data[0] = 0x01;uart_cmd.data[1] = 0x02;
            memcpy(&uart_cmd.data[2],&uart.pack_size,2);
            memcpy(&uart_cmd.data[4],&uart.file_lenth,4);

            uart_cmd.pgn = VTU;
            uart_cmd.send_lenth=8;
            uart_send_cmd(&uart_cmd);

            uart.wait_cnt_en = false;
            uart.complete_flag = false;
            uart.success_flag = false;
            uart.fail_flag = false;

            printLog("接受到BOT...");

        }
        else if(pgn == VCU)
        {
            uart.pack_cnt=0;                       //包计数器清零
            uart.send_data_flag = true;            //开始发送数据
            uart.wait_cnt_en = false; //停止等待计数
            uart.wait_cnt=0;
            printLog("传输第一包");
        }



        break;

    case UART_FSM_READ_VER:

        uart.fsm_status = UART_FSM_INIT;

        break;
    case UART_FSM_WRITE_DATA:




        break;

    case UART_FSM_RCK:
        if(pgn == RCK)  //收到确认数据回复
        {
            if(uart.pack_cnt<uart.pack_lenth-1)
            {
                uart.send_data_flag = true; //开启下一次传输
                uart.pack_cnt++;
                printLog("收到RCK");
            }
            else if(uart.pack_cnt == uart.pack_lenth-1)
            {
                printLog("所有数据传输完成");
                uart.success_flag = true;  //升级成功
                uart.complete_flag = true;
                uart.fsm_status = UART_FSM_IDLE;
            }
            uart.wait_rck_cnt_en=false;  //停止等待
            uart.wait_rck_cnt=0;
            uart.error_cnt=0;
        }
        break;
     case UART_FSM_CBOT:
        if(pgn == CBOT)
        {
//            uart.wait_cbot_en = false;  //停止计数
//            uart.wait_cbot_cnt=0;
            uart_can.fsm_status = UART_CAN_FSM_CVTU;
            printLog("目标板在bootloader区...");

        }
        break;
     case UART_FSM_CVCU:
        if(pgn == CVCU)
        {
            uart_can.fsm_status = UART_CAN_FSM_WRITE_DATA;
            uart_can.pack_cnt=0;
            printLog("收到CVCU...");
        }

        break;
    case UART_FSM_CACK:
        if(pgn == CACK)  //收到确认数据回复
        {
            if(uart_can.pack_cnt<uart_can.pack_lenth-1)
            {

                printLog("收到CACK");

            }
            else if(uart_can.pack_cnt == uart_can.pack_lenth-1)
            {
                printLog("所有数据传输完成");

            }
            uart_can.fsm_status = UART_CAN_FSM_WAIT_CRCK;
            uart_can.wait_cbot_cnt=0;

        }

        if(pgn == CRCK)  //得到目标板写数据成功回复
        {
            if(uart_can.pack_cnt<uart_can.pack_lenth-1)
            {
                uart_can.pack_cnt++;
                uart_can.fsm_status = UART_CAN_FSM_WRITE_DATA; //继续发送数据
                printLog("收到CRCK");

            }
            else if(uart_can.pack_cnt == uart_can.pack_lenth-1) //完成升级
            {


                if(uart_can.target_id<uart_can.target_num) //下一个
                {
                    uart_can.target_id++;
                    uart_can.fsm_status = UART_CAN_FSM_INIT;
                    printLog("下一目标");
                }
                else
                {
                     uart_can.fsm_status = UART_CAN_FSM_IDLE; //继续发送数据
                     uart.fsm_status = UART_FSM_IDLE;
                     printLog("完成升级");
                }
                uart_can.pack_cnt=0;
            }

        }

        break;


    }
    serial.clear();
    return true;


}

uint8_t MainWindow::convertBCD(int value)
{
    uint8_t result;
    result  = (uint8_t)value;
    result = ((result/10)<<4) + result%10;
    return result;


}

void MainWindow::uart_send_cmd(uartCmdTypedef *cmd)
{
    uint16_t crc;
    uint8_t data[512];

    data[0] = cmd->ps;
    data[1] = cmd->pgn ;
    memcpy(&data[2], &cmd->send_lenth, 2); //数据长度
    memcpy(&data[4], &cmd->data_pack_num,2); //数据包编号
    if(cmd->send_lenth>0)
    {
        memcpy(&data[6],cmd->data,cmd->send_lenth);
    }


    crc = CRC16_MODBUS(data, cmd->send_lenth + 6);
    memcpy(&data[cmd->send_lenth + 6], &crc, 2);


    serial.write((const char*)data,cmd->send_lenth+8);
//    if(!serial.waitForBytesWritten())
//    {
//       //serial.write((const char*)data,cmd->send_lenth+8);
//        printLog("发送失败");
//    }
    serial.waitForBytesWritten(200);
   // uart_module_send_hex( data, cmd->send_lenth+6 );

}


void MainWindow::uart_boot_handler()
{
    uartCmdTypedef uart_cmd={0};
    uart_cmd.ps = 0xF9; // 源地址
    static uint8_t delay_cnt=0;
    if(uart.send_data_flag)
    {
        uart.send_data_flag = false;
        if(uart.pack_cnt<uart.pack_lenth-1)
        {
            printLog("发送第"+QString::number(uart.pack_cnt)+"包数据");
            uart_cmd.pgn = DAT;
            uart_cmd.send_lenth = 256;
            uart_cmd.data_pack_num = uart.pack_cnt;
            memcpy(uart_cmd.data,&uart.file_pointer[uart.pack_cnt*256],256);
            uart_send_cmd(&uart_cmd);
            uart.fsm_status = UART_FSM_RCK;  //等待ACK

            uart.wait_rck_cnt_en=true;
            uart.wait_rck_cnt=0;
            ui->progressBar->setValue(uart.pack_cnt*(100.00/uart.pack_lenth));
            //printLog("发送CCK");
        }
        else if(uart.pack_cnt == uart.pack_lenth-1)
        {
            printLog("发送第"+QString::number(uart.pack_cnt)+"包数据,最后一包。");
            uart_cmd.pgn = DAT;
            uart_cmd.send_lenth = 256;
            uart_cmd.data_pack_num = uart.pack_cnt;
            memcpy(uart_cmd.data,&uart.file_pointer[uart.pack_cnt*256],256);
            uart_send_cmd(&uart_cmd);
            uart.fsm_status = UART_FSM_RCK;  //等待ACK

            uart.wait_rck_cnt_en=true;
            uart.wait_rck_cnt=0;
            ui->progressBar->setValue(100);
        }
    }

    if(uart.wait_cnt_en)  //等待BOT
    {
        uart.wait_cnt++;
        uart_cmd.pgn = EBD;
        uart_cmd.ps = 0xF9;
        uart_cmd.send_lenth=0;
        uart_send_cmd(&uart_cmd);//轮询EBD
        if(uart.wait_cnt>30) //5S
        {
            uart.wait_cnt_en = false;
            uart.wait_cnt = 0;

            uart.success_flag = false;  //升级失败
            uart.complete_flag = true;
            uart.fsm_status = UART_FSM_IDLE;

            QMessageBox::warning(this,"Error","响应超时");


        }
    }

    if(uart.wait_rck_cnt_en)  //等到RCK
    {
        uart.wait_rck_cnt++;
        if(uart.wait_rck_cnt>10) //1s
        {
            //canBoot.wait_rck_cnt_en = false;
            uart.wait_rck_cnt = 0;

            uart_cmd.pgn = DAT;
            uart_cmd.send_lenth = 256;
            uart_cmd.data_pack_num = uart.pack_cnt;
            memcpy(uart_cmd.data,&uart.file_pointer[uart.pack_cnt*256],256); //重发数据
            uart_send_cmd(&uart_cmd);

            printLog("重新发送第"+QString::number(uart.pack_cnt)+"包数据");
            uart.error_cnt++;
            if( uart.error_cnt>5)
            {
                 uart.error_cnt=0;
                uart.success_flag = false;  //升级失败
                uart.complete_flag = true;
                uart.fsm_status = UART_FSM_IDLE;

                QMessageBox::warning(this,"Error","CCK响应超时");
                uart.wait_rck_cnt_en = false;

            }

        }
    }



    if(uart.mode) //bootloader 模式，循环发送心跳维持状态
    {
        if(++delay_cnt>10)
        {
            delay_cnt=0;
            if(uart.fsm_status == UART_FSM_IDLE)
            {
                uart_cmd.ps = 0xF9;
                uart_cmd.pgn = HBT;
                uart_cmd.send_lenth = 0;
                uart_cmd.data_pack_num = 0;
                uart_send_cmd(&uart_cmd);

            }
        }

    }

}


void MainWindow::uart_can_upgrade_handler()
{
    uartCmdTypedef cmd;
    cmd.ps = 0xF9;
    switch(uart_can.fsm_status)
    {
      case UART_CAN_FSM_IDLE:

      break;
    case UART_CAN_FSM_INIT:

        cmd.pgn = CEBD;
        cmd.send_lenth=1;
        cmd.data[0] = uart_can.target_id;
        uart_send_cmd(&cmd);
        uart.fsm_status = UART_FSM_CBOT;
        uart_can.fsm_status = UART_CAN_FSM_WAIT_CBOT;
        uart_can.wait_cbot_en = true; //计时器开启
        uart_can.pack_cnt = 0;
        ui->P_ID->setValue(uart_can.target_id);
      break;

    case UART_CAN_FSM_WAIT_CBOT:
        uart_can.wait_cbot_cnt++;
        if(uart_can.wait_cbot_cnt>50) //3S
        {
            uart_can.wait_cbot_en = false;
            uart_can.wait_cbot_cnt = 0;

            uart_can.success_flag = false;  //升级失败
            uart_can.complete_flag = true;
            uart_can.fsm_status = UART_CAN_FSM_FAIL;
            uart.fsm_status = UART_FSM_IDLE;
            uart_can.error_code =1;
            //QMessageBox::warning(this,"Error","目标板响应超时");


        }
        break;


    case UART_CAN_FSM_CVTU:
        cmd.pgn = CVTU;
        cmd.ps = 0xF9;
        cmd.send_lenth=9;
        cmd.data[0] = uart_can.target_id;
        cmd.data[1] = 0x01;cmd.data[2] = 0x02;
        memcpy(&cmd.data[3],&uart.pack_size,2);
        memcpy(&cmd.data[5],&uart.file_lenth,4);

        uart_send_cmd(&cmd);
        uart.fsm_status = UART_FSM_CVCU;
        uart_can.fsm_status = UART_CAN_FSM_WAIT_CVCU;
        printLog("发CVTU...");

    break;

    case UART_CAN_FSM_WAIT_CVCU:

        break;

    break;

    case UART_CAN_FSM_WRITE_DATA:
        memset(cmd.data,0,sizeof(cmd.data));
        if(uart_can.pack_cnt<uart_can.pack_lenth-1)
        {
            printLog("发送第"+QString::number(uart_can.pack_cnt)+"包数据");
            memcpy(&cmd.data[1],&uart_can.file_pointer[uart_can.pack_cnt*256],256);
            progressBar[uart_can.target_id]->setValue(uart_can.pack_cnt*(100.00/uart_can.pack_lenth));
        }

        else if(uart_can.pack_cnt == uart_can.pack_lenth-1)
        {
            memcpy(&cmd.data[1],&uart_can.file_pointer[uart_can.pack_cnt*256],uart_can.file_lenth - uart_can.pack_cnt*256);
            printLog("发送第"+QString::number(uart_can.pack_cnt)+"包数据,最后一包。");
            progressBar[uart_can.target_id]->setValue(100);
        }

        cmd.pgn = CDAT;
        cmd.send_lenth = 257;
        cmd.data_pack_num = uart_can.pack_cnt;
        cmd.data[0] = uart_can.target_id;

        uart_send_cmd(&cmd);
        uart.fsm_status = UART_FSM_CACK;
        uart_can.fsm_status = UART_CAN_FSM_WAIT_CACK;  //等待ACK
        uart_can.wait_cbot_cnt=0;
        uart_can.error_cnt = 0;
        //printLog("发送CCK");


    break;
    case UART_CAN_FSM_WAIT_CACK:
        uart_can.wait_cbot_cnt++;
        if(uart_can.wait_cbot_cnt>50) //
        {
            uart_can.wait_cbot_cnt = 0;
            uart.fsm_status = UART_CAN_FSM_WRITE_DATA;  //重发
            uart_can.error_cnt++;
            printLog("CACK响应超时,重发...");
        }
        if(uart_can.error_cnt>3) //3S
        {
            uart_can.error_cnt=0;
            uart_can.wait_cbot_en = false;
            uart_can.wait_cbot_cnt = 0;
            uart_can.success_flag = false;  //升级失败
            uart_can.complete_flag = true;
            uart.fsm_status = UART_FSM_IDLE;
            uart_can.fsm_status = UART_CAN_FSM_FAIL;

            // QMessageBox::warning(this,"Error","目标板响应超时");
            uart_can.error_code =2;

        }

        break;
    case UART_CAN_FSM_WAIT_CRCK:
        uart_can.wait_cbot_cnt++;
        if(uart_can.wait_cbot_cnt>50) //
        {
            uart_can.wait_cbot_cnt = 0;
            uart.fsm_status = UART_CAN_FSM_WRITE_DATA;  //重发
            uart_can.error_cnt++;
            printLog("CRCK响应超时,重发...");
        }
        if(uart_can.error_cnt>3) //
        {
            uart_can.error_cnt = 0;
            uart_can.wait_cbot_en = false;
            uart_can.wait_cbot_cnt = 0;
            uart_can.success_flag = false;  //升级失败
            uart_can.complete_flag = true;
            uart.fsm_status = UART_FSM_IDLE;
            uart_can.fsm_status = UART_CAN_FSM_FAIL;

            //QMessageBox::warning(this,"Error","目标板响应超时");
            uart_can.error_code =3;

        }

        break;
    case UART_CAN_FSM_FAIL:

        printLog("目标板ID["+QString::number(uart_can.target_id)+"]升级失败");
        if(uart_can.target_id<uart_can.target_num) //下一个
        {
            uart_can.target_id++;
            uart_can.fsm_status = UART_CAN_FSM_INIT;

        }
        else
        {
            uart_can.fsm_status = UART_CAN_FSM_IDLE;
            uart.fsm_status = UART_FSM_IDLE;
            if(uart_can.error_code ==1)
            {
                printLog("CBOT响应超时");
            }
            else if(uart_can.error_code ==2)
            {
                printLog("CACK响应超时");
            }
            else if(uart_can.error_code ==3)
            {
                printLog("CRCK响应超时");
            }
        }


        break;

    }



//    if(uart_can.wait_cbot_en)
//    {
//        uart_can.wait_cbot_cnt++;
//        if(uart_can.wait_cbot_cnt>30) //3S
//        {
//            uart_can.wait_cbot_en = false;
//            uart_can.wait_cbot_cnt = 0;

//            uart_can.success_flag = false;  //升级失败
//            uart_can.complete_flag = true;
//            uart_can.fsm_status = UART_FSM_IDLE;

//            QMessageBox::warning(this,"Error","目标板响应超时");


//        }

//    }


}






void MainWindow::Button_en(bool en)
{

}

void MainWindow::Delay_MSec_Suspend(int msec)
{
    QTime _Timer = QTime::currentTime().addMSecs(msec);
    while( QTime::currentTime() < _Timer );
}
void InvertUint8(uint8_t *dBuf,uint8_t *srcBuf)
{
    uint8_t i;
    uint8_t tmp[4];
    tmp[0] = 0;
    for(i=0;i< 8;i++)
    {
      if(srcBuf[0]& (1 << i))
        tmp[0]|=1<<(7-i);
    }
    dBuf[0] = tmp[0];

}
void InvertUint16(uint16_t *dBuf,uint16_t *srcBuf)
{
    uint8_t i;
    uint16_t tmp[4];
    tmp[0] = 0;
    for(i=0;i< 16;i++)
    {
      if(srcBuf[0]& (1 << i))
        tmp[0]|=1<<(15 - i);
    }
    dBuf[0] = tmp[0];
}

uint8_t MainWindow::checkCRC( uint8_t *databuff,uint16_t len)  //Verify  CRC   0:fail 1:success
{
  uint8_t *P;
  P = databuff;
  uint16_t crc16;

//    if (*P != addr)
//	{
//	  return 0;
//	}

  crc16 = CRC16_MODBUS (P, len - 2);

  if (((((uint16_t) *(P + len - 1)) << 8) + *(P + len - 2)) == crc16)
    {
      return 1;
    }
  else

  return 0;

}
uint16_t MainWindow::CRC16_MODBUS(uint8_t *puchMsg, uint16_t usDataLen)
{
  uint16_t wCRCin = 0xFFFF;
  uint16_t wCPoly = 0x8005;
  uint8_t wChar = 0;

  while (usDataLen--)
  {
        wChar = *(puchMsg++);
        InvertUint8(&wChar,&wChar);
        wCRCin ^= (wChar << 8);
        for(uint8_t i = 0;i < 8;i++)
        {
          if(wCRCin & 0x8000)
            wCRCin = (wCRCin << 1) ^ wCPoly;
          else
            wCRCin = wCRCin << 1;
        }
  }
  InvertUint16(&wCRCin,&wCRCin);
  return (wCRCin) ;
}

