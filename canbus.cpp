#include "mainwindow.h"
#include "ui_mainwindow.h"
//#include "database.h"
#include "canbus.h"

int nDeviceType = 3; /* USBCAN-I */
int nDeviceInd = 0;
int nReserved =0;
int nCANInd = 0;
DWORD dwRel;

canBootTypeDef canBoot;
uDTypeDef unified;


extern "C"    //由于是C版的dll文件，在C++中引入其头文件要加extern "C" {},注意
{
     #include "ECanVci.h"
}



bool  MainWindow::InitCanDevice()
{
    INIT_CONFIG init_config;
    init_config.AccCode = 0;
    init_config.AccMask =0xffffffff;
    init_config.Filter = 0;

    init_config.Timing0 = 0x01;  //  buadrate=250Kbit/s
    init_config.Timing1 = 0x1c;

    init_config.Mode = 0;


    dwRel = InitCAN(nDeviceType, nDeviceInd, nCANInd,&init_config);
    if (dwRel != STATUS_OK)
    {
       QMessageBox::warning(NULL, "设备初始化失败", "Content", QMessageBox::Yes | QMessageBox::No, QMessageBox::Yes);
       printLog("初始化设备失败！");

       return false;
    }
    dwRel = StartCAN(nDeviceType, nDeviceInd, nCANInd);
    if (dwRel == STATUS_ERR)
    {
     CloseDevice(nDeviceType, nDeviceInd);
     QMessageBox::warning(NULL, "CAN启动失败", "Content", QMessageBox::Yes | QMessageBox::No, QMessageBox::Yes);
      return false;
    }
    printLog("初始化设备成功");
    printLog("波特率 = 250K");
    return true;

}

bool MainWindow::openCanDevice()
{

    dwRel = OpenDevice(nDeviceType, nDeviceInd, nReserved);
    if (dwRel != STATUS_OK)
    {
       QMessageBox::warning(NULL, "设备打开失败", "USB-CAN连接错误", QMessageBox::Yes | QMessageBox::No, QMessageBox::Yes);
       printLog("打开设备失败！");
       open_can_flag = false;
       return false;

    }
    printLog("打开设备成功！");
    open_can_flag = InitCanDevice();

    if(open_can_flag == true)
      ui->openPortButton->setText("关闭设备");

    return open_can_flag;
}

bool MainWindow::closeCanDevice()
{
    dwRel = CloseDevice(nDeviceType, nDeviceInd);
    if (dwRel != STATUS_OK)
    {
       QMessageBox::warning(NULL, "设备关闭失败", " ", QMessageBox::Yes | QMessageBox::No, QMessageBox::Yes);
       printLog("关闭设备失败！");
       return false;
    }
    printLog("关闭设备成功");
    open_can_flag = false;
    if(open_can_flag == false)
      ui->openPortButton->setText("打开设备");
    return true;

}

bool MainWindow::canSendData(uint32_t frameID,uint8_t *data)
{
    //uint8_t data[8] = {0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08};
    CAN_OBJ vco;
    DWORD dwRel;
    ZeroMemory(&vco, sizeof (CAN_OBJ));
    //vco.ID = 0x001FFFF;
    vco.ID = frameID;
    vco.SendType = 0;
    vco.RemoteFlag = 0;
    vco.ExternFlag = 1;
    vco.DataLen = 8;
    data[7] = checkSum(data,8);
    memcpy(vco.Data,data,8);

    dwRel = Transmit(nDeviceType,nDeviceInd,nCANInd,&vco,1);
    //printLog("发送了"+QString::number(dwRel)+"帧数据");

    //getCanStatus();
    return dwRel;

}
bool MainWindow::canSendRemote(uint32_t frameID)
{
    CAN_OBJ vco;
    DWORD dwRel;
    ZeroMemory(&vco, sizeof (CAN_OBJ));
    //vco.ID = 0x001FFFF;
    vco.ID = frameID;
    vco.SendType = 0;
    vco.RemoteFlag = 1;
    vco.ExternFlag = 1;
    vco.DataLen = 8;
    //data[7] = checkSum(data,8);
    // memcpy(vco.Data,data,8);

    dwRel = Transmit(nDeviceType,nDeviceInd,nCANInd,&vco,1);
    //printLog("发送了"+QString::number(dwRel)+"帧数据");

    //getCanStatus();
    return dwRel;

}

bool MainWindow::can_boot_send_data(uint8_t pgn , uint8_t *data)
{
    uint32_t temp1,temp2;
    temp1 = pgn;
    temp2 = ui->ps_spinBox->value();
    CAN_OBJ vco;
    DWORD dwRel;
    ZeroMemory(&vco, sizeof (CAN_OBJ));
    //vco.ID = 0x001FFFF;
    vco.ID = ui->sa_spinBox->value() | temp1<<16 | temp2<<8;
    //vco.ID = 0x00B002F9;
    vco.SendType = 0;
    vco.RemoteFlag = 0;
    vco.ExternFlag = 1;
    vco.DataLen = 8;
    //data[7] = checkSum(data,8);
    memcpy(vco.Data,data,8);

    dwRel = Transmit(nDeviceType,nDeviceInd,nCANInd,&vco,1);
    if(dwRel<1)
    {
         printLog("数据帧发送失败");
    }


    //getCanStatus();
    return dwRel;

}

uint16_t MainWindow::readCanReceiveBuffer()
{
    DWORD dwRel;
    QString buffer;
    QString logout;
    CAN_OBJ vco[100];
    uint8_t pgn;  //参数组编号
    uint8_t sa;   //数据源地址
    uint8_t ps;   //数据源地址
    uint16_t crc;
    static uint8_t err_cnt=0;
    dwRel = Receive(nDeviceType, nDeviceInd, nCANInd, vco, 100, 100);
    // printLog( "读到数据帧数 = ");
    // printLog(QString::number(dwRel,10));

    if(dwRel>0)
    {
        for(int i=0;i<(int)dwRel;i++)
        {
            QByteArray temp_data((const char*)(vco[i].Data),8);
            buffer = temp_data.toHex();
            if(vco[i].RemoteFlag == 0)
            {
                logout = "接收数据帧 frameID = 0x";
                logout.append(QString::number(vco[i].ID,16));
                logout.append("  Data = ");
                logout.append(buffer);
                pgn = (uint8_t)(vco[i].ID>>16);
                sa =  (uint8_t)vco[i].ID;
                ps =  (uint8_t)(vco[i].ID>>8);
                if(sa== ui->ps_spinBox->value() || ps == ui->sa_spinBox->value()) //检查地址
                {
                    switch(canBoot.fsm_status)
                    {
                    case CAN_FSM_READ_INF:
                        if(pgn == RHB)
                        {
                            memcpy(canBoot.boot_version,&temp_data[4],2);
                            memcpy(canBoot.hw_model,temp_data,4);
                            memcpy(canBoot.app_version,&temp_data[6],2);
                            ui->BvLable->setText(QString(canBoot.boot_version));
                            ui->HvLable->setText(QString(canBoot.hw_model));
                            ui->AvLable->setText(QString(canBoot.app_version));

                            printLog("读取设备型号成功.");
                        }
                        canBoot.fsm_status = CAN_FSM_INIT;
                        break;
                    case CAN_FSM_INIT:
                        if(pgn == BOT)
                        {
                            memcpy(canBoot.boot_version,&temp_data[4],2);
                            memset(canBoot.tx_data,0,8);
                            canBoot.tx_data[0] = 0x01;canBoot.tx_data[1] = 0x02;
                            memcpy(&canBoot.tx_data[2],&canBoot.pack_size,2);
                            memcpy(&canBoot.tx_data[4],&canBoot.file_lenth,4);
                            can_boot_send_data(VTU,canBoot.tx_data);
                            canBoot.wait_cnt_en = false;

                            canBoot.complete_flag = false;
                            canBoot.success_flag = false;
                            canBoot.fail_flag = false;
                        }
                        else if(pgn == VCU)
                        {
                            canBoot.fsm_status = CAN_FSM_WRITE_DATA;  //开始传输数据
                            canBoot.pack_cnt=0;                       //包计数器清零
                            canBoot.send_data_flag = true;            //开始发送数据

                        }
                        break;
                    case CAN_FSM_WRITE_DATA:


                        break;
                    case CAN_FSM_CCK:
                        if(pgn == RCK)
                        {
                           // printLog("数据验证成功");
                            err_cnt=0;
                            if(canBoot.pack_cnt<canBoot.pack_lenth-1)
                            {
                                canBoot.pack_cnt++;
                                canBoot.fsm_status = CAN_FSM_WRITE_DATA;
                                canBoot.send_data_flag = true;        //开始发送数据

                            }
                            else if(canBoot.pack_cnt == canBoot.pack_lenth-1) //所有数据传输完成
                            {
                                printLog("所有数据传输完成");
                                canBoot.success_flag = true;  //升级成功
                                canBoot.complete_flag = true;
//                                canBoot.fsm_status = CAN_FSM_INIT;
                                can_boot_send_data(RUN,canBoot.tx_data);
                                canBoot.fsm_status = CAN_FSM_RUN;
                            }

                            canBoot.wait_rck_cnt_en=false;  //停止等待
                            canBoot.wait_rck_cnt=0;

                        }
                        else if(pgn == UER)
                        {

                            printLog("数据验证错误，重发第"+QString::number(canBoot.pack_cnt)+"包数据");
                            err_cnt++;
                            if(err_cnt>5)
                            {
                                err_cnt=0;
                                canBoot.pack_cnt=0;
                                printLog("重发超时。");

                                canBoot.success_flag = false;  //升级失败
                                canBoot.complete_flag = true;
                                canBoot.fsm_status = CAN_FSM_INIT;


                            }
                            else
                            {
                            canBoot.fsm_status = CAN_FSM_WRITE_DATA;
                            canBoot.send_data_flag = true;
                            }
                        }
                        break;
                    case CAN_FSM_RUN:
                          canBoot.fsm_status = CAN_FSM_INIT;


                        break;
                    case CAN_FSM_SUCCESS:
//                         canBoot.success_flag = true;  //升级成功
//                         canBoot.complete_flag = true;
//                         canBoot.fsm_status = CAN_FSM_INIT;
                        break;
                    case CAN_FSM_ERROR:
//                         canBoot.success_flag = false;  //升级未成功
//                         canBoot.complete_flag = true;
//                         canBoot.fsm_status = CAN_FSM_INIT;
                        break;


                    }
                }



            }
        }
    }
    return dwRel;

}

uint32_t MainWindow::getCanStatus()
{
    //DWORD dwRel;
    ERR_INFO vei;
    //memset(&vbi,0,sizeof(BOARD_INFO));
    //dwRel = ReadBoardInfo(nDeviceType,nDeviceInd,&vbi);
    //dwRel = ReadErrInfo(nDeviceType, nDeviceInd, nCANInd, &vei);
    ReadErrInfo(nDeviceType, nDeviceInd, nCANInd, &vei);
    //dwRel = dwRel;
    return vei.ErrCode;
}


uint8_t MainWindow::checkSum(uint8_t *data,uint16_t lenth)
{
    uint16_t temp=0;
    for(int i=0;i<lenth-1;i++)
    {
        temp+=data[i];
    }
    return (uint8_t)temp;

}

void MainWindow::can_boot_handler()
{
    uint8_t temp_data[8]={0};
    uint16_t crc16,lenth;
    uint8_t cash_data[256]={0};
    static uint8_t error_cnt=0;
    if(canBoot.send_data_flag)
    {
        canBoot.send_data_flag = false;
        if(canBoot.pack_cnt<canBoot.pack_lenth-1)
        {
            //printLog("发送第"+QString::number(canBoot.pack_cnt)+"包数据");
            for(int i=0;i<32;i++)
            {
                can_boot_send_data(DAT,&can_write_data[i*8+canBoot.pack_cnt*256]);
                Delay_MSec_Suspend(4);
            }
            crc16 = CRC16_MODBUS(&can_write_data[canBoot.pack_cnt*256],256);
            lenth = 256;
            memcpy(temp_data,&lenth,2);
            memcpy(&temp_data[2],&crc16,2);
            canBoot.fsm_status = CAN_FSM_CCK;
            can_boot_send_data(CCK,temp_data);
            canBoot.wait_rck_cnt_en=true;
            canBoot.wait_rck_cnt=0;
            ui->can_progressBar->setValue(canBoot.pack_cnt*(100.00/canBoot.pack_lenth));
            //printLog("发送CCK");
        }
        else if(canBoot.pack_cnt == canBoot.pack_lenth-1)
        {
           // printLog("发送第"+QString::number(canBoot.pack_cnt)+"包数据,最后一包。");
            memcpy(cash_data,&can_write_data[canBoot.pack_cnt*256],canBoot.file_lenth - canBoot.pack_cnt*256);
            for(int i=0;i<32;i++)
            {
                can_boot_send_data(DAT,&cash_data[i*8]);
            }
            crc16 = CRC16_MODBUS(cash_data,256);
            lenth = 256;
            memcpy(temp_data,&lenth,2);
            memcpy(&temp_data[2],&crc16,2);
            canBoot.fsm_status = CAN_FSM_CCK;
            can_boot_send_data(CCK,temp_data);
            ui->can_progressBar->setValue(100);
        }

    }

    if(canBoot.wait_cnt_en)
    {
        canBoot.wait_cnt++;
        can_boot_send_data(EBD,temp_data);  //轮询EBD
        if(canBoot.wait_cnt>50) //5S
        {
            canBoot.wait_cnt_en = false;
            canBoot.wait_cnt = 0;

            canBoot.success_flag = false;  //升级失败
            canBoot.complete_flag = true;
            canBoot.fsm_status = CAN_FSM_INIT;

            QMessageBox::warning(this,"Error","响应超时");


        }
    }

    if(canBoot.wait_rck_cnt_en)
    {
       canBoot.wait_rck_cnt++;
    if(canBoot.wait_rck_cnt>10) //1s
       {
           //canBoot.wait_rck_cnt_en = false;
           canBoot.wait_rck_cnt = 0;
           can_boot_send_data(CCK,temp_data);  //重发送CCK
           //QMessageBox::warning(this,"Error","CCK响应超时");
           printLog("重发CCK...");
           error_cnt++;
           if(error_cnt>5)
           {
               error_cnt=0;
               canBoot.success_flag = false;  //升级失败
               canBoot.complete_flag = true;
               canBoot.fsm_status = CAN_FSM_INIT;

               QMessageBox::warning(this,"Error","CCK响应超时");
               canBoot.wait_rck_cnt_en = false;

           }

       }
    }




}

void MainWindow::unified_upgrade()
{

    uint8_t temp[8] = {0};
    temp[0] = 0xFF;
    if(unified.en)
    {
        unified.start_id = ui->start_addr->value();
        unified.end_id = ui->end_addr->value();
        if((unified.end_id - unified.start_id) < 0 || (unified.end_id - unified.start_id) > 32)  //检查地址
        {
          QMessageBox::warning(this,"Error","地址范围错误！");
          unified.en = false;
          return;
        }

        switch(unified.fsm_status)
        {
        case UNI_FSM_INIT:


            ui->ps_spinBox->setValue(unified.start_id+unified.target_cnt); // 设置地址

            canSendData(0x18017F00+unified.start_id+unified.target_cnt,temp); //复位
            canSendData(0x18017F00+unified.start_id+unified.target_cnt,temp); //复位
            unified.fsm_status = UNI_FSM_CHECK_BOOT;
            printLog("复位设备...");

            canBoot.complete_flag = false;
            break;
         case UNI_FSM_CHECK_BOOT:
            on_can_writeFileButton_clicked();  //写入文件
            unified.fsm_status = UNI_FSM_WAIT_RESULT;
            break;

        case UNI_FSM_WAIT_RESULT:
            if(canBoot.complete_flag) //等待升级完成
            {
                //if()
               if(canBoot.success_flag)
               {
                   printLog("板卡["+QString::number(unified.target_cnt)+"]升级成功。");
               }
               else
               {
                    printLog("板卡["+QString::number(unified.target_cnt)+"]升级失败!");
               }
               if( unified.target_cnt < (unified.end_id - unified.start_id))
               {
                   unified.target_cnt++;
                   unified.fsm_status = UNI_FSM_INIT;
               }
               else
               {
                 unified.en = false;
                 unified.target_cnt = 0;
                 printLog("所有升级完成。");
               }

            }
            break;


        }




    }




}






