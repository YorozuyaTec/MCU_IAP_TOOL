#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QSerialPort>        //提供访问串口的功能
#include <QSerialPortInfo>    //提供系统中存在的串口的信息
#include <QTimer>
#include <QTime>
#include <QDebug>
#include <QMessageBox>
#include <QFileDialog>
#include <QThread>
#include <QProgressDialog>
#include "windows.h"//windows
#include "ECanVci.h"
#include "uart.h"

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    bool open_can_flag;
    uint8_t get_data_stage;
    QProgressBar *progressBar[16];

    void lcd_init();
    void ui_init();
    void uart_init();
    void timmer_init(void);
    QString getTime(void);
    void Delay_MSec_Suspend(int msec);
    void printLog(QString log);
    void openPort(void);
    uint16_t getCrcCheck(uint8_t *puchMsg, uint16_t usDataLen); //获取CRC
    void uartReceiveHandle(void);
    void setUnit(QString unit);
    bool dataProcess(); //串口数据处理
    void showSFR(uint8_t *data);
    void showTime(uint8_t *data);
    uint8_t convertBCD(int value);
    uint16_t CRC16_MODBUS(uint8_t *puchMsg, uint16_t usDataLen);
    uint8_t checkCRC(uint8_t *databuff,uint16_t len);
    void WriteDateToDSRAM(uint8_t *data,uint16_t lenth);
    void Button_en(bool en);
    void uart_send_cmd(uartCmdTypedef *cmd);
    void uart_boot_handler(void);
    void uart_can_upgrade_handler(void);


    bool InitCanDevice(void);
    bool openCanDevice(void);
    bool closeCanDevice(void);
    bool canSendData(uint32_t frameID,uint8_t *data);
    bool canSendRemote(uint32_t frameID);
    bool  can_boot_send_data(uint8_t pgn , uint8_t *data);
    void can_boot_handler(void);
    void unified_upgrade(void);
    // bool canSendData(void);
    bool canReceiveData(void);
    uint32_t getCanStatus(void);
    uint16_t readCanReceiveBuffer(void);
    void creatReport(void);
    uint8_t checkSum(uint8_t *data,uint16_t lenth);
    void cellBoardDataProcess(CAN_OBJ *vco,bool islastframe);
    void powerBoardDataProcess(CAN_OBJ *vco);
    void A2DataProcess(CAN_OBJ *vco,bool islastframe);
    void A4DataProcess(CAN_OBJ *vco);
    void chargeBoardDataProcess(CAN_OBJ *vco,bool islastframe);



    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

    uint8_t rs485_addr;
    bool uartFirstRecflag;
    bool overReceiveFlag;
    bool isAckFlag;
    QString filePath;
    uint8_t *read_data;
    uint8_t *write_data;
    uint8_t *can_write_data;
    QByteArray temp_file;
    QByteArray can_temp_file;
    uint8_t file_data[0x7FF0]; //文件缓存
    uint16_t start_addr;
    uint32_t file_lenth;
    uint32_t can_file_lenth;
    enum fsm
    {
        FSM_INIT=0,
        FSM_IDLE,
        FSM_READ_CMD,
        FSM_READ_DATA,
        FSM_WRITE,
        FSM_ERITE_WAIT
    };
    enum FalgStatus
    {
        INIT =0,
        CHECK,
        TEST,
        READSFR,
        READTIME,
        SETCLK,
        SETTIME,
        READDATA,
        READNEXT,
        READCARIB,
        WRITEDATA,
        WRITENEXT,
        WRITEEND,
        WRITECHECK,
        RUN_APP,
        READDATA1,
        READDATA2,
        READDATA3,
        READRESULT,
        AUTOSEND,
        SETZERO
    };
    int fsm_status;  //状态机
    int can_fsm_status;
    uint8_t RecStatus;

private slots:
    void timerIRQ();
    void timer2IRQ(void);
    void on_openUartButton_clicked();

    void on_fileSelectButton_clicked();

    void on_writeFileButton_clicked();

    void on_freshenButton_clicked();

    void on_run_app_clicked();

    void on_openPortButton_clicked();

    void on_can_fileSelectButton_clicked();

    void on_can_run_app_clicked();

    void on_can_writeFileButton_clicked();

    void on_pushButton_3_clicked();

    void on_readDevButton_clicked();

    void on_rebootDevButton_clicked();

    void on_all_update_clicked();

    void on_UART_read_clicked();

    void on_checkBox_stateChanged(int arg1);

    void on_checkBox_clicked(bool checked);

    void on_UART_reboot_clicked();

    void on_writeAll_clicked();

private:
    Ui::MainWindow *ui;
    QSerialPort serial;
    QTimer *pTimer;
    QTimer *pTimer2;
    bool isConnectedFlag;
};
#endif // MAINWINDOW_H
