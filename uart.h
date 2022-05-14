#ifndef UART_H
#define UART_H



#include "ui_mainwindow.h"

#include<qdebug.h>

typedef struct
{
    uint8_t pgn;
    uint8_t ps; //目标地址
    uint8_t data[512]; //数据
    uint16_t data_pack_num; //数据包编号
    uint16_t send_lenth;

} uartCmdTypedef;


typedef struct
{
    uint8_t data[512];
    uint16_t rec_counter;  //接收计数器
    uint8_t fsm_status;
    uint32_t pack_cnt;  //数据包计数器
    uint16_t pack_lenth;
    uint16_t pack_size; //包大小
    //uint32_t frame_cnt; //帧计数器
    uint16_t crc;
    uint16_t app_version;
    uint32_t file_lenth;
    uint8_t jump_flag;
    uint8_t tx_message[8];
    uint8_t jump_delay_en;   //跳转计数器使能
    uint32_t jump_delay_cnt; //跳转计数器

    uint8_t *file_pointer;  //文件指针
    uint8_t error_cnt;



    bool wait_cnt_en;
    int wait_cnt;  //等待BOT计数器

    bool wait_rck_cnt_en;  //等待RCK计数器
    int wait_rck_cnt;



    bool send_data_flag;

    bool complete_flag;
    bool success_flag;
    bool fail_flag;

    bool mode;  //0 自由运行模式  1 Bootloader模式

} uartTypedef;

typedef struct
{
    uint8_t *file_pointer;  //文件指针
    uint8_t fsm_status;
    uint32_t pack_cnt;  //数据包计数器
    uint16_t pack_lenth;
    uint16_t pack_size; //包大小
    uint32_t file_lenth;
    uint16_t app_version;
    uint8_t target_id;
    uint8_t target_num;  //目标数量
    bool wait_cbot_en;  //等待CBOT计数器
    int wait_cbot_cnt;

    bool complete_flag;
    bool success_flag;
    bool fail_flag;
    uint8_t error_code;
    uint8_t error_cnt;

}uartCanTypedef;

extern uartTypedef uart;
extern uartCanTypedef uart_can;
enum UART_FSM_ENUM
{
    UART_FSM_IDLE = 0U,
    UART_FSM_INIT,
    UART_FSM_READ_VER,
    UART_FSM_WRITE_DATA,
    UART_FSM_REC_DATA,
    UART_FSM_RCK,
    UART_FSM_CCK,
    UART_FSM_RUN,
    UART_FSM_CBOT, //等待目标板进入bootloader
    UART_FSM_CVCU, //等待目标板进入bootloader
    UART_FSM_CACK,
    UART_FSM_CRUN

};

enum UART_CAN_FSM_ENUM
{
    UART_CAN_FSM_IDLE = 0U,
    UART_CAN_FSM_INIT,
    UART_CAN_FSM_WAIT_CBOT,
    UART_CAN_FSM_CVTU,
    UART_CAN_FSM_WAIT_CVCU,

    UART_CAN_FSM_WRITE_DATA,
    UART_CAN_FSM_WAIT_CACK,
    UART_CAN_FSM_WAIT_CRCK,
    UART_CAN_FSM_FAIL,
    UART_CAN_FSM_RCK,
    UART_CAN_FSM_RUN

};



void InvertUint16(uint16_t *dBuf,uint16_t *srcBuf);
void InvertUint8(uint8_t *dBuf,uint8_t *srcBuf);
uint16_t CRC16_MODBUS(uint8_t *puchMsg, uint16_t usDataLen);


#endif // UART_H
