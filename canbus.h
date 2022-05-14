#ifndef CANBUS_H
#define CANBUS_H

#include "mainwindow.h"
#include "ui_mainwindow.h"




#define EBD 0xB0
#define BOT 0xB1
#define VTU 0xB2
#define VCU 0xB3
#define DAT 0xB4
#define CCK 0xB5
#define RCK 0xB6
#define RUN 0xB7
#define RVN 0xB8
#define UER 0xBF

//UART-CAN 升级指令

#define CEBD 0xC0
#define CBOT 0xC1
#define CVTU 0xC2
#define CVCU 0xC3
#define CDAT 0xC4
#define CACK 0xC5
#define CRCK 0xC6
#define CRUN 0xC7
#define CRVN 0xC8
#define CUER 0xCF

//App区指令表

#define	HBT 0x00
#define	RHB 0X01
#define	RIS 0X02
#define	FIS 0X03
#define	OPD 0X10
#define	CTB 0X11
#define	CTCH 0X12
#define	CTF 0X13
#define	CTP 0X14
#define	CLG 0X15
#define	CLR 0X16
#define	RBF 0X20
#define	TBF 0X21

typedef struct
{
    uint8_t fsm_status;
    int pack_cnt;  //数据包计数器
    uint32_t frame_cnt; //帧计数器
    uint8_t cash_data[1024];
    uint8_t tx_data[8];
    uint16_t crc;
    char hw_model[4];
    char app_version[2];
    char boot_version[2];
    bool send_data_flag;
    uint32_t file_lenth;
    int pack_lenth;
    int pack_size; //包大小

    bool wait_cnt_en;
    int wait_cnt;  //100ms计数器

    bool wait_rck_cnt_en;  //等待RCK计数器
    int wait_rck_cnt;

    bool complete_flag;
    bool success_flag;
    bool fail_flag;


}canBootTypeDef;


typedef struct
{
    bool en;
     uint8_t fsm_status;
     uint8_t target_cnt;
     uint8_t start_id;
     uint8_t end_id;


}uDTypeDef;


enum CAN_FSM_ENUM
{
    CAN_FSM_READ_INF = 0U,
    CAN_FSM_INIT ,
    CAN_FSM_WRITE_DATA,
    CAN_FSM_CCK,
    CAN_FSM_SUCCESS,
    CAN_FSM_ERROR,
    CAN_FSM_RUN

};

enum UNI_FSM_ENUM
{
    UNI_FSM_INIT = 0U,
    UNI_FSM_CHECK_BOOT,
    UNI_FSM_WAIT_RESULT,

};

extern uDTypeDef unified;
extern canBootTypeDef canBoot;

#endif // CANBUS_H
