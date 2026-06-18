#ifndef __INSTANCE_H
#define __INSTANCE_H

#include <stdio.h>
#include <string.h>

/******** UWB 芯片选择：双目标条件编译，由 CMake 目标提供 ************************************
 * TAG_DW1000 → 定义 USE_DW1000；TAG_DW3000 → 定义 USE_DW3000。
 * 各自的厂家驱动 / 平台头文件分别存放于 Core/Dw1000、Core/Dw3000（CMake 按目标加入 include 路径）。
 *******************************************************************************************/
#if defined(USE_DW3000)
#include "dw3000_port.h"
#include "deca_device_api.h"
#include "deca_regs.h"
#include "deca_types.h"
#include "deca_spi.h"
/* DW1000 deca_types.h 提供 Decawave 短类型别名(uint8/uint16/.../int32)，DW3000 SDK 不提供，
 * 而应用层与 TWR 算法沿用 DW1000 风格，这里补齐（仅 DW3000 目标生效，与 DW1000 互斥）。 */
#ifndef _DECA_UINT8_
#define _DECA_UINT8_
typedef uint8_t  uint8;
#endif
#ifndef _DECA_UINT16_
#define _DECA_UINT16_
typedef uint16_t uint16;
#endif
#ifndef _DECA_UINT32_
#define _DECA_UINT32_
typedef uint32_t uint32;
#endif
#ifndef _DECA_INT8_
#define _DECA_INT8_
typedef int8_t   int8;
#endif
#ifndef _DECA_INT16_
#define _DECA_INT16_
typedef int16_t  int16;
#endif
#ifndef _DECA_INT32_
#define _DECA_INT32_
typedef int32_t  int32;
#endif
/* DW3000 SDK 未提供以下常量/宏，取值与 DW1000 deca_device_api.h 一致，TWR 算法共用 */
#ifndef UUS_TO_DWT_TIME
#define UUS_TO_DWT_TIME  65536
#endif
#ifndef FRAME_LEN_MAX
#define FRAME_LEN_MAX    (127)
#endif
#ifndef FINAL_MSG_TS_LEN
#define FINAL_MSG_TS_LEN 4
#endif
#ifndef SPEED_OF_LIGHT
#define SPEED_OF_LIGHT   (299702547.0)
#endif
#ifndef FCS_LEN
#define FCS_LEN          (2)
#endif
#elif defined(USE_DW1000)
#include "../DW1000/platform/port_dw1000.h"
#include "../DW1000/decadriver/deca_device_api.h"
#include "../DW1000/decadriver/deca_regs.h"
#include "../Dw1000/decadriver/deca_types.h"
#include "../DW1000/platform/deca_spi.h" 
#else
// #error "请定义 USE_DW1000 或 USE_DW3000（由 CMake 目标 TAG_DW1000 / TAG_DW3000 提供）"
#include "../DW3000/platform/port_dw1000.h"
#include "../DW3000/decadriver/deca_device_api.h"
#include "../DW3000/decadriver/deca_regs.h"
#include "../Dw3000/decadriver/deca_types.h"
#include "../DW3000/platform/deca_spi.h"
#endif

#include "usart.h"
//#include "ssd1306.h"
#include "kalman.h"
//#include "e2prom.h"
//#include "cw2015.h"
//#include "trilateration.h"
//#include "iwdg.h"

/********定义板卡角色，只能定义一个*************************************/
#define LD150
/***********************************************************************************************/
// #define ANCRANGE             //基站间测距，用于基站自标定
/***********************************************************************************************/


#define SOFTWARE_VER                   "V1"

#define MAX_AHCHOR_NUMBER               2       // 系统内最大基站数量，取4或者8，比如实际3个取4，实际6个取8

#define LOST_TIMEOUT_MS                 5000    // 基站通信超时阈值（ms），超过此时间未收到有效测距即判定失联
#define MAX_TAG_NUMBER                  50      // 设置最大标签个数

/* 天线延时
 * 计算距离结果比实际距离小，需要增大距离，则减小这个数
 * 计算距离结果比实际距离大，需要减小距离，则增大这个数
 */                                                                                                               

#define ANT_DLY                         16485
/* 发射功率，目前设定为最大值 */
#if defined(USE_DW3000)
#define TX_POWER                        0xfdfdfdfd
#elif defined(USE_DW1000)
#define TX_POWER                        0x1f1f1f1f
#endif

#define MAX_TAG_LIST_SIZE       (MAX_TAG_NUMBER)
#define MASK_40BIT              (0x00FFFFFFFFFF)  // DW1000 counter is 40 bits
#define MASK_TXDTS              (0x00FFFFFFFE00)  // The TX timestamp will snap to 8 ns resolution - mask lower 9 bits.

/* 数据帧超时及延时时间*/
#define PRE_TIMEOUT                     5

#if (MAX_AHCHOR_NUMBER == 3)
#define ONE_SLOT_TIME_MS_110K           28
#define ONE_SLOT_TIME_MS_850K           12
#define ONE_SLOT_TIME_MS_6P8M           9
#elif (MAX_AHCHOR_NUMBER == 2)
#define ONE_SLOT_TIME_MS_110K           28
#define ONE_SLOT_TIME_MS_850K           8       // 一个标签同多基站通信完成所需要的时间
#define ONE_SLOT_TIME_MS_6P8M           9
#elif (MAX_AHCHOR_NUMBER == 8)
#define ONE_SLOT_TIME_MS_110K           50
#define ONE_SLOT_TIME_MS_850K           20
#define ONE_SLOT_TIME_MS_6P8M           15
#endif

#define FINAL_RX_TIMEOUT_6P8M           600
#define RESP_RX_TIMEOUT_6P8M            450
#define FIRST_RESP_SEND_6P8M            900     //6.8M通信速率下，第一个resp消息发送延时
#define DATA_INTERVAL_TIME_6P8M         1100    //6.8M通信速率下，相邻消息间隔时间
#define ANC_RESP_SEND_BACK_6P8M         100     //6.8M通信速率下，基站延后发送RESP消息时间
#define TAG_FINALE_SEND_BACK_6P8M       100     //6.8M通信速率下，标签延后发送FINAL消息时间

#define FINAL_RX_TIMEOUT_110K           6000
#define RESP_RX_TIMEOUT_110K            3800
#define FIRST_RESP_SEND_110K            3000    //110K通信速率下，第一个resp消息发送延时
#define DATA_INTERVAL_TIME_110K         3900    //110K通信速率下，相邻消息间隔时间
#define ANC_RESP_SEND_BACK_110K         1080    //110K通信速率下，基站延后发送RESP消息时间
#define TAG_FINALE_SEND_BACK_110K       1080    //110K通信速率下，标签延后发送FINAL消息时间

#define FINAL_RX_TIMEOUT_850K           1300
#define RESP_RX_TIMEOUT_850K            1000
#define FIRST_RESP_SEND_850K            1300    //850K通信速率下，第一个resp消息发送延时
#define DATA_INTERVAL_TIME_850K         1600    //850K通信速率下，相邻消息间隔时间
#define ANC_RESP_SEND_BACK_850K         300     //850K通信速率下，基站延后发送RESP消息时间
#define TAG_FINALE_SEND_BACK_850K       300     //850K通信速率下，标签延后发送FINAL消息时间

#define MAX_POLL_SEND_SLEEP_COUNT       150     //MAX_POLL_SEND_SLEEP_COUNT次发送后无运动则进入休眠
#define ANC_RANGE_COUNT                 5       //自标定时每个基站测距次数

/* PAN ID */
#define PAN_ID                          0xDECA

/* 中断状态标志 */
#define RX_WAIT                         0
#define TX_WAIT                         0
#define RX_OK                           1
#define TX_OK                           1
#define RX_TIMEOUT                      2
#define RX_ERROR                        3

/* 数据帧长度，一定要留够空间，不然测距异常 */
#define POLL_MSG_LEN                    26
#define RESP_MSG_LEN                    19
#define FIANL_MSG_LEN                   (22 + 5 * MAX_AHCHOR_NUMBER + 10)
#define BLINK_MSG_LEN                   10
#define INIT_MSG_LEN                    12
#define SYNC_MSG_LEN                    15

/* 数据帧数组索引 */
#define SEQ_NB_IDX                      2
#define PANID_IDX                       3
#define RECEIVER_SHORT_ADD_IDX          5
#define SENDER_SHORT_ADD_IDX            7
#define FUNC_CODE_IDX                   9
#define RANGE_NB_IDX                    10

#define POLL_MSG_SOS_IDX                11
#define POLL_MSG_ALARM_STA_IDX          12
#define POLL_MSG_BATTERY_IDX            13
#define POLL_MSG_USER_IDX               14

#define RESP_MSG_SLEEP_COR_IDX          11
#define RESP_MSG_PREV_DIS_IDX           13
#define RESP_MSG_ALARM_IDX              17
#define RESP_MSG_GROUP_IDX              18

// #define INIT_MSG_SLEEP_COR_IDX          10

#define FINAL_MSG_FINAL_VALID_IDX       11
#define FINAL_MSG_POLL_TX_TS_IDX        12
#define FINAL_MSG_FINAL_TX_TS_IDX       17
#define FINAL_MSG_A0_GROUP_ID_IDX       22
#define FINAL_MSG_RESP1_RX_TS_IDX       23
#define FINAL_MSG_RESP2_RX_TS_IDX       28
// #define FINAL_MSG_A0_GROUP_ID_IDX       20
// #define FINAL_MSG_RESP1_RX_TS_IDX       21
// #define FINAL_MSG_RESP2_RX_TS_IDX       26


// #define FINAL_MSG_RESP3_RX_TS_IDX       31
// #define FINAL_MSG_RESP4_RX_TS_IDX       36
// #define FINAL_MSG_RESP5_RX_TS_IDX       41
// #define FINAL_MSG_RESP6_RX_TS_IDX       46
// #define FINAL_MSG_RESP7_RX_TS_IDX       51
// #define FINAL_MSG_RESP8_RX_TS_IDX       56

// #define SYNC_MSG_TIME_IDX               10
// #define FINAL_MSG_A1_GROUP_ID_IDX       25
// #define FINAL_MSG_A2_GROUP_ID_IDX       30
// #define FINAL_MSG_A3_GROUP_ID_IDX       35
// #define FINAL_MSG_A4_GROUP_ID_IDX       40
// #define FINAL_MSG_A5_GROUP_ID_IDX       45
// #define FINAL_MSG_A6_GROUP_ID_IDX       50
// #define FINAL_MSG_A7_GROUP_ID_IDX       44


/*  function code */
#define FUNC_CODE_POLL                  0x21
#define FUNC_CODE_RESP                  0x10
#define FUNC_CODE_FINAL                 0x23
#define FUNC_CODE_BLINK                 0x36
#define FUNC_CODE_INIT                  0x38
#define FUNC_CODE_SYNC                  0x42

/* 拨码开关键值 */
#define SWS1_IMU_MODE                   0x80	   //IMU标签 on=输出IMU数据， off=输出正常mc数据
#define SWS1_SHF_MODE                   0x40	   //默认off=10标签，100ms更新一次，on=1标签，10ms更新一次，可修改宏定义MAX_TAG_NUMBER_SW2_OFF 和 MAX_TAG_NUMBER_SW2_ON
#define SWS1_HPR_MODE                   0x20	   //外部功耗增加开关
#define SWS1_ROLE_MODE                  0x10     //工作模式0=tag 1=anchor
#define SWS1_A1A_MODE                   0x08     //anchor/tag address A1
#define SWS1_A2A_MODE                   0x04     //anchor/tag address A2
#define SWS1_A3A_MODE                   0x02     //anchor/tag address A3
#define SWS1_KAM_MODE                   0x01     //卡尔曼滤波开关


#define MIN_DISTANCE 					100.0		//最小报警距离
/* 状态机标志位 */
typedef enum
{
    STA_IDLE, 
    STA_SEND_POLL,
    STA_WAIT_RESP,
    STA_RECV_RESP,
    STA_SEND_FINAL,
    STA_INIT_POLL_SYNC,
    STA_WAIT_POLL_SYNC,
    STA_RECV_POLL_SYNC,
    STA_SEND_RESP,
    STA_WAIT_FINAL,
    STA_RECV_FINAL,
    STA_SORR_RESP,
    STA_SEND_BLINK,
    STA_WAIT_INIT,
    STA_RECV_INIT,
    STA_SEND_SYNC
} instStatus;


/* TWR测距状态 */
typedef enum
{
    RANGE_NULL, 
    RANGE_TWR_OK,
    RANGE_ERROR
} twrStatus;

/* 系统运行角色 */
typedef enum
{
    TAG, 
    ANCHOR
} instanceModes;

extern uint8_t instance_mode;                         //设备运行角色
extern uint8_t dev_id;                                         //设备ID
extern uint8_t switch8;
extern uint8_t anc_id;
extern uint8_t tag_id;
extern uint8_t group_id;                                       //组ID
extern uint8_t state;
extern int32_t distance_report[8];
extern int32_t group_report[8];   	//基站组ID数组，用于打包输出
extern uint32_t range_time;
extern uint8_t inst_ch;           	//信道号Channel number
extern uint8_t inst_prf;          	//PRF
extern uint8_t frame_seq_nb;  			//每帧数据增加1
extern uint8_t range_nb;      			//每次range增加1(poll resp1~4 fianl维护一套range_nb)
extern uint8_t recv_tag_id;
extern uint8_t recv_anc_id;
extern uint8_t range_status;
extern float rx_power;
extern uint16_t inst_slot_number;
extern uint8_t inst_dataRate; 
extern uint8_t inst_one_slot_time;
extern uint32_t inst_final_rx_timeout;
extern uint32_t inst_resp_rx_timeout;
extern uint32_t inst_init_rx_timeout;                          
extern uint64_t inst_poll2final_time;
extern uint32_t inst_data_interval;
extern uint16 ant_dly;
extern double distance_now_m;  
extern int32 distance_offset_cm;                               //距离校准，单位cm
extern uint8_t sos;
extern uint8_t alarm;
extern uint8_t battery;
extern uint8_t USE_IMU;
extern int user_data[10];
extern uint32_t distance_flag;
extern int lost_flag;
extern uint32_t last_range_ok_tick;
extern uint32_t right_work;  //正常工作

extern volatile uint8_t dw1000_spiDmaCpltFlag;
extern volatile uint8_t dw1000_spiDmaBusyFlag;

#if defined(ANCRANGE)
extern uint8_t temp_dev_id;
extern uint8_t ancrange_flag;
extern uint8_t ancrange_count;
extern uint8_t target_ancid;
#endif

void anchor_app(void);
void tag_app(void);
void dw_init(void);
void print_config(void);                     //打印系统参数信息
extern void set_instance(void);

void tag_rx_ok_cb(const dwt_cb_data_t *cb_data);
void tag_rx_to_cb(const dwt_cb_data_t *cb_data);
void tag_rx_err_cb(const dwt_cb_data_t *cb_data);
void tag_tx_conf_cb(const dwt_cb_data_t *cb_data);

void anc_rx_ok_cb(const dwt_cb_data_t *cb_data);
void anc_rx_to_cb(const dwt_cb_data_t *cb_data);
void anc_rx_err_cb(const dwt_cb_data_t *cb_data);
void anc_tx_conf_cb(const dwt_cb_data_t *cb_data);

void delay500ms(void);
#endif
