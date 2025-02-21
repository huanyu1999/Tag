#ifndef __INSTANCE_H
#define __INSTANCE_H

#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <math.h>

#include "port.h"
#include "deca_device_api.h"
#include "deca_regs.h"
#include "deca_types.h"
#include "deca_spi.h"
// #include "mac.h"
#include "main.h"
#include "usart.h"
#include "gpio.h"
// #include "iwdg.h"
#include "kalman.h"

// #include "com_MultiTimer.h"

/***********************************定义板卡角色，只能定义一个*************************************/
#define LD150
/***********************************************************************************************/
// #define ANCRANGE                 //基站间测距，用于基站自标定
/***********************************************************************************************/

#define SOFTWARE_VER                   "V1.1"

#define DEVICE_ID  0x0B

#define MAX_AHCHOR_NUMBER               3    //系统内最大基站数量，取4或者8，比如实际3个取4，实际6个取8
#define MAX_TAG_LIST_SIZE               40   // 设置最大标签个数

#define MAX_ANCHOR_LIST_SIZE            (4) //this is limited to 4 in this application
#define NUM_EXPECTED_RESPONSES			(3) //e.g. MAX_ANCHOR_LIST_SIZE - 1
#define NUM_EXPECTED_RESPONSES_ANC0		(2) //anchor A0 expects response from A1 and A2
#define NUM_EXPECTED_RESPONSES_ANC1		(1) //anchor A1 expects response from A2
#define WAIT4TAGFINAL					2
#define WAIT4ANCFINAL					1

#define NUM_INST                        1   // one instance (tag or anchor - controlling one DW1000)
#define MASK_40BIT                      (0x00FFFFFFFFFF)  // DW1000 counter is 40 bits
#define MASK_TXDTS                      (0x00FFFFFFFE00)  // The TX timestamp will snap to 8 ns resolution - mask lower 9 bits.

#define INST_DONE_WAIT_FOR_NEXT_EVENT       1   //this signifies that the current event has been processed and instance is ready for next one
#define INST_DONE_WAIT_FOR_NEXT_EVENT_TO    2   //this signifies that the current event has been processed and that instance is waiting for next one with a timeout
//which will trigger if no event coming in specified time
#define INST_NOT_DONE_YET                   0   //this signifies that the instance is still processing the current event

#define DEEP_SLEEP (0)//To enable deep-sleep set this to 1
//DEEP_SLEEP mode can be used, for example, by a Tag instance to put the DW1000 into low-power deep-sleep mode while it is
//waiting for start of next ranging exchange

#define CORRECT_RANGE_BIAS  (1)     // Compensate for small bias due to uneven accumulator growth at close up high power

#define ANCTOANCTWR (0)             //if set to 1 then anchor to anchor two-way ranging will be done in the last 2 slots, this
//is used for TREK demo, to aid in anchor installation,

#define TAG_HASTO_RANGETO_A0 (0) //if set to 1 then tag will only send the Final if the Response from A0 has been received

#define READ_EVENT_COUNTERS (0) //read event counters - can be used for debug to periodically output event counters

#define DISCOVERY (0)           //set to 1 to enable tag discovery - tags starts by sending blinks (with own ID) and then
//anchor assigns a slot to it and gives it short address

/* 天线延时
 * 计算距离结果比实际距离小，需要增大距离，则减小这个数
 * 计算距离结果比实际距离大，需要减小距离，则增大这个数
 */
#define ANT_DLY                         16485
#define TX_POWER                        0x1f1f1f1f

/* 数据帧超时及延时时间*/
#define PRE_TIMEOUT                     5

#if (MAX_AHCHOR_NUMBER == 3)            // 每个时隙的持续时间，时隙时间过小，会导致相邻ID的标签，位于后方的标签无法测距，被上一个标签测距所影响，尝试增大该值，但是标签的时隙值不能相同，否则还是一样的情况？
#define ONE_SLOT_TIME_MS_110K           28
#define ONE_SLOT_TIME_MS_850K           12
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
#define FIRST_RESP_SEND_850K            1250     //850K通信速率下，第一个resp消息发送延时
#define DATA_INTERVAL_TIME_850K         1650    //850K通信速率下，相邻消息间隔时间
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

/* 数据帧长度 */
#define POLL_MSG_LEN                    26
#define RESP_MSG_LEN                    19
#define FIANL_MSG_LEN                   (20 + 5 * MAX_AHCHOR_NUMBER)
#define BLINK_MSG_LEN                   10
#define INIT_MSG_LEN                    12
#define SYNC_MSG_LEN                    15

/* Message length define */
#define TAG_POLL_MSG_LEN                2				// FunctionCode(1), Range Num (1)
#define ANCH_RESPONSE_MSG_LEN           8               // FunctionCode(1), Sleep Correction Time (2), Measured_TOF_Time(4), Range Num (1) (previous)
#define TAG_FINAL_MSG_LEN               33              // FunctionCode(1), Range Num (1), Poll_TxTime(5),
// Resp0_RxTime(5), Resp1_RxTime(5), Resp2_RxTime(5), Resp3_RxTime(5), Final_TxTime(5), Valid Response Mask (1)

#define ANCH_POLL_MSG_LEN_S			    2		        // FunctionCode(1), Range Num (1),
#define ANCH_POLL_MSG_LEN               4		        // FunctionCode(1), Range Num (1), Next Anchor (2)
#define ANCH_FINAL_MSG_LEN              33              // FunctionCode(1), Range Num (1), Poll_TxTime(5),
// Resp0_RxTime(5), Resp1_RxTime(5), Resp2_RxTime(5), Resp3_RxTime(5), Final_TxTime(5), Valid Response Mask (1)
#define MAX_MAC_MSG_DATA_LEN            (TAG_FINAL_MSG_LEN) //max message len of the above

#define STANDARD_FRAME_SIZE         127

#define ADDR_BYTE_SIZE_L            (8)
#define ADDR_BYTE_SIZE_S            (2)

#define FRAME_CONTROL_BYTES         2
#define FRAME_SEQ_NUM_BYTES         1
#define FRAME_PANID                 2
#define FRAME_CRC					2
#define FRAME_SOURCE_ADDRESS_S      (ADDR_BYTE_SIZE_S)
#define FRAME_DEST_ADDRESS_S        (ADDR_BYTE_SIZE_S)
#define FRAME_SOURCE_ADDRESS_L      (ADDR_BYTE_SIZE_L)
#define FRAME_DEST_ADDRESS_L        (ADDR_BYTE_SIZE_L)
#define FRAME_CTRLP					(FRAME_CONTROL_BYTES + FRAME_SEQ_NUM_BYTES + FRAME_PANID)   //5
#define FRAME_CRTL_AND_ADDRESS_L    (FRAME_DEST_ADDRESS_L + FRAME_SOURCE_ADDRESS_L + FRAME_CTRLP) //21 bytes for 64-bit addresses)
#define FRAME_CRTL_AND_ADDRESS_S    (FRAME_DEST_ADDRESS_S + FRAME_SOURCE_ADDRESS_S + FRAME_CTRLP) //9 bytes for 16-bit addresses)
#define FRAME_CRTL_AND_ADDRESS_LS	(FRAME_DEST_ADDRESS_L + FRAME_SOURCE_ADDRESS_S + FRAME_CTRLP) //15 bytes for one 16-bit address and one 64-bit address)
#define MAX_USER_PAYLOAD_STRING_LL  (STANDARD_FRAME_SIZE-FRAME_CRTL_AND_ADDRESS_L-TAG_FINAL_MSG_LEN-FRAME_CRC)  //127 - 21 - 16 - 2 = 88
#define MAX_USER_PAYLOAD_STRING_SS  (STANDARD_FRAME_SIZE-FRAME_CRTL_AND_ADDRESS_S-TAG_FINAL_MSG_LEN-FRAME_CRC)  //127 - 9 - 16 - 2 = 100
#define MAX_USER_PAYLOAD_STRING_LS  (STANDARD_FRAME_SIZE-FRAME_CRTL_AND_ADDRESS_LS-TAG_FINAL_MSG_LEN-FRAME_CRC) //127 - 15 - 16 - 2 = 94

#define BLINK_FRAME_CONTROL_BYTES       (1)
#define BLINK_FRAME_SEQ_NUM_BYTES       (1)
#define BLINK_FRAME_CRC					(FRAME_CRC)
#define BLINK_FRAME_SOURCE_ADDRESS      (ADDR_BYTE_SIZE_L)
#define BLINK_FRAME_CTRLP				(BLINK_FRAME_CONTROL_BYTES + BLINK_FRAME_SEQ_NUM_BYTES) //2
#define BLINK_FRAME_CRTL_AND_ADDRESS    (BLINK_FRAME_SOURCE_ADDRESS + BLINK_FRAME_CTRLP) //10 bytes
#define BLINK_FRAME_LEN_BYTES           (BLINK_FRAME_CRTL_AND_ADDRESS + BLINK_FRAME_CRC)

#define GATEWAY_ANCHOR_ADDR				(0x8000)
#define A1_ANCHOR_ADDR					(0x8001)
#define A2_ANCHOR_ADDR					(0x8002)
#define A3_ANCHOR_ADDR					(0x8003)

//application data payload byte offsets
#define FCODE                               0               // Function code is 1st byte of messageData
#define PTXT                                2				// Poll TX time
#define RRXT0                               7				// A0 Response RX time
#define RRXT1                               12				// A1 Response RX time
#define RRXT2                               17				// A2 Response RX time
#define RRXT3                               22				// A3 Response RX time
#define FTXT                                27				// Final TX time
#define VRESP                               32				// Mask of valid response times (e.g. if bit 1 = A0's response time is valid)
#define RES_TAG_SLP0                        1               // Response tag sleep correction LSB
#define RES_TAG_SLP1                        2               // Response tag sleep correction MSB
#define TOFR                                3				// ToF (n-1) 4 bytes
#define TOFRN								7				// range number 1 byte
#define POLL_RNUM                           1               // Poll message range number
#define POLL_NANC							2				// Address of next anchor to send a poll message (e.g. A1)

#define RES_TAG_ADD0                        3               // Tag's short address (slot num) LSB
#define RES_TAG_ADD1                        4               // Tag's short address (slot num) MSB

#define DW_RX_ON_DELAY                      (16)            //us - the DW RX has 16 us RX on delay before it will receive any data

//this it the delay used for configuring the receiver on delay (wait for response delay)
//NOTE: this RX_RESPONSE_TURNAROUND is dependent on the microprocessor and code optimisations
#define RX_RESPONSE_TURNAROUND              (300) //this takes into account any turnaround/processing time (reporting received poll and sending the response)

#define PTO_PACS                            (5)   //tag will use PTO to reduce power consumption (if no response coming stop RX)

//Tag = Exchanges DecaRanging messages (Poll-Response-Final) with Anchor and enabling Anchor to calculate the range between the two instances
//Anchor = see above
//Anchor_Rng = the anchor (assumes a tag function) and ranges to another anchor - used in Anchor to Anchor TWR for auto positioning function
typedef enum instanceModes {TAG, ANCHOR, ANCHOR_RNG, NUM_MODES} INST_MODE;

// instance sending a poll (starting TWR) is INITIATOR
// instance which receives a poll (and will be involved in the TWR) is RESPONDER
// instance which does not receive a poll (default state) will be a LISTENER - will send no responses
typedef enum instanceTWRModes {INITIATOR, RESPONDER_A, RESPONDER_B, RESPONDER_T, LISTENER, GREETER, ATWR_MODES} ATWR_MODE;

#define TOF_REPORT_NUL 0
#define TOF_REPORT_T2A 1
#define TOF_REPORT_A2A 2

#define INVALID_TOF (0xABCDFFFF)

/* 数据帧数组索引 offset */
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
#define INIT_MSG_SLEEP_COR_IDX          10
#define FINAL_MSG_FINAL_VALID_IDX       11
#define FINAL_MSG_POLL_TX_TS_IDX        12
#define FINAL_MSG_FINAL_TX_TS_IDX       16
#define FINAL_MSG_A0_GROUP_ID_IDX       20
#define FINAL_MSG_A1_GROUP_ID_IDX       25
#define FINAL_MSG_A2_GROUP_ID_IDX       30
#define FINAL_MSG_A3_GROUP_ID_IDX       35
#define FINAL_MSG_A4_GROUP_ID_IDX       40
#define FINAL_MSG_A5_GROUP_ID_IDX       45
#define FINAL_MSG_A6_GROUP_ID_IDX       50
#define FINAL_MSG_A7_GROUP_ID_IDX       44
#define FINAL_MSG_RESP1_RX_TS_IDX       21
#define FINAL_MSG_RESP2_RX_TS_IDX       26
#define FINAL_MSG_RESP3_RX_TS_IDX       31
#define FINAL_MSG_RESP4_RX_TS_IDX       36
#define FINAL_MSG_RESP5_RX_TS_IDX       41
#define FINAL_MSG_RESP6_RX_TS_IDX       46
#define FINAL_MSG_RESP7_RX_TS_IDX       51
#define FINAL_MSG_RESP8_RX_TS_IDX       56
#define SYNC_MSG_TIME_IDX               10


/*  function code */
#define FUNC_CODE_POLL                  0x21
#define FUNC_CODE_RESP                  0x10
#define FUNC_CODE_FINAL                 0x23
#define FUNC_CODE_BLINK                 0x36
#define FUNC_CODE_INIT                  0x38
#define FUNC_CODE_SYNC                  0x42

//! callback events
#define DWT_SIG_RX_NOERR            0
#define DWT_SIG_TX_DONE             1       // Frame has been sent
#define DWT_SIG_RX_OKAY             2       // Frame Received with Good CRC
#define DWT_SIG_RX_ERROR            3       // Frame Received but CRC is wrong
#define DWT_SIG_RX_TIMEOUT          4       // Timeout on receive has elapsed
#define DWT_SIG_TX_AA_DONE          6       // ACK frame has been sent (as a result of auto-ACK)
#define DWT_SIG_RX_BLINK            7       // Received ISO EUI 64 blink message
#define DWT_SIG_RX_PHR_ERROR        8       // Error found in PHY Header
#define DWT_SIG_RX_SYNCLOSS         9       // Un-recoverable error in Reed Solomon Decoder
#define DWT_SIG_RX_SFDTIMEOUT       10      // Saw preamble but got no SFD within configured time
#define DWT_SIG_RX_PTOTIMEOUT       11      // Got preamble detection timeout (no preamble detected)
#define DWT_SIG_TX_PENDING          12      // TX is pending
#define DWT_SIG_TX_ERROR            13      // TX failed
#define DWT_SIG_RX_PENDING          14      // RX has been re-enabled
#define DWT_SIG_DW_IDLE             15      // DW radio is in IDLE (no TX or RX pending)
#define SIG_RX_UNKNOWN              99      // Received an unknown frame

//DecaRTLS frame function codes
#define RTLS_DEMO_MSG_RNG_INIT              (0x71)          // Ranging initiation message
#define RTLS_DEMO_MSG_TAG_POLL              (0x81)          // Tag poll message
#define RTLS_DEMO_MSG_ANCH_RESP             (0x70)          // Anchor response to poll
#define RTLS_DEMO_MSG_ANCH_POLL             (0x7A)          // Anchor to anchor poll message
#define RTLS_DEMO_MSG_ANCH_RESP2            (0x7B)          // Anchor response to poll from anchor
#define RTLS_DEMO_MSG_ANCH_FINAL            (0x7C)          // Anchor final massage back to Anchor
#define RTLS_DEMO_MSG_TAG_FINAL             (0x82)          // Tag final massage back to Anchor

/*  function code, differentiate tag and anchor */
#define FUNC_CODE_TAG_POLL              0x71           // Tag poll message
#define FUNC_CODE_ANCH_POLL             0x71           // Anchor to anchor poll message
#define FUNC_CODE_ANCH_RESP_TO_TAG      0x71           // Anchor response to poll
#define FUNC_CODE_ANCH_RESP_TO_ANCH     0x71
#define FUNC_CODE_ANCH_FINAL            0x71
#define FUNC_CODE_TAG_FINAL             0x71
#define FUNC_CODE_TAG_POLL              0x71

typedef enum inst_states
{
    TA_INIT,                    //0

    TA_TXE_WAIT,                //1 - state in which the instance will enter sleep (if ranging finished) or proceed to transmit a message
    TA_TXBLINK_WAIT_SEND,       //2 - configuration and sending of Blink message
    TA_TXPOLL_WAIT_SEND,        //2 - configuration and sending of Poll message
    TA_TXFINAL_WAIT_SEND,       //3 - configuration and sending of Final message
    TA_TXRESPONSE_WAIT_SEND,    //4 - a place holder - response is sent from call back
    TA_TX_WAIT_CONF,            //5 - confirmation of TX done message

    TA_RXE_WAIT,                //6
    TA_RX_WAIT_DATA,            //7

    TA_SLEEP_DONE,               //8
    TA_TXRESPONSE_SENT_POLLRX,   //9
    TA_TXRESPONSE_SENT_RESPRX,   //10
    TA_TXRESPONSE_SENT_TORX,     //11
    TA_TXRESPONSE_SENT_APOLLRX,  //12
    TA_TXRESPONSE_SENT_ARESPRX   //13

} INST_STATES;

// This file defines data and functions for access to Parameters in the Device
//message structure for Poll, Response and Final message

typedef struct
{
    uint8 frameCtrl[2];                         	//  frame control bytes 00-01
    uint8 seqNum;                               	//  sequence_number 02
    uint8 panID[2];                             	//  PAN ID 03-04
    uint8 destAddr[ADDR_BYTE_SIZE_L];             	//  05-12 using 64 bit addresses
    uint8 sourceAddr[ADDR_BYTE_SIZE_L];           	//  13-20 using 64 bit addresses
    uint8 messageData[MAX_USER_PAYLOAD_STRING_LL] ; //  22-124 (application data and any user payload)
    uint8 fcs[2] ;                              	//  125-126  we allow space for the CRC as it is logically part of the message. However ScenSor TX calculates and adds these bytes.
} srd_msg_dlsl ;

typedef struct
{
    uint8 frameCtrl[2];                         	//  frame control bytes 00-01
    uint8 seqNum;                               	//  sequence_number 02
    uint8 panID[2];                             	//  PAN ID 03-04
    uint8 destAddr[ADDR_BYTE_SIZE_S];             	//  05-06
    uint8 sourceAddr[ADDR_BYTE_SIZE_S];           	//  07-08
    uint8 messageData[MAX_USER_PAYLOAD_STRING_SS] ; //  09-124 (application data and any user payload)
    uint8 fcs[2] ;                              	//  125-126  we allow space for the CRC as it is logically part of the message. However ScenSor TX calculates and adds these bytes.
} srd_msg_dsss ;

typedef struct
{
    uint8 frameCtrl[2];                         	//  frame control bytes 00-01
    uint8 seqNum;                               	//  sequence_number 02
    uint8 panID[2];                             	//  PAN ID 03-04
    uint8 destAddr[ADDR_BYTE_SIZE_L];             	//  05-12 using 64 bit addresses
    uint8 sourceAddr[ADDR_BYTE_SIZE_S];           	//  13-14
    uint8 messageData[MAX_USER_PAYLOAD_STRING_LS] ; //  15-124 (application data and any user payload)
    uint8 fcs[2] ;                              	//  125-126  we allow space for the CRC as it is logically part of the message. However ScenSor TX calculates and adds these bytes.
} srd_msg_dlss ;

typedef struct
{
    uint8 frameCtrl[2];                         	//  frame control bytes 00-01
    uint8 seqNum;                               	//  sequence_number 02
    uint8 panID[2];                             	//  PAN ID 03-04
    uint8 destAddr[ADDR_BYTE_SIZE_S];             	//  05-06
    uint8 sourceAddr[ADDR_BYTE_SIZE_L];           	//  07-14 using 64 bit addresses
    uint8 messageData[MAX_USER_PAYLOAD_STRING_LS] ; //  15-124 (application data and any user payload)
    uint8 fcs[2] ;                              	//  125-126  we allow space for the CRC as it is logically part of the message. However ScenSor TX calculates and adds these bytes.
} srd_msg_dssl ;

//12 octets for Minimum IEEE ID blink
typedef struct
{
    uint8 frameCtrl;                         		//  frame control bytes 00
    uint8 seqNum;                               	//  sequence_number 01
    uint8 tagID[BLINK_FRAME_SOURCE_ADDRESS];        //  02-09 64 bit address
    uint8 fcs[2] ;                              	//  10-11  we allow space for the CRC as it is logically part of the message. However ScenSor TX calculates and adds these bytes.
} iso_IEEE_EUI64_blink_msg ;

typedef struct
{
    uint8 channelNumber ;       // valid range is 1 to 11
    uint8 pulseRepFreq ;        // NOMINAL_4M, NOMINAL_16M, or NOMINAL_64M
    uint8 preambleLen ;         // values expected are 64, (128), (256), (512), 1024, (2048), and 4096
    uint8 pacSize ;
    uint8 preambleCode ;        // 00 = use NS code, 1 to 24 selects code
    uint8 nsSFD ;
    uint8 dataRate ;            // DATA_RATE_1 (110K), DATA_RATE_2 (850K), DATA_RATE_3 (6M81)
    uint8 phrMode ;             //!< PHR mode {0x0 - standard DWT_PHRMODE_STD, 0x3 - extended frames DWT_PHRMODE_EXT}
    uint16 sfdTO;               //!< SFD timeout value (in symbols) e.g. preamble length (128) + SFD(8) - PAC + some margin ~ 135us... DWT_SFDTOC_DEF; //default value
} instanceConfig_t ;

typedef struct
{
    uint16 slotDuration_ms ;    //slot duration (time for 1 tag to range to 4 anchors)
    uint16 numSlots ; 		    // number of slots in one superframe (number of tags supported)
    uint16 sfPeriod_ms ;	    // superframe period in ms
    uint16 tagPeriod_ms ; 	    // the time during which tag ranges to anchors and then sleeps, should be same as FRAME PERIOD so that tags don't interfere
    uint16 pollTxToFinalTxDly_us ; //response delay time (Poll to Final delay)
} sfConfig_t ;                  // super frame config

//size of the event queue, in this application there should be at most 2 unprocessed events,
//i.e. if there is a transmission with wait for response then the TX callback followed by RX callback could be executed
//in turn and the event queued up before the instance processed the TX event.
#define MAX_EVENT_NUMBER (4)

typedef struct
{
    uint8  type;			// event type - if 0 there is no event in the queue
    //uint8  typeSave;		// holds the event type - does not clear (used to show what event has been processed)
    uint8  typePend;	    // set if there is a pending event (i.e. DW is not in IDLE (TX/RX pending)
    uint16 rxLength ;		// length of RX data (does not apply to TX events)

    uint64 timeStamp ;		// last timestamp (Tx or Rx) - 40 bit DW1000 time

    uint32 timeStamp32l ;		   // last tx/rx timestamp - low 32 bits of the 40 bit DW1000 time
    uint32 timeStamp32h ;		   // last tx/rx timestamp - high 32 bits of the 40 bit DW1000 time

    uint32 uTimeStamp ;			  //32 bit system counter (ms) - STM32 tick time (at time of IRQ)

    union {
        // holds received frame (after a good RX frame event)
        uint8   frame[STANDARD_FRAME_SIZE];
        srd_msg_dlsl rxmsg_ll ; //64 bit addresses
        srd_msg_dssl rxmsg_sl ;
        srd_msg_dlss rxmsg_ls ;
        srd_msg_dsss rxmsg_ss ; //16 bit addresses
        // iso_IEEE_EUI64_blink_msg rxblinkmsg;
    } msgu;

    //uint8 gotit;			//stores the instance function which processed the event (used for debug)
} event_data_t;

// TX power and PG delay configuration structure
typedef struct {
    uint8 pgDelay;

    //TX POWER
    //31:24     BOOST_0.125ms_PWR
    //23:16     BOOST_0.25ms_PWR-TX_SHR_PWR
    //15:8      BOOST_0.5ms_PWR-TX_PHR_PWR
    //7:0       DEFAULT_PWR-TX_DATA_PWR
    uint32 txPwr[2]; //
} tx_struct;

typedef struct
{
    INST_MODE mode;				//instance mode (tag or anchor)
    ATWR_MODE twrMode;
    INST_STATES testAppState ;			//state machine - current state
    INST_STATES nextState ;				//state machine - next state
    INST_STATES previousState ;			//state machine - previous state

    //configuration structures
    dwt_config_t    configData ;	//DW1000 channel configuration
    dwt_txconfig_t  configTX ;		//DW1000 TX power configuration
    uint16			txAntennaDelay ; //DW1000 TX antenna delay
    uint16			rxAntennaDelay ; //DW1000 RX antenna delay
    uint32 			txPower ;		 //DW1000 TX power
    uint8 txPowerChanged ;			//power has been changed - update the register on next TWR exchange
    uint8 antennaDelayChanged;		//antenna delay has been changed - update the register on next TWR exchange

    uint16 instanceAddress16; //contains tag/anchor 16 bit address

    //timeouts and delays
    int32 tagPeriod_ms; // in ms, tag ranging + sleeping period
    int32 tagSleepTime_ms; //in milliseconds - defines the nominal Tag sleep time period
    int32 tagSleepRnd_ms; //add an extra slot duration to sleep time to avoid collision before getting synced by anchor 0

    //this is the delay used for the delayed transmit
    uint64 pollTx2FinalTxDelay ;        // this is delay from Poll Tx time to Final Tx time in DW1000 units (40-bit)
    uint64 pollTx2FinalTxDelayAnc ;     // this is delay from Poll Tx time to Final Tx time in DW1000 units (40-bit) for Anchor to Anchor ranging
    uint32 fixedReplyDelayAnc32h ;      // this is a delay used for calculating delayed TX/delayed RX on time (units: 32bit of 40bit DW time)
    uint16 anc1RespTx2FinalRxDelay_sy ; // This is delay for RX on when A1 expecting Final form A0 or A2 expecting Final from A1
    uint16 anc2RespTx2FinalRxDelay_sy ; // This is delay for RX on when A2 waiting for A0's final (anc to anc ranging)
    uint32 preambleDuration32h ;        // preamble duration in device time (32 MSBs of the 40 bit time)
    uint32 tagRespRxDelay_sy ;          // TX to RX delay time when tag is awaiting response message an another anchor
    uint32 ancRespRxDelay_sy ;          // TX to RX delay time when anchor is awaiting response message an another anchor

    int fwtoTime_sy ;                   // this is FWTO for response message (used by initiating anchor in anchor to anchor ranging)
    int fwto4RespFrame_sy ;             // this is a frame wait timeout used when awaiting reception of Response frames (used by both tag/anchor)
    int fwto4FinalFrame_sy ;            // this is a frame wait timeout used when awaiting reception of Final frames
    uint32 delayedTRXTime32h;           // time at which to do delayed TX or delayed RX (note TX time is time of SFD, RX time is RX on time)

    //message structure used for holding the data of the frame to transmit before it is written to the DW1000
    srd_msg_dsss msg_f ;                // ranging message frame with 16-bit addresses
    iso_IEEE_EUI64_blink_msg blinkmsg ; // frame structure (used for tx blink message)
    srd_msg_dlss rng_initmsg ;          // ranging init message (destination long, source short)

    //Tag function address/message configuration
    uint8   shortAdd_idx ;				// device's 16-bit address low byte (used as index into arrays [0 - 3])
    uint8   eui64[8];				// device's EUI 64-bit address
    uint16  psduLength ;			// used for storing the TX frame length
    uint8   frameSN;				// modulo 256 frame sequence number - it is incremented for each new frame transmission
    uint16  panID ;					// panid used in the frames

    //64 bit timestamps
    //union of TX timestamps
    union {
        uint64 txTimeStamp ;		   // last tx timestamp
        uint64 tagPollTxTime ;		   // tag's poll tx timestamp
        uint64 anchorRespTxTime ;	   // anchor's reponse tx timestamp
    } txu;
    uint32 tagPollTxTime32h ;
    uint64 tagPollRxTime ;          // receive time of poll message


    //application control parameters
    uint8	wait4ack ;				// if this is set to DWT_RESPONSE_EXPECTED, then the receiver will turn on automatically after TX completion
    uint8   wait4final ;

    uint8   instToSleep;			// if set the instance will go to sleep before sending the blink/poll message
    uint8	instanceTimerEn;		// enable/start a timer
    uint32	instanceWakeTime_ms;    // micro time at which the tag was waken up
    uint32  nextWakeUpTime_ms;		// micro time at which to wake up tag

    uint8   rxResponseMaskAnc;		// bit mask - bit 0 not used;
    // bit 1 = received response from anchor ID = 1;
    // bit 2 from anchor ID = 2,
    // bit 3 set if two responses (from Anchor 1 and Anchor 2) received and A0 got third response (from A2)

    uint8   rxResponseMask;			// bit mask - bit 0 = received response from anchor ID = 0, bit 1 from anchor ID = 1 etc...
    uint8   rxResponseMaskReport;   // this will be set before outputting range reports to signify which are valid
    uint8	rangeNum;				// incremented for each sequence of ranges (each slot)
    uint8	rangeNumA[MAX_TAG_LIST_SIZE];				// array which holds last range number from each tag
    uint8	rangeNumAnc;			// incremented for each sequence of ranges (each slot) - anchor to anchor ranging
    uint8	rangeNumAAnc[MAX_ANCHOR_LIST_SIZE]; //array which holds last range number for each anchor

    int8    rxResps;				// how many responses were received to a poll (in current ranging exchange)
    int8    remainingRespToRx ;		// how many responses remain to be received (in current ranging exchange)

    uint16  sframePeriod_ms;		// superframe period in ms
    uint16  slotDuration_ms;		// slot duration in ms
    uint16  numSlots;
    uint32  a0SlotTime_ms;			// relative time in superframe at which A0 starts ranging (this is start of 2nd last slot)
    uint32  a1SlotTime_ms;			// absolute time in superframe at which A1 starts ranging
    uint32  a2aStartTime_ms;		// absolute time in superframe at which A0 starts ranging
    int32   tagSleepCorrection_ms;  // tag's sleep correction to keep it in it's assigned slot

    //diagnostic counters/data, results and logging
    uint32 tof[MAX_TAG_LIST_SIZE]; //this is an array which holds last ToF from particular tag (ID 0-(MAX_TAG_LIST_SIZE-1))

    //this is an array which holds last ToF to each anchor it should
    uint32 tofArray[MAX_ANCHOR_LIST_SIZE]; //contain 4 ToF to 4 anchors all relating to same range number sequence

    uint32 tofAnc[MAX_ANCHOR_LIST_SIZE]; //this is an array which holds last ToFs from particular anchors (0, 0-1, 0-2, 1-2)

    //this is an array which holds last ToFs of the Anchor to Anchor ranging
    uint32 tofArrayAnc[MAX_ANCHOR_LIST_SIZE]; //it contains 3 ToFs relating to same range number sequence (0, 0-1, 0-2, 1-2)

#if (DISCOVERY ==1)
    uint8 tagListLen ;
    uint8 tagList[MAX_TAG_LIST_SIZE][8];
#endif

    //debug counters
    //int txMsgCount; //number of transmitted messages
    //int rxMsgCount; //number of received messages
    //int rxTimeouts ; //number of received timeout events
    //int lateTX; //number of "LATE" TX events
    //int lateRX; //number of "LATE" RX events


    //ranging counters
    int longTermRangeCount ; //total number of ranges

    int newRange;			//flag set when there is a new range to report TOF_REPORT_A2A or TOF_REPORT_T2A
    int newRangeAncAddress; //last 4 bytes of anchor address - used for printing/range output display
    int newRangeTagAddress; //last 4 bytes of tag address - used for printing/range output display
    int newRangeTime;

    uint8 gatewayAnchor ; //set to TRUE = 1 if anchor address == GATEWAY_ANCHOR_ADDR

    //event queue - used to store DW1000 events as they are processed by the dw_isr/callback functions
    event_data_t dwevent[MAX_EVENT_NUMBER]; //this holds any TX/RX events and associated message data
    uint8 dweventIdxOut;
    uint8 dweventIdxIn;
    uint8 dweventPeek;
    uint8 monitor;
    uint32 timeofTx ;

    uint8 smartPowerEn;

#if (READ_EVENT_COUNTERS == 1)
    dwt_deviceentcnts_t ecounters;
#endif

} instance_data_t ;

/* uwb 射频配置枚举 */
typedef enum {
    CONFIG_BR_110K_1,
    CONFIG_BR_6M8_1,
    CONFIG_BR_6M8_2,
    CONFIG_BR_850K_1,
    CONFIG_BR_850K_2,
    CONFIG_BR_NUM
} rf_config_e;

/* 状态机标志位，保留 */
typedef enum {
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
typedef enum {
    RANGE_NULL,
    RANGE_TWR_OK,
    RANGE_ERROR
} twrStatus;

#define ANCHOR_SELF_DIS  0
#define OTHER_ANCHOR_DIS 1
#define FINAL_DIS        2
#define MIN_DIS_QUEUE_LEN 3

typedef struct {            /* 三种距离状态 */
    int32_t dis_value;
    uint8_t dis_class;
} distanceData_t;

// extern uint8_t instance_mode;                           //设备运行角色
extern uint8_t dev_id;                                  //设备ID
extern uint8_t switch8;
extern uint8_t anc_id;
extern uint8_t tag_id;
extern uint8_t group_id;                                //组ID
extern uint8_t state;
extern int32_t distance_report[8];
extern int32_t sort_distance[MAX_TAG_LIST_SIZE];
extern int32_t group_report[8];                         //基站组ID数组，用于打包输出
extern uint32_t range_time;
extern uint8_t inst_ch;                                 //信道号Channel number
extern uint8_t inst_prf;                                //PRF
extern uint8_t frame_seq_nb;                            //每帧数据增加1
extern uint8_t range_nb;                                //每次range增加1(poll resp1~4 fianl维护一套range_nb)
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
extern int32 distance_offset_cm;                        //距离校准，单位cm
extern uint8_t sos;
extern uint8_t alarm;
extern uint8_t battery;
extern uint8_t USE_IMU;
extern int user_data[10];
extern uint32_t distance_flag;



#if defined(ANCRANGE)
extern uint8_t temp_dev_id;
extern uint8_t ancrange_flag;
extern uint8_t ancrange_count;
extern uint8_t target_ancid;
#endif

/*******************************************dw_main.c****************************************/
uint32 init_uwbApplication(void);
int dw_main(void);
void print_config(void);
double calculate_RSSI(dwt_rxdiag_t* rx_diag);

void anchor_app(void);
void tag_app(void);

//void tag_rx_ok_cb(const dwt_cb_data_t *cb_data);
//void tag_rx_to_cb(const dwt_cb_data_t *cb_data);
//void tag_rx_err_cb(const dwt_cb_data_t *cb_data);
//void tag_tx_conf_cb(const dwt_cb_data_t *cb_data);



//-------------------------------------------------------------------------------------------------------------
//
//	Functions used in logging/displaying range and status data
//
//-------------------------------------------------------------------------------------------------------------

// function to calculate and the range from given Time of Flight
int instance_calculate_rangefromTOF(int idx, uint32 tofx);

void instance_cleardisttable(int idx);
void instance_set_tagdist(int tidx, int aidx);
double instance_get_tagdist(int idx);

double instance_get_idist(int idx);
double instance_get_idistraw(int idx);
int instance_get_idist_mm(int idx);
int instance_get_idistraw_mm(int idx);
uint8 instance_validranges(void);

int instance_get_rnum(void);
int instance_get_rnuma(int idx);
int instance_get_rnumanc(int idx);
int instance_get_lcount(void);

int instance_newrangeancadd(void);
int instance_newrangetagadd(void);
int instance_newrangepolltim(void);
int instance_newrange(void);
int instance_newrangetim(void);

int instance_calc_ranges(uint32 *array, uint16 size, int reportRange, uint8* mask);

// clear the status/ranging data
void instance_clearcounts(void) ;

void instance_cleardisttableall(void);

//-------------------------------------------------------------------------------------------------------------
//
//	Functions used in driving/controlling the ranging application
//
//-------------------------------------------------------------------------------------------------------------

// Call init, then call config, then call run.
// initialise the instance (application) structures and DW1000 device
int instance_init(int role);
// configure the instance and DW1000 device
void instance_config(instanceConfig_t *config, sfConfig_t *sfconfig) ;

// configure the MAC address
void instance_set_16bit_address(uint16 address) ;
void instance_config_frameheader_16bit(instance_data_t *inst);

void tag_process_rx_timeout(instance_data_t *inst);

// called (periodically or from and interrupt) to process any outstanding TX/RX events and to drive the ranging application
int tag_run(void) ;
int anch_run(void) ;       // returns indication of status report change

// configure TX/RX callback functions that are called from DW1000 ISR
void rx_ok_cb_tag(const dwt_cb_data_t *cb_data);
void rx_to_cb_tag(const dwt_cb_data_t *cb_data);
void rx_err_cb_tag(const dwt_cb_data_t *cb_data);
//void tx_conf_cb_tag(const dwt_cb_data_t *cb_data);

void rx_ok_cb_anch(const dwt_cb_data_t *cb_data);
void rx_to_cb_anch(const dwt_cb_data_t *cb_data);
void rx_err_cb_anch(const dwt_cb_data_t *cb_data);

void tx_conf_cb(const dwt_cb_data_t *cb_data);

void anc_rx_ok_cb(const dwt_cb_data_t *cb_data);
void anc_rx_to_cb(const dwt_cb_data_t *cb_data);
void anc_rx_err_cb(const dwt_cb_data_t *cb_data);
// void anc_tx_conf_cb(const dwt_cb_data_t *cb_data);

void instance_set_replydelay(int delayms);

// set/get the instance roles e.g. Tag/Anchor
// done though instance_init void instance_set_role(int mode) ;                //
int instance_get_role(void) ;
// get the DW1000 device ID (e.g. 0xDECA0130 for DW1000)
uint32 instance_readdeviceid(void) ;                                 // Return Device ID reg, enables validation of physical device presence

void rnganch_change_back_to_anchor(instance_data_t *inst);
int instance_send_delayed_frame(instance_data_t *inst, int delayedTx);

uint64 instance_convert_usec_to_devtimeu (double microsecu);


void instance_seteventtime(event_data_t *dw_event, uint8* timeStamp);

int instance_peekevent(void);

void instance_saveevent(event_data_t newevent, uint8 etype);

event_data_t instance_getsavedevent(void);

void instance_putevent(event_data_t newevent, uint8 etype);

event_data_t* instance_getevent(int x);

void instance_clearevents(void);

void instance_notify_DW1000_inIDLE(int idle);

// configure the antenna delays
void instance_config_antennadelays(uint16 tx, uint16 rx);
void instance_set_antennadelays(void);
uint16 instance_get_txantdly(void);
uint16 instance_get_rxantdly(void);

// configure the TX power
void instance_config_txpower(uint32 txpower);
void instance_set_txpower(void);
int instance_starttxtest(int framePeriod);


instance_data_t* instance_get_local_structure_ptr(unsigned int x);

#endif
