#include "instance.h"

// uint8_t switch8 = 0;                                    //拨码开关键值
// uint8_t instance_mode = ANCHOR;                         //设备运行角色
uint8_t dev_id;                                         //设备ID
// uint8_t group_id;                                       //组ID
// uint8_t anc_id;                                         //如当前角色是基站，则表示当前基站ID
uint8_t tag_id;                                         //如当前角色是标签，则表示当前标签ID
// uint8_t state = STA_IDLE;                               //状态机状态控制
int32_t distance_report[8];                             //基站测距值数组，用于打包输出
// int32_t previous_sort_distance[MAX_TAG_LIST_SIZE] = {-1}; // 
int32_t sort_distance[MAX_TAG_LIST_SIZE] = {-1};        //用于标签距离排序
// int32_t group_report[8];                                //基站组ID数组，用于打包输出
// uint32_t range_time;                                    //测距产生时间，串口打包发送
// uint8_t frame_seq_nb = 0;                               //每帧数据增加1
// uint8_t range_nb = 0;                                   //每次range增加1(poll resp1~4 fianl维护一套range_nb)
// uint8_t recv_tag_id;                                    //如当前角色是基站，则表示当前基站收到标签发送过来数据的标签ID
// uint8_t recv_anc_id;                                    //如当前角色是标签，则表示当前标签收到基站发送过来数据的基站ID
// uint8_t range_status = RANGE_NULL;                      //测距成功标志位，用于打包输出
// float rx_power;                                         //接收RSSI
// uint16_t inst_slot_number;                              //系统内最大标签容量
// uint8_t inst_dataRate;                                  //通信速率，用于根据当前110K还是6.8M确定数据超时等通信过程相关参数
// uint8_t inst_ch;                                        //信道号Channel number
// uint8_t inst_prf;                                       //PRF
// uint8_t inst_one_slot_time;                             //一个slot的时间，根据通信速率不同而不同，单位ms
// uint32_t inst_final_rx_timeout;                         =//基站final接收超时时间，根据通信速率不同而不同，单位us
// uint32_t inst_resp_rx_timeout;                          //标签发送poll后接收resp超时时间，根据通信速率不同而不同，单位us
// // uint32_t inst_init_rx_timeout;                          //标签发送blink后接收init超时时间，根据通信速率不同而不同，单位us
// uint64_t inst_poll2final_time;                          //单TWR周期poll起始到final结束的总时间
// uint32_t inst_data_interval;                            //相邻两条数据的间隔，如poll和第一个resp的间隔，resp1和resp2的间隔，根据通信速率不同而不同，单位us
uint16 ant_dly = ANT_DLY;                                   //天线延时
uint32 tx_power = TX_POWER;;                                        //发射增益代码
// uint8_t UART_RX_BUF[200];                               //串口接收BUF
// uint32_t uart_rx_len;                                   //串口接收数据长度
// // vec3d anchorArray[8];                                   //基站坐标，用于标签解算自身位置
// double distance_now_m;                                  //基站计算本周期测距结果，单位米
// int32 distance_offset_cm;                               //距离校准，单位cm
// uint8_t sos = 0;
// uint8_t alarm = 0;
// int user_data[10];
uint32_t distance_flag = 0;                                                             //判断蜂鸣器和灯标志位


int instance_mode = ANCHOR;
int instance_anchaddr = 0;

/* 計算接收功率 */
static double  RX_level = 0;    // 接收功率
static int  RX_level_C = 0;	    //0x12 CIR_PWR 接收功率参数
static int  RX_level_N = 0;     //0x10 RXPACC  接收功率参数
static float	RX_level_A=0;   //
uint32_t D17F = 0;

/* dw1000 rf 配置  */
static instanceConfig_t uwb_config[CONFIG_BR_NUM] = {
    {
        .channelNumber = 2, .pulseRepFreq = DWT_PRF_64M, .preambleLen = DWT_PLEN_1024, .pacSize = DWT_PAC32, 
        .preambleCode = 9, .nsSFD = 1, .dataRate = DWT_BR_110K, .phrMode = DWT_PHRMODE_STD, .sfdTO = (1025 + DW_NS_SFD_LEN_110K - 32) 
    }, // uwb_config0
    {
        .channelNumber = 5, .pulseRepFreq = DWT_PRF_16M, .preambleLen = DWT_PLEN_128, .pacSize = DWT_PAC8, 
        .preambleCode = 3, .nsSFD = 0, .dataRate = DWT_BR_6M8, .phrMode = DWT_PHRMODE_STD, .sfdTO = (129 + DW_NS_SFD_LEN_6M8 - 8)
    }, // uwb_config1
    {
        .channelNumber = 5, .pulseRepFreq = DWT_PRF_64M, .preambleLen = DWT_PLEN_128, .pacSize = DWT_PAC8, 
        .preambleCode = 10, .nsSFD = 1, .dataRate = DWT_BR_6M8, .phrMode = DWT_PHRMODE_STD, .sfdTO = (129 + DW_NS_SFD_LEN_6M8 - 8)
    }, // uwb_config2
    {
        .channelNumber = 2, .pulseRepFreq = DWT_PRF_64M, .preambleLen = DWT_PLEN_256, .pacSize = DWT_PAC16, 
        .preambleCode = 9, .nsSFD = 1, .dataRate = DWT_BR_850K, .phrMode = DWT_PHRMODE_STD, .sfdTO = (257 + DW_NS_SFD_LEN_850K - 16)
    }, // uwb_config3 
    {
        .channelNumber = 5, .pulseRepFreq = DWT_PRF_64M, .preambleLen = DWT_PLEN_256, .pacSize = DWT_PAC16, 
        .preambleCode = 10, .nsSFD = 1, .dataRate = DWT_BR_850K, .phrMode = DWT_PHRMODE_STD, .sfdTO = (257 + DW_NS_SFD_LEN_850K - 16) 
    }, // uwb_config4 当前使用的配置，channel 5，baudrate 850K
};

/* super frame配置 */
sfConfig_t sf_config[2] = {
    {       // 针对uwb_config5设置
        .slotDuration_ms = 12, //slot duration in milliseconds (NOTE: the ranging exchange must be able to complete in this time
        //e.g. tag sends a poll, 4 anchors send responses and tag sends the final + processing time
        .numSlots = (MAX_TAG_LIST_SIZE + 2), //number of slots in the superframe (40 tag slots and 2 used for anchor to anchor ranging)
        .sfPeriod_ms = 12 * 42, // in ms => 378ms
        .tagPeriod_ms = 12 * 42, //tag period in ms (sleep time + ranging time)
        .pollTxToFinalTxDly_us = 2500 //poll to final delay in microseconds (needs to be adjusted according to lengths of ranging frames)
    },
    {       // 针对uwb_config3设置
        .slotDuration_ms = 10, //slot duration in milliseconds (NOTE: the ranging exchange must be able to complete in this time
        //e.g. tag sends a poll, 4 anchors send responses and tag sends the final + processing time
        .numSlots = (MAX_TAG_LIST_SIZE + 2), //number of slots in the superframe (40 tag slots and 2 used for anchor to anchor ranging)
        .sfPeriod_ms = 12 * 42, // in ms => 378ms
        .tagPeriod_ms = 12 * 42, //tag period in ms (sleep time + ranging time)
        .pollTxToFinalTxDly_us = 2500 //poll to final delay in microseconds (needs to be adjusted according to lengths of ranging frames)
    }
};

/* dw1000 发送配置 */
tx_struct txSpeConfig[1] = {
    0xC2,
    {
        TX_POWER,
        TX_POWER
    }
};

/*******************************************************函数声明********************************************************/

uint32 init_uwbApplication(void)
{
    int result;

    reset_DW1000();               /* Target specific drive of RSTn line into DW1000 low for a period. */
    port_set_dw1000_slowrate();
    if(DWT_DEVICE_ID != dwt_readdevid())    // 若读取ID失败，先执行唤醒
    {
        port_wakeup_IC();                   // 使用SPI-NS管脚唤醒DW1000
        if(DWT_DEVICE_ID != dwt_readdevid())
        {
            return (-1);
        }
        dwt_softreset();                    // 软件复位
    }
    reset_DW1000();                         // 复位

    instance_mode = TAG;                    //当前角色控制为标签

    result = instance_init(instance_mode);
    if(0 > result) return (-1);

    port_set_dw1000_fastrate();

    // inst_slot_number = MAX_TAG_NUMBER;
    // dwt_configure(&uwb_config[4]);
    // inst_dataRate = uwb_config[4].dataRate;
    // inst_ch       = uwb_config[4].chan;

    if(uwb_config[4].pulseRepFreq == DWT_PRF_64M) 
    { 
        RX_level_A = 121.74;
    }
    else if (uwb_config[4].pulseRepFreq == DWT_PRF_16M)
    {
        RX_level_A = 113.77;
    }
    D17F = pow(2, 17);
    
    /* 配置设备ID, 在instance.h中修改 */
    dev_id = DEVICE_ID;
    //按角色初始化设备短地址，设置状态机初始状态
    if(instance_mode == ANCHOR)
    {
        /* 设备短地址为2个字节，为了区分A0和T0短地址，基站的最高位为1
         * 如A1短地址=0x8001，T1短地址=0x0001
         */
        uint16_t instance_anchaddr = 0x8000 | dev_id;
        instance_set_16bit_address(instance_anchaddr);
        dwt_setaddress16(instance_anchaddr);
        dwt_forcetrxoff();
    }
    else
    {
        instance_set_16bit_address(instance_anchaddr);
        dwt_setaddress16(dev_id);
        tag_id = dev_id;
        dwt_forcetrxoff();
    }

    instance_config(&uwb_config[0], &sf_config[1]);

    return dev_id;
}

int dw_main(void)
{
    int rx = 0;

    if(init_uwbApplication() == (uint32_t)-1)
    {
        return 0;
    }
    print_config();                                 //打印系统参数信息
    port_EnableEXT_IRQ();

    while(1)                                        //测距功能实现，按角色执行基站状态机或标签状态机
    {
        int n = 0;
        instance_data_t* inst = instance_get_local_structure_ptr(0);
        int monitor_local = inst->monitor ;
        uint32_t txdiff = (portGetTickCnt() - inst->timeofTx);

        instance_mode = instance_get_role();
        if(instance_mode == TAG)
        {
            tag_run();
        }
        else
        {
            anch_run();
        }

        //if delayed TX scheduled but did not happen after expected time then it has failed... (has to be < slot period)
        //if anchor just go into RX and wait for next message from tags/anchors
        //if tag handle as a timeout
        if((monitor_local == 1) && ( txdiff > inst->slotDuration_ms))
        {
            int an = 0;
            uint32 tdly ;
            uint32 reg1, reg2;

            reg1 = dwt_read32bitoffsetreg(0x0f, 0x1);
            reg2 = dwt_read32bitoffsetreg(0x019, 0x1);
            tdly = dwt_read32bitoffsetreg(0x0a, 0x1);
            printf_use_dma("T%08x %08x time %08x %08x", (unsigned int) reg2, (unsigned int) reg1,
                    (unsigned int) dwt_read32bitoffsetreg(0x06, 0x1), (unsigned int) tdly);

            inst->wait4ack = 0;

            if(instance_mode == TAG)
            {
                tag_process_rx_timeout(inst);
            }
            else //if(instance_mode == ANCHOR)
            {
                dwt_forcetrxoff();	//this will clear all events
                inst->testAppState = TA_RXE_WAIT ;
            }
            inst->monitor = 0;  
        }

        rx = instance_newrange();
        if(rx != TOF_REPORT_NUL)
        {
            printf_use_dma("range .\r\n");              // 打印调试用
        }
    }
    return 0;
}

double calculate_RSSI(dwt_rxdiag_t* rx_diag)
{
    dwt_readdiagnostics(rx_diag);
    RX_level_C = (int)rx_diag->maxGrowthCIR;
    RX_level_N = (int)rx_diag->rxPreamCount;

    /* 計算接收功率 */
    RX_level = RX_level_C * D17F;
    RX_level = RX_level / (RX_level_N * RX_level_N);
    RX_level = 10 * log10(RX_level);
    RX_level = RX_level - RX_level_A;

    return RX_level;
}



void print_config(void)
{
    int len;
    uint8_t UART_TX_DATA[512];
    len = sprintf((char*)&UART_TX_DATA[0], "\r\n***************************************************\r\n");
    HAL_UART_Transmit(&huart1, &UART_TX_DATA[0], len, 1000);

    len = sprintf((char*)&UART_TX_DATA[0], "* firmware = %s\r\n* role = %s\r\n* addr = %x\r\n", SOFTWARE_VER, (instance_mode == TAG)?"TAG":"AHCHOR", dev_id);
    HAL_UART_Transmit(&huart1, &UART_TX_DATA[0], len, 1000);

    // len = sprintf((char*)&UART_TX_DATA[0], "* max_anc_num = %d\r\n* max_tag_num = %d\r\n* sync = 0\r\n", MAX_AHCHOR_NUMBER, inst_slot_number);
    // HAL_UART_Transmit(&huart1, &UART_TX_DATA[0], len, 1000);

    // len = sprintf((char*)&UART_TX_DATA[0], "* baud_rate = %s\r\n* channel = CH%d\r\n", (inst_dataRate == DWT_BR_110K)? "110K" : "6.8M", inst_ch);
    // HAL_UART_Transmit(&huart1, &UART_TX_DATA[0], len, 1000);

    // len = sprintf((char*)&UART_TX_DATA[0], "* data_rate = %dHz\r\n* update_time = %dms\r\n* kalmanfilter = %d\r\n", 1000 / (inst_slot_number * inst_one_slot_time), inst_slot_number * inst_one_slot_time, (switch8 & SWS1_KAM_MODE)? 1:0);
    // HAL_UART_Transmit(&huart1, &UART_TX_DATA[0], len, 1000);

    len = sprintf((char*)&UART_TX_DATA[0], "* ant_dly  = %d\r\n* tx_power = %08lx\r\n", ant_dly, tx_power);
    HAL_UART_Transmit(&huart1, &UART_TX_DATA[0], len, 1000);

    len = sprintf((char*)&UART_TX_DATA[0], "***************************************************\r\n");
    HAL_UART_Transmit(&huart1, &UART_TX_DATA[0], len, 1000);
}
