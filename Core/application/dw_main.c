#include "dw_instance.h"
#include "log.h"                                        //SEGGER RTT 调试日志（USART1 留给上位机协议）

uint8_t switch8 = 0;                                    //拨码开关键值
uint8_t instance_mode = ANCHOR;                         //设备运行角色
uint8_t dev_id;                                         //设备ID
uint8_t group_id;                                       //组ID
uint8_t anc_id;                                         //如当前角色是基站，则表示当前基站ID
uint8_t tag_id;                                         //如当前角色是标签，则表示当前标签ID
uint8_t state = STA_IDLE;                               //状态机状态控制
int32_t distance_report[8];                             //基站测距值数组，用于打包输出
int32_t group_report[8];                                //基站组ID数组，用于打包输出
uint32_t range_time;                                    //测距产生时间，串口打包发送
uint8_t frame_seq_nb = 0;                               //每帧数据增加1
uint8_t range_nb = 0;                                   //每次range增加1(poll resp1~4 fianl维护一套range_nb)
uint8_t recv_tag_id;                                    //如当前角色是基站，则表示当前基站收到标签发送过来数据的标签ID
uint8_t recv_anc_id;                                    //如当前角色是标签，则表示当前标签收到基站发送过来数据的基站ID
uint8_t range_status = RANGE_NULL;                      //测距成功标志位，用于打包输出
float rx_power;                                         //接收RSSI
// static char lcd_data[10];                               //OLED显示数据
uint16_t inst_slot_number;                              //系统内最大标签容量
uint8_t inst_dataRate;                                  //通信速率，用于根据当前110K还是6.8M确定数据超时等通信过程相关参数
uint8_t inst_ch;                                        //信道号Channel number
uint8_t inst_prf;                                       //PRF
uint8_t inst_one_slot_time;                             //一个slot的时间，根据通信速率不同而不同，单位ms（超帧配置输入）
twrTimings_t twr_timings;                               //TWR 统一时序参数，由 twr_set_replydelay() 在 init 时装填一次
sfConfig_t   sfConfig;                                  //超帧配置，init 时按 inst_one_slot_time / inst_slot_number 填充
uint16 ant_dly = ANT_DLY;                               //天线延时
uint32 tx_power;                                        //发射增益代码
uint8_t UART_RX_BUF[200];                               //串口接收BUF
uint32_t uart_rx_len;                                   //串口接收数据长度
// vec3d anchorArray[8];                                //基站坐标，用于标签解算自身位置
double distance_now_m;                                  //基站计算本周期测距结果，单位米
int32 distance_offset_cm;                               //距离校准，单位cm
uint8_t sos = 0;
uint8_t alarm = 0;
int user_data[10];
uint32_t distance_flag = 0;								//判断蜂鸣器和灯标志位
int lost_flag = 0;										//失联标志位
uint32_t last_range_ok_tick = 0;						//最后一次成功测距的时间戳
/* 没有板载EEPROM */
uint8_t USE_EEPROM = 0;  

#define TAG_ID 0x0F
/*******************************************************SPI DMA 读写完成标志********************************************************/
volatile uint8_t dw1000_spiDmaCpltFlag = 0;
volatile uint8_t dw1000_spiDmaBusyFlag = 0;

/*******************************************************函数声明********************************************************/
void parse_uart(uint8_t* data);
void read_anc_coord(void);
void print_config(void);

#if defined(USE_DW1000)

static dwt_txconfig_t txconfig_options = {
    .PGdly = 0XC2,            /* PG delay */
    .power = TX_POWER         /* TX power */
};

/* dw1000 rf 配置  */
static dwt_config_t uwb_config_channel5[7] = {
    {   /* uwb_config0，channel5 脉冲频率64M 前导码长度256 数据率 850K，该配置还算稳定，先前一直长期使用 */
        .chan = 5,
        .prf = DWT_PRF_64M,
        .txPreambLength = DWT_PLEN_256,
        .rxPAC = DWT_PAC16,
        .txCode = 10,
        .rxCode = 10,
        .nsSFD = 1,
        .dataRate = DWT_BR_850K,
        .phrMode = DWT_PHRMODE_STD,
        .sfdTO = (257 + DW_NS_SFD_LEN_850K - 16)
    }, 
    {    /* uwb_config1，channel5 脉冲频率64M 前导码长度512 数据率 850K */
        .chan = 5,
        .prf = DWT_PRF_64M,
        .txPreambLength = DWT_PLEN_512,
        .rxPAC = DWT_PAC16,
        .txCode = 10,
        .rxCode = 10,
        .nsSFD = 1,
        .dataRate = DWT_BR_850K,
        .phrMode = DWT_PHRMODE_STD,
        .sfdTO = (513 + DW_NS_SFD_LEN_850K - 16)
    },
    {   /* uwb_config2，channel5 脉冲频率64M 前导码长度1024 数据率 850K */
        .chan = 5,
        .prf = DWT_PRF_64M,
        .txPreambLength = DWT_PLEN_1024,
        .rxPAC = DWT_PAC32,
        .txCode = 10,
        .rxCode = 10,
        .nsSFD = 1,
        .dataRate = DWT_BR_850K,
        .phrMode = DWT_PHRMODE_STD,
        .sfdTO = (1025 + DW_NS_SFD_LEN_850K - 32)
    },
    {    /* uwb_config3，channel5 脉冲频率64M 前导码长度1024 数据率 850K PAC_Size 64 */
        .chan = 5,
        .prf = DWT_PRF_64M,
        .txPreambLength = DWT_PLEN_1024,
        .rxPAC = DWT_PAC64,
        .txCode = 10,
        .rxCode = 10,
        .nsSFD = 1,
        .dataRate = DWT_BR_850K,
        .phrMode = DWT_PHRMODE_STD,
        .sfdTO = (1025 + DW_NS_SFD_LEN_850K - 64)
    }, 
    {    /* uwb_config4，channel5 脉冲频率64M 前导码长度256 数据率 6M8 */
        .chan = 5,
        .prf = DWT_PRF_64M,
        .txPreambLength = DWT_PLEN_256,
        .rxPAC = DWT_PAC16,
        .txCode = 10,
        .rxCode = 10,
        .nsSFD = 1,
        .dataRate = DWT_BR_6M8,
        .phrMode = DWT_PHRMODE_STD,
        .sfdTO = (257 + DW_NS_SFD_LEN_6M8 - 16)
    }, 
    {    /* uwb_config5，channel5 脉冲频率64M 前导码长度1024 数据率 110K */
        .chan = 5,
        .prf = DWT_PRF_64M,
        .txPreambLength = DWT_PLEN_1024,
        .rxPAC = DWT_PAC32,
        .txCode = 10,
        .rxCode = 10,
        .nsSFD = 1,
        .dataRate = DWT_BR_110K,
        .phrMode = DWT_PHRMODE_STD,
        .sfdTO = (1025 + DW_NS_SFD_LEN_110K - 32)
    }, 
    {    /* uwb_config6，channel5 脉冲频率64M 前导码长度2048 数据率 110K， 测试近距离都丢包严重 */
        .chan = 5,
        .prf = DWT_PRF_64M,
        .txPreambLength = DWT_PLEN_2048,
        .rxPAC = DWT_PAC64,
        .txCode = 10,
        .rxCode = 10,
        .nsSFD = 1,
        .dataRate = DWT_BR_110K,
        .phrMode = DWT_PHRMODE_STD,
        .sfdTO = (2049 + DW_NS_SFD_LEN_110K - 64)
    }, 
};
#elif defined(USE_DW3000)
/* dw3000 rf 配置 channel5（6.5GHz，与 DW1000 ch5 设备互通）
 * DW3000 不支持 110K 速率；PRF 由 txCode/rxCode 隐含（code 9~24 = 64MHz）
 * sfdType: 0=IEEE 8bit, 1=DW 8bit, 2=DW 16bit, 3=4z BPRF
 */
static dwt_config_t uwb_config_channel5[1] = {
    {   /* uwb_config0，channel5 PRF64M 前导码长度1024 数据率 850K，主要使用 */
        .chan = 5,
        .txPreambLength = DWT_PLEN_1024,
        .rxPAC = DWT_PAC32,
        .txCode = 10,
        .rxCode = 10,
        .sfdType = 2,                   /* DW 16-bit SFD，与 DW1000 nsSFD 配置对齐 */
        .dataRate = DWT_BR_850K,
        .phrMode = DWT_PHRMODE_STD,
        .phrRate = DWT_PHRRATE_STD,
        .sfdTO = (1025 + DWT_SFD_LEN16 - 32),
        .stsMode = DWT_STS_MODE_OFF,
        .stsLength = DWT_STS_LEN_64,
        .pdoaMode = DWT_PDOA_M0
    },
};

static dwt_txconfig_t txconfig_options = {
    .PGdly = 0xC2,           /* PG delay */
    .power = TX_POWER,       /* TX power */
    .PGcount = 0
};
#endif

/******************************************** TWR 时序计算（TREK1000 同源）********************************************
 * 自 ../Anchor_RTOS/01_Core/App/Src/dw_main.c 逐字移植，双端必须同公式同参数，否则空口槽位对不上。
 * 字段语义见 dw_instance.h twrTimings_t，双端约定与参考数值见 ../Anchor_RTOS/docs/TWR_TIMING.md。
 *********************************************************************************************************************/
static float calc_length_data(float msgdatalen)
{
    int x = 0;

    /*
        根据802.15.4 UWB PHY 规定,PSDU 数据要经过 RS(63,55) 编码:
        每 330 bits 数据为一块, 每块附加 48 bits 校验
    */
    x = ((int)msgdatalen * 8 + 329) / 330;      // 不足330bits 的尾块也要按一整块加 48 bits，整数向上取整（避免 libm 的 ceil）
    msgdatalen = msgdatalen * 8.0f + x * 48.0f; // 计算总的编码后数据长度，单位bits，每个330bits数据块加上48bits校验码

    // Assume PHR length is 172308ns for 110k and 21539ns for 850k/6.8M.
#if defined(USE_DW1000)
    if (inst_dataRate == DWT_BR_110K)
    {
        msgdatalen *= 8205.13f;
        msgdatalen += 172308.0f;
    }
    else
#endif
    if (inst_dataRate == DWT_BR_850K)
    {
        msgdatalen *= 1025.64f; // 以850K为例，计算出来的总的bit数，乘上每个bit占用的时间
        msgdatalen += 21539.0f; // 加上PHR的占用时间
    }
    else
    {
        msgdatalen *= 128.21f;
        msgdatalen += 21539.0f;
    }

    return msgdatalen;         // 返回计算完成的的air time，单位ns
}

/* 前导码长度（symbol 数），从 RF 配置枚举反查。
 * 仅列出两种芯片 SDK 都有的档位，本项目实际只用 256/1024 */
static uint16_t plen_symbols(const dwt_config_t *rf)
{
    switch (rf->txPreambLength)
    {
    case DWT_PLEN_64:   return 64;
    case DWT_PLEN_128:  return 128;
    case DWT_PLEN_256:  return 256;
    case DWT_PLEN_512:  return 512;
    case DWT_PLEN_1024: return 1024;
    default:            return 1024;
    }
}

/* DW 非标 SFD 长度按速率（TREK dwnsSFDlen[] 同值）：110K=64, 850K=16, 6M8=8 */
static uint8_t sfd_length(void)
{
#if defined(USE_DW1000)
    if (inst_dataRate == DWT_BR_110K)
    {
        return 64;
    }
#endif
    return (inst_dataRate == DWT_BR_850K) ? 16 : 8;
}

/* us → DW 设备时间单位（40bit，~15.65ps/tick），TREK instance_convert_usec_to_devtimeu 移植 */
static uint64_t conv_us_to_devtime(double microsecu)
{
    return (uint64_t)((microsecu / (double)DWT_TIME_UNITS) / 1e6);
}

/* 统一计算 TWR 时序，TREK1000 instance_set_replydelay()（instance_common.c）移植，
 * dwt_configure 之后调用一次。全部时序（含 final 时刻）由帧长公式导出，无按速率展开的时序宏。
 * 调用前须先确定 inst_dataRate / inst_one_slot_time / inst_slot_number。 */
static void twr_set_replydelay(const dwt_config_t *rf)
{
    twrTimings_t *t = &twr_timings;
    int margin = 3000;      // ns，接收超时里的帧长余量（TREK 同值）

    /* 帧长 air time(ns)：MSG_LEN 为 MHR+载荷，FCS 也在空口飞，须计入（TREK 含 FRAME_CRC）。
     * 槽间隔只由 resp 帧长决定，故标签的 POLL_MSG_LEN 与基站不同不影响双端对齐 */
    float msgdatalen_resp  = calc_length_data(RESP_MSG_LEN  + FCS_LEN);
    float msgdatalen_final = calc_length_data(FIANL_MSG_LEN + FCS_LEN);

    /* 前导码时长(us)：PRF64 符号 1.01763us，PRF16 用 0.99359us */
#if defined(USE_DW1000)
    float sym_us = (rf->prf == DWT_PRF_16M) ? 0.99359f : 1.01763f;
#else
    float sym_us = 1.01763f;    // DW3000 本工程固定 PRF64（txCode 9~24 隐含）
#endif
    float preamble_us = (plen_symbols(rf) + sfd_length()) * sym_us;

    /* resp 整帧时长(us)与统一槽间隔(us) */
    float respframe_us   = preamble_us + msgdatalen_resp / 1000.0f;
    float replyDelay_us  = respframe_us + RX_RESPONSE_TURNAROUND; // 一个 resp 帧占用信道的时间 + 双方处理该帧并执行下一步动作的时间

    /* 接收超时(symbol)：RX 开机延时 + 前导码 + 数据段 + 余量 */
    int respframe_sy  = DW_RX_ON_DELAY + (int)((preamble_us + (msgdatalen_resp  + margin) / 1000.0f) / 1.0256f);
    int finalframe_sy = DW_RX_ON_DELAY + (int)((preamble_us + (msgdatalen_final + margin) / 1000.0f) / 1.0256f);

    t->fixedReplyDelayAnc32h  = (uint32_t)(conv_us_to_devtime(replyDelay_us) >> 8);
    t->fixedReplyDelay_sy     = (uint16_t)(replyDelay_us / 1.0256f); // 转换成symbol
    t->preambleDuration32h    = (uint32_t)(conv_us_to_devtime(preamble_us) >> 8) + DW_RX_ON_DELAY;

    /* 标签 final 时刻 = poll TX + (N+1)×槽间隔（基站按同公式开 final 接收窗）。
     * 不能取 N×：最后一个 resp 槽的数据段在其 RMARKER 之后还要飞，而 final 的前导码
     * 在 final RMARKER 之前就开始飞，(N+1)× 天然留出一个槽的间隔避免空口重叠 */
    t->pollTx2FinalTxDelay32h = (MAX_AHCHOR_NUMBER + 1) * t->fixedReplyDelayAnc32h;
    t->fwto4RespFrame_sy      = (uint16_t)respframe_sy;              // fwto: frame wait timeout
    t->fwto4FinalFrame_sy     = (uint16_t)(finalframe_sy + 200);     // 加余量防过早超时（TREK 同值）

    uint32_t finalDelay_us = (uint32_t)((MAX_AHCHOR_NUMBER + 1) * replyDelay_us);
    sfConfig.pollTxToFinalTxDly_us = (uint16)finalDelay_us;          // 由公式导出，供打印/校核

    LOG_I("TWR timing: replyDelay=%uus(32h=%lu) preamble=%uus fwtoResp=%usy fwtoFinal=%usy pollTx2Final=%uus",
          (unsigned)replyDelay_us,
          (unsigned long)t->fixedReplyDelayAnc32h,
          (unsigned)preamble_us,
          (unsigned)t->fwto4RespFrame_sy,
          (unsigned)t->fwto4FinalFrame_sy,
          (unsigned)finalDelay_us);

    /* 单次 T2A 交换必须放得进一个 slot：poll 前导 + pollTx→finalTx 总延时 + final 数据段。
     * （标签不参与 A2A，故不做基站那条 A2A 检查） */
    uint32_t t2a_us  = (uint32_t)(preamble_us + finalDelay_us + msgdatalen_final / 1000.0f);
    uint32_t slot_us = (uint32_t)inst_one_slot_time * 1000U;
    if (t2a_us > slot_us)
    {
        LOG_E("TWR exchange exceeds slot %ums! T2A=%uus", inst_one_slot_time, (unsigned)t2a_us);
    }
}

/* 超帧配置装填：须在 inst_one_slot_time / inst_slot_number 确定之后调用 */
static void sf_config_init(void)
{
    sfConfig.slotDuration_ms = inst_one_slot_time;                      // 850K 下 12ms
    sfConfig.numSlots        = inst_slot_number;                        // MAX_TAG_NUMBER = 50
    sfConfig.sfPeriod_ms     = inst_one_slot_time * inst_slot_number;   // 600ms，与基站一致
    sfConfig.tagPeriod_ms    = sfConfig.sfPeriod_ms;                    // 标签测距周期 = 超帧周期
    /* pollTxToFinalTxDly_us 由 twr_set_replydelay() 填 */
}

#if defined(USE_DW1000)
static void dw1000_init(void)
{
    dwt_config_t *current_rfConfig = &uwb_config_channel5[2];

    reset_DW1000();               /* Target specific drive of RSTn line into DW1000 low for a period. */
    port_set_dw1000_slowrate();
    if(DWT_DEVICE_ID != dwt_readdevid())    // 若读取ID失败，先执行唤醒
    {
        dev_id = dwt_readdevid();
        port_wakeup_IC();                   // 使用SPI-NS管脚唤醒DW1000
        dwt_softreset();                    // 软件复位
    }
    reset_DW1000();                         // 复位

    if (dwt_initialise(DWT_LOADUCODE) == DWT_ERROR) // dw1000初始化失败
    {
        while (1);
    }
    port_set_dw1000_fastrate();

    inst_slot_number = MAX_TAG_NUMBER;
    
    dwt_configure(current_rfConfig);
    inst_dataRate = current_rfConfig->dataRate;
    inst_ch       = current_rfConfig->chan;

    /* 超帧 slot 时长按速率选择；其余 TWR 时序统一由 twr_set_replydelay() 按帧长公式算出 */
    if(inst_dataRate == DWT_BR_6M8)
    {
        inst_one_slot_time = ONE_SLOT_TIME_MS_6P8M;
    }
    else if(inst_dataRate == DWT_BR_110K)
    {
        inst_one_slot_time = ONE_SLOT_TIME_MS_110K;
    }
    else
    {
        inst_one_slot_time = ONE_SLOT_TIME_MS_850K;
    }
    sf_config_init();
    twr_set_replydelay(current_rfConfig);


#if (USE_EEPROM == 1) //板载EEPROM
    {
        uint8_t tx_pwr_read[EEP_UNIT_SIZE]={0};
        E2prom_Read(TX_PWR_ADDR, tx_pwr_read, EEP_UNIT_SIZE);
        if(tx_pwr_read[0] == 0xAA) //固定AA
        {
            txconfig_options.power  = (uint32)tx_pwr_read[1] << 24;
            txconfig_options.power += (uint32)tx_pwr_read[2] << 16;
            txconfig_options.power += (uint32)tx_pwr_read[3] << 8;
            txconfig_options.power += (uint32)tx_pwr_read[4];
        }
        else    //未配置，写入默认值
        {
            txconfig_options.power = TX_POWER;
            uint8_t tx_pwr_write[EEP_UNIT_SIZE]={0};
            tx_pwr_write[0] = 0xAA;
            tx_pwr_write[1] = txconfig_options.power >> 24;
            tx_pwr_write[2] = txconfig_options.power >> 16;
            tx_pwr_write[3] = txconfig_options.power >> 8;
            tx_pwr_write[4] = txconfig_options.power;
            E2prom_Write(TX_PWR_ADDR, tx_pwr_write, EEP_UNIT_SIZE);
        }
    }
    else    //未板载EEPROM，按define值设置
#endif
    {
        txconfig_options.power = TX_POWER;
    }
    tx_power = txconfig_options.power;
    dwt_configuretxrf(&txconfig_options);//设置发射功率和pg值

#if (USE_EEPROM == 1) //板载EEPROM
    {
        uint8_t ant_dly_read[EEP_UNIT_SIZE] = {0};
        E2prom_Read(ANT_DLY_ADDR, ant_dly_read, EEP_UNIT_SIZE);
        if(ant_dly_read[0] == 0xAA) //固定AA
        {
            ant_dly = (uint16)ant_dly_read[1] << 8 | (uint16)ant_dly_read[2];
        }
        else    //未配置，写入默认值
        {
            ant_dly = ANT_DLY;
            uint8_t ant_dly_write[EEP_UNIT_SIZE] = {0};
            ant_dly_write[0] = 0xAA;
            ant_dly_write[1] = ant_dly >> 8;
            ant_dly_write[2] = (uint8)ant_dly;
            E2prom_Write(ANT_DLY_ADDR, ant_dly_write, EEP_UNIT_SIZE);
        }
    }
    else    //未板载EEPROM，按define值设置
#endif
    {
        ant_dly = ANT_DLY;
    }
    dwt_setrxantennadelay(ant_dly);                 //设置天线延时
    dwt_settxantennadelay(ant_dly);
//    dwt_setrxtimeout(inst_resp_rx_timeout);         //设置接收超时时间
//    dwt_setpreambledetecttimeout(0);                //设置前导码超时

    dwt_setpanid(PAN_ID);                           //设置PAN ID 组号
    dwt_enableframefilter(DWT_FF_DATA_EN | DWT_FF_ACK_EN);  //设置帧过滤模式开启

    dwt_setlnapamode(1, 1);//设置外置PA和LNA控制开启
    dwt_setleds(DWT_LEDS_ENABLE | DWT_LEDS_INIT_BLINK);//设置DW3000控制的收发指示灯开启，低功耗时可注释掉
    
    /* 配置角色 */
    instance_mode = TAG; //当前角色控制为标签
    
#if (USE_EEPROM == 1) //板载EEPROM
    {
        uint8_t dev_id_read[EEP_UNIT_SIZE] = {0};
        E2prom_Read(DEV_ID_ADDR, dev_id_read, EEP_UNIT_SIZE);
        if(dev_id_read[0] == 0xAA) //固定AA
        {
            dev_id = dev_id_read[1];
        }
        else
        {
            //读取拨码开关设备ID
            dev_id = ((switch8 & SWS1_A1A_MODE) + (switch8 & SWS1_A2A_MODE) + (switch8 & SWS1_A3A_MODE)) >> 1;
        }
    }
    else
#endif
    {
        /* 配置设备ID */
        dev_id = TAG_ID;
    }

    //设置中断标志
    dwt_setinterrupt(DWT_INT_TFRS | DWT_INT_RFCG | (DWT_INT_ARFE | DWT_INT_RFSL | DWT_INT_SFDT | DWT_INT_RPHE | DWT_INT_RFCE | DWT_INT_RFTO | DWT_INT_RXPTO), 1);

    if(instance_mode == ANCHOR)
    {
        //设置基站的中断回调函数
        // dwt_setcallbacks(&anc_tx_conf_cb, &anc_rx_ok_cb, &anc_rx_to_cb, &anc_rx_err_cb);
    }
    else 
    {   //设置标签的中断回调函数
        dwt_setcallbacks(&tag_tx_conf_cb, &tag_rx_ok_cb, &tag_rx_to_cb, &tag_rx_err_cb);
    }

    //按角色初始化设备短地址，设置状态机初始状态
    if(instance_mode == ANCHOR)
    {
        /* 设备短地址为2个字节，为了区分A0和T0短地址，基站的最高位为1
         * 如A1短地址=0x8001，T1短地址=0x0001
         */
        uint16_t anc_short_add = 0x8000 | dev_id;
        dwt_setaddress16(anc_short_add);
        anc_id = dev_id;

        if(anc_id == 0)             //A0的group ID最高bit设置为1，为时序校准基站
        {
            group_id = group_id | 0x80;
        }
        dwt_forcetrxoff();
        state = STA_INIT_POLL_SYNC;
    }
    else
    {
        dwt_setaddress16(dev_id);
        tag_id = dev_id;
        dwt_forcetrxoff();
        state = STA_IDLE;
    }
    // 因为当前板子引脚分配，dw1000中断脚为PB0，复位引脚为PA0，共用一个中断线，在setup_DW1000RSTnIRQ中会关闭该中断，因此在这里重新打开。
    HAL_NVIC_EnableIRQ(EXTI0_IRQn);
}
#elif defined(USE_DW3000)
static void dw3000_init(void)
{
    dwt_config_t *current_rfConfig = &uwb_config_channel5[0];

    /* 关键：CubeMX 的 MX_GPIO_Init() 已使能 EXTI0(PB0)，且早于本函数执行。
     * DW3000 复位后会立即拉高 IRQ(SPIRDY/RCINIT)，若此时 EXTI0 是活的，会在
     * port_set_dwic_isr() 注册前就进入 process_deca_irq()，因 isr 仍为 NULL 而死循环。
     * 故初始化期间先屏蔽 EXTI0，待 ISR 注册完毕、清掉 pending 后再于末尾重新打开。*/
    port_DisableEXT_IRQ();

    reset_DW3000();                         /* 硬复位 DW3000（平台层提供，待 F4→F1 移植补齐）*/
    port_set_dw_ic_spi_slowrate();
    while (!dwt_checkidlerc())               /* 等待 DW3000 进入 IDLE_RC 状态后再继续 */
    { };
    if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR) // dw3000初始化失败
    {
        while (1);
    }
    port_set_dw_ic_spi_fastrate();

    inst_slot_number = MAX_TAG_NUMBER;

    dwt_configure(current_rfConfig);
    inst_dataRate = current_rfConfig->dataRate;
    inst_ch       = current_rfConfig->chan;

    /* 超帧 slot 时长按速率选择（DW3000 不支持 110K，仅 850K/6.8M）；
     * 其余 TWR 时序统一由 twr_set_replydelay() 按帧长公式算出 */
    if(inst_dataRate == DWT_BR_6M8)
    {
        inst_one_slot_time = ONE_SLOT_TIME_MS_6P8M;
    }
    else
    {
        inst_one_slot_time = ONE_SLOT_TIME_MS_850K;
    }
    sf_config_init();
    twr_set_replydelay(current_rfConfig);

    txconfig_options.power = TX_POWER;
    tx_power = txconfig_options.power;
    dwt_configuretxrf(&txconfig_options);   //设置发射功率和pg值

    ant_dly = ANT_DLY;
    dwt_setrxantennadelay(ant_dly);         //设置天线延时
    dwt_settxantennadelay(ant_dly);

    dwt_setpanid(PAN_ID);                                                   //设置PAN ID 组号
    dwt_configureframefilter(DWT_FF_ENABLE_802_15_4, DWT_FF_DATA_EN | DWT_FF_ACK_EN); //设置帧过滤模式开启
    dwt_setlnapamode(DWT_LNA_ENABLE | DWT_PA_ENABLE);                       //设置外置PA和LNA控制开启
    dwt_setleds(DWT_LEDS_ENABLE | DWT_LEDS_INIT_BLINK);                     //设置收发指示灯开启

    /* 配置角色为标签 */
    instance_mode = TAG;
    dev_id = TAG_ID;

    /* 设置中断标志：必须使能 ARFE（帧过滤拒绝）中断，与 DW1000 路径(DWT_INT_ARFE)对齐，
     * 否则漏收 poll 又收到发给标签的 resp 帧时，接收机会被静默关闭、再也收不到后续 poll。 */
    dwt_setinterrupt(SYS_ENABLE_LO_TXFRS_ENABLE_BIT_MASK | SYS_ENABLE_LO_RXFCG_ENABLE_BIT_MASK | SYS_ENABLE_LO_RXFTO_ENABLE_BIT_MASK |
                     SYS_ENABLE_LO_RXPTO_ENABLE_BIT_MASK | SYS_ENABLE_LO_RXPHE_ENABLE_BIT_MASK | SYS_ENABLE_LO_RXFCE_ENABLE_BIT_MASK |
                     SYS_ENABLE_LO_RXFSL_ENABLE_BIT_MASK | SYS_ENABLE_LO_RXSTO_ENABLE_BIT_MASK |
                     SYS_ENABLE_LO_ARFE_ENABLE_BIT_MASK, 0, DWT_ENABLE_INT);

    //设置标签的中断回调函数（DW3000 为 6 参数：含 SPI 错误/就绪回调，未用传 NULL）
    dwt_setcallbacks(&tag_tx_conf_cb, &tag_rx_ok_cb, &tag_rx_to_cb, &tag_rx_err_cb, NULL, NULL);

    dwt_setaddress16(dev_id);
    tag_id = dev_id;
    dwt_forcetrxoff();
    state = STA_IDLE;

    /* 注册 ISR 指针（屏蔽态下安装，不改使能态）：中断里调用 dwt_isr() 分发到上面注册的回调 */
    port_set_dwic_isr(dwt_isr);

    /* 清掉 init 期间(复位/配置)在 PB0 上锁存的 EXTI 边沿与 NVIC pending，
     * 避免一使能就立刻进一次"陈旧"中断。之后 dwt_isr 能正常读/清 SYS_STATUS。*/
    __HAL_GPIO_EXTI_CLEAR_IT(DW1000_IRQ_Pin);
    HAL_NVIC_ClearPendingIRQ(EXTI0_IRQn);

    /* 唯一的“开”：进入稳态，使能 EXTI0(PB0)。此后 decamutex 在每次 SPI 收发时
     * 依赖该稳态使能来 save/disable/restore，故必须在这里打开。*/
    port_EnableEXT_IRQ();
}
#endif

/* 对外统一入口：main.c 调用 dw_init() 完成 UWB 初始化，内部按编译目标分派到对应芯片(DW1000/DW3000) */
void dw_init(void)
{
#if defined(USE_DW1000)
    dw1000_init();
#elif defined(USE_DW3000)
    dw3000_init();
#endif
}

void dw_main(void)
{   
    last_range_ok_tick = portGetTickCnt();          //初始化，给系统启动预留超时窗口
    while(1)                                        //测距功能实现，按角色执行基站状态机或标签状态机
    {
        // HAL_IWDG_Refresh(&hiwdg);                //喂狗
        tag_app();
        
        if(range_status == RANGE_TWR_OK)            //TWR测距有效，进行数据滤波和打包输出、屏显
        {
            range_status = RANGE_NULL;              //清空标志位
            
            for(int i=0; i<MAX_AHCHOR_NUMBER; i++)
            {
                if(distance_report[i] > 0)          //数据有效，进行卡尔曼滤波计算
                {
                    distance_report[i] = kalman_filter(distance_report[i], i, (instance_mode == TAG) ? tag_id : recv_tag_id);
                }
            }
            /* 打印所有基站距离：按 MAX_AHCHOR_NUMBER 动态生成，两位小数，单位 m。
             * 距离<0 视为无效显示 null。以后改 MAX_AHCHOR_NUMBER 即自动增减，无需再手列 A0~A3。 */
            char dist_line[MAX_AHCHOR_NUMBER * 24 + 16];
            int  pos = sprintf(dist_line, "TAG_ID%d", dev_id);
            for(int i = 0; i < MAX_AHCHOR_NUMBER; i++)
            {
                if(distance_report[i] >= 0)
                {
                    pos += sprintf(dist_line + pos, ", A%d %.2f", i, (float)distance_report[i] / 1000.0);
                }
                else
                {
                    pos += sprintf(dist_line + pos, ", A%d null", i);
                }
            }
            LOG_RAW("%s\r\n", dist_line);
            led_off(RUN_LED2);
            
            if( (float)(distance_report[recv_anc_id] / 1000.0) < MIN_DISTANCE )
            {
                distance_flag = 1;
            }
            else
            {
                distance_flag = 0;
            }

			last_range_ok_tick = portGetTickCnt(); //刷新成功测距时间戳
			lost_flag = 0;

        }
        else if(range_status == RANGE_ERROR)
        {
            range_status = RANGE_NULL;              //清空标志位

            LOG_W("RANGE_ERROR, ID = %d, rb = %d, range_time = %d", dev_id, range_nb, range_time);

            for(uint8_t i = 0; i < 8; i++)  //清空distance_report数组，设置无效值
            {
                distance_report[i] = -1;
                group_report[i] = -1;
            }
        }

        // 基于时间戳判定失联：超过 LOST_TIMEOUT_MS 未成功测距即置失联标志
        if((portGetTickCnt() - last_range_ok_tick) > LOST_TIMEOUT_MS)
        {
            lost_flag = 1;
        }

    }
}

void print_config(void)
{
    LOG_RAW("\r\n***************************************************\r\n");
    LOG_RAW("* role = TAG\r\n");
    LOG_RAW("* firmware = %s\r\n* role = %s\r\n* addr = %d\r\n", SOFTWARE_VER, (instance_mode == TAG)?"TAG":"AHCHOR", dev_id);
    LOG_RAW("* max_anc_num = %d\r\n* max_tag_num = %d\r\n* sync = 0\r\n", MAX_AHCHOR_NUMBER, inst_slot_number);

#if defined(USE_DW1000)
    const char *baud_str = (inst_dataRate == DWT_BR_110K) ? "110K" : "850K";  // DW1000 当前配置为 850K
#else
    const char *baud_str = (inst_dataRate == DWT_BR_6M8)  ? "6.8M" : "850K";  // DW3000 无 110K
#endif
    LOG_RAW("* baud_rate = %s\r\n* channel = CH%d\r\n", baud_str, inst_ch);
    LOG_RAW("* data_rate = %dHz\r\n* update_time = %dms\r\n* kalmanfilter = %d\r\n", 1000 / sfConfig.sfPeriod_ms, sfConfig.sfPeriod_ms, (switch8 & SWS1_KAM_MODE)? 1:0);
    LOG_RAW("* ant_dly  = %d\r\n* tx_power = %08lx\r\n", ant_dly, tx_power);
    LOG_RAW("***************************************************\r\n");
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance == SPI1)
    {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET); // DMA发送完成，SPI片选拉高
        dw1000_spiDmaCpltFlag = 1;
        dw1000_spiDmaBusyFlag = 0;
    }
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance == SPI1)
    {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET); // DMA接收完成，SPI片选拉高
        dw1000_spiDmaCpltFlag = 1;
        dw1000_spiDmaBusyFlag = 0;
    }
}
