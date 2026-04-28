#include "instance.h"

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
uint8_t inst_one_slot_time;                             //一个slot的时间，根据通信速率不同而不同，单位ms
uint32_t inst_final_rx_timeout;                         //基站final接收超时时间，根据通信速率不同而不同，单位us
uint32_t inst_resp_rx_timeout;                          //标签发送poll后接收resp超时时间，根据通信速率不同而不同，单位us
// uint32_t inst_init_rx_timeout;                          //标签发送blink后接收init超时时间，根据通信速率不同而不同，单位us
uint64_t inst_poll2final_time;                          //单TWR周期poll起始到final结束的总时间
uint32_t inst_data_interval;                            //相邻两条数据的间隔，如poll和第一个resp的间隔，resp1和resp2的间隔，根据通信速率不同而不同，单位us
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
/* 没有板载EEPROM */
uint8_t USE_EEPROM = 0;  

#define TAG_ID 0x1D

/*******************************************************SPI DMA 读写完成标志********************************************************/
volatile uint8_t dw1000_spiDmaCpltFlag = 0;
volatile uint8_t dw1000_spiDmaBusyFlag = 0;

/*******************************************************函数声明********************************************************/
void parse_uart(uint8_t* data);
void read_anc_coord(void);
void print_config(void);

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
    {    /* uwb_config3，channel5 脉冲频率64M 前导码长度128 数据率 6M8 */
        .chan = 5,
        .prf = DWT_PRF_64M,
        .txPreambLength = DWT_PLEN_128,
        .rxPAC = DWT_PAC8,
        .txCode = 10,
        .rxCode = 10,
        .nsSFD = 1,
        .dataRate = DWT_BR_6M8,
        .phrMode = DWT_PHRMODE_STD,
        .sfdTO = (129 + DW_NS_SFD_LEN_6M8 - 8)
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

static dwt_txconfig_t txconfig_options = {
    .PGdly = 0XC2,            /* PG delay */
    .power = TX_POWER         /* TX power */
};

void DW1000_init(void)
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

    /* 配置通信相关时序 */
    if(inst_dataRate == DWT_BR_6M8)
    {
        inst_one_slot_time    = ONE_SLOT_TIME_MS_6P8M;
        inst_final_rx_timeout = FINAL_RX_TIMEOUT_6P8M;
        inst_resp_rx_timeout  = RESP_RX_TIMEOUT_6P8M;
        inst_data_interval    = DATA_INTERVAL_TIME_6P8M;
        inst_poll2final_time  = ((FIRST_RESP_SEND_6P8M +  MAX_AHCHOR_NUMBER * inst_data_interval) * UUS_TO_DWT_TIME);
    }
    else if(inst_dataRate == DWT_BR_110K)
    {
        inst_one_slot_time    = ONE_SLOT_TIME_MS_110K;
        inst_final_rx_timeout = FINAL_RX_TIMEOUT_110K;
        inst_resp_rx_timeout  = RESP_RX_TIMEOUT_110K;
        inst_data_interval    = DATA_INTERVAL_TIME_110K;
        inst_poll2final_time  = ((FIRST_RESP_SEND_110K +  MAX_AHCHOR_NUMBER * inst_data_interval) * UUS_TO_DWT_TIME);
    }
    else if(inst_dataRate == DWT_BR_850K) 
    {
        inst_one_slot_time    = ONE_SLOT_TIME_MS_850K;
        inst_final_rx_timeout = FINAL_RX_TIMEOUT_850K;
        inst_resp_rx_timeout  = RESP_RX_TIMEOUT_850K;
        inst_data_interval    = DATA_INTERVAL_TIME_850K;
        inst_poll2final_time  = ((FIRST_RESP_SEND_850K +  MAX_AHCHOR_NUMBER * inst_data_interval) * UUS_TO_DWT_TIME);
    }
    
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

void dw_main(void)
{   
    int lost_work = 0;
    char distance_report_char[4][40];
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
            for(int i = 0; i < MAX_AHCHOR_NUMBER; i++)
            {
                if(distance_report[i] >= 0)
                {
                    sprintf(distance_report_char[i], "%.2f", (float)distance_report[i]/1000.0);
                }
            }
            printf_use_dma( "TAG_ID%d, A0 %s, A1 %s, A2 %s, A3 %s\r\n",
                            dev_id, 
                            (distance_report[0] >= 0) ? distance_report_char[0] : "null", 
                            (distance_report[1] >= 0) ? distance_report_char[1] : "null", 
                            (distance_report[2] >= 0) ? distance_report_char[2] : "null", 
                            (distance_report[3] >= 0) ? distance_report_char[3] : "null" );
            led_off(RUN_LED2);
            
            if( (float)(distance_report[recv_anc_id] / 1000.0) < MIN_DISTANCE )
            {
                distance_flag = 1;
            }
            else
            {
                distance_flag = 0;
            }

			lost_flag = 0; //没有失联
//			right_work++;
//			if (right_work == 68) // 正常工作30s响一次
//			{
//				bee_on();
//				
//			}else{
//				bee_close();
//			}
//			
//			if (right_work == 70) 
//			{
//				right_work = 0;
//			}

        }
        else if(range_status == RANGE_ERROR) 
        {
            range_status = RANGE_NULL;              //清空标志位
            
            printf_use_dma("RANGE_ERROR, ID = %d, rb = %d, range_time = %d\r\n", dev_id, range_nb, range_time);
            
            for(uint8_t i = 0; i < 8; i++)  //清空distance_report数组，设置无效值
            {
               distance_report[i] = -1; 
               group_report[i] = -1; 
            }
			
			lost_work++; //没有测到60次为失联
			if(lost_work == 60)
			{
				lost_flag = 1;
				lost_work = 0;
			}
	
        }
	
    }
}

void print_config(void)
{
    int len;
    uint8_t UART_TX_DATA[512];
    len = sprintf((char*)&UART_TX_DATA[0], "\r\n***************************************************\r\n");
    HAL_UART_Transmit(&huart1, &UART_TX_DATA[0], len, 1000);

    len = sprintf((char*)&UART_TX_DATA[0], "* role = TAG\r\n");
    HAL_UART_Transmit(&huart1, &UART_TX_DATA[0], len, 1000);

    len = sprintf((char*)&UART_TX_DATA[0], "* firmware = %s\r\n* role = %s\r\n* addr = %d\r\n", SOFTWARE_VER, (instance_mode == TAG)?"TAG":"AHCHOR", dev_id);
    HAL_UART_Transmit(&huart1, &UART_TX_DATA[0], len, 1000);

    len = sprintf((char*)&UART_TX_DATA[0], "* max_anc_num = %d\r\n* max_tag_num = %d\r\n* sync = 0\r\n", MAX_AHCHOR_NUMBER, inst_slot_number);
    HAL_UART_Transmit(&huart1, &UART_TX_DATA[0], len, 1000);

    len = sprintf((char*)&UART_TX_DATA[0], "* baud_rate = %s\r\n* channel = CH%d\r\n", (inst_dataRate == DWT_BR_110K)? "110K" : "6.8M", inst_ch);
    HAL_UART_Transmit(&huart1, &UART_TX_DATA[0], len, 1000);

    len = sprintf((char*)&UART_TX_DATA[0], "* data_rate = %dHz\r\n* update_time = %dms\r\n* kalmanfilter = %d\r\n", 1000 / (inst_slot_number * inst_one_slot_time), inst_slot_number * inst_one_slot_time, (switch8 & SWS1_KAM_MODE)? 1:0);
    HAL_UART_Transmit(&huart1, &UART_TX_DATA[0], len, 1000);

    len = sprintf((char*)&UART_TX_DATA[0], "* ant_dly  = %d\r\n* tx_power = %08lx\r\n", ant_dly, tx_power);
    HAL_UART_Transmit(&huart1, &UART_TX_DATA[0], len, 1000);

    // len = sprintf((char*)&UART_TX_DATA[0], "* use_eeprom = %d\r\n* use_imu = %d\r\n* use_oled = %d\r\n", USE_EEPROM, USE_IMU, USE_OLED);
    // HAL_UART_Transmit(&huart2, &UART_TX_DATA[0], len, 1000);
    
//    if(instance_mode == TAG)
//    {
//        len = sprintf((char*)&UART_TX_DATA[0], "* A0(%.2f, %.2f, %.2f) A1(%.2f, %.2f, %.2f)\r\n* A2(%.2f, %.2f, %.2f) A3(%.2f, %.2f, %.2f)\r\n", 
//                                                    anchorArray[0].x, anchorArray[0].y, anchorArray[0].z, 
//                                                    anchorArray[1].x, anchorArray[1].y, anchorArray[1].z,
//                                                    anchorArray[2].x, anchorArray[2].y, anchorArray[2].z,
//                                                    anchorArray[3].x, anchorArray[3].y, anchorArray[3].z);
//        HAL_UART_Transmit(&huart2, &UART_TX_DATA[0], len, 1000);
//    }

    len = sprintf((char*)&UART_TX_DATA[0], "***************************************************\r\n");
    HAL_UART_Transmit_DMA(&huart1, &UART_TX_DATA[0], len);
}

void delay500ms(void)
{
    unsigned char i,j,k;
    for(i=15;i>0;i--)
        for(j=202;j>0;j--)
            for(k=81;k>0;k--);
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

//static dwt_config_t uwb_config_channel7[] = {
//    {   /* uwb_config0，channel7 脉冲频率64M 前导码长度128 数据率 6M8 */
//        .chan = 7,
//        .prf = DWT_PRF_64M,
//        .txPreambLength = DWT_PLEN_128,
//        .rxPAC = DWT_PAC8,
//        .txCode = 19,
//        .rxCode = 20,
//        .nsSFD = 1,
//        .dataRate = DWT_BR_6M8,
//        .phrMode = DWT_PHRMODE_STD,
//        .sfdTO = (129 + DW_NS_SFD_LEN_6M8 - 8)
//    },
//    {   /* uwb_config1，channel7 脉冲频率64M 前导码长度256 数据率 6M8 */
//        .chan = 7,
//        .prf = DWT_PRF_64M,
//        .txPreambLength = DWT_PLEN_256,
//        .rxPAC = DWT_PAC16,
//        .txCode = 19,
//        .rxCode = 20,
//        .nsSFD = 1,
//        .dataRate = DWT_BR_6M8,
//        .phrMode = DWT_PHRMODE_STD,
//        .sfdTO = (257 + DW_NS_SFD_LEN_6M8 - 16)
//    }, 
//    {   /* uwb_config2，channel7 脉冲频率64M 前导码长度256 数据率 850K */
//        .chan = 7,
//        .prf = DWT_PRF_64M,
//        .txPreambLength = DWT_PLEN_256,
//        .rxPAC = DWT_PAC16,
//        .txCode = 19,
//        .rxCode = 20,
//        .nsSFD = 1,
//        .dataRate = DWT_BR_850K,
//        .phrMode = DWT_PHRMODE_STD,
//        .sfdTO = (257 + DW_NS_SFD_LEN_850K - 16)
//    }, 
//};

//void read_anc_coord(void)
//{
//    char anc_coord_read[56];
//    char coord_cut_data[15][10];
//    E2prom_Read(ANC_COORD_ADDR, (uint8_t*)anc_coord_read, 56);
//    if(anc_coord_read[0] == '$')
//    {
//        char *ptr, *retptr;
//        ptr = anc_coord_read;
//        uint8_t i = 0;

//        while((retptr=strtok(ptr,",")) != NULL)
//        {
//            strcpy(coord_cut_data[i], retptr);
//            ptr = NULL;
//            i++;
//        }

//        anchorArray[0].x = atof(coord_cut_data[1]);
//        anchorArray[0].y = atof(coord_cut_data[2]);
//        anchorArray[0].z = atof(coord_cut_data[3]);

//        anchorArray[1].x = atof(coord_cut_data[4]);
//        anchorArray[1].y = atof(coord_cut_data[5]);
//        anchorArray[1].z = atof(coord_cut_data[6]);

//        anchorArray[2].x = atof(coord_cut_data[7]);
//        anchorArray[2].y = atof(coord_cut_data[8]);
//        anchorArray[2].z = atof(coord_cut_data[9]);

//        anchorArray[3].x = atof(coord_cut_data[10]);
//        anchorArray[3].y = atof(coord_cut_data[11]);
//        anchorArray[3].z = atof(coord_cut_data[12]);
//    }
//}


/*
    串口指令集，注意发送指令以$开头，以\r\n结尾
    $rboot            重启
    $rantdly          查询天线延时参数
    $reset            恢复默认参数
    $santdly,16375    设置天线延时参数（10进制）
    $stxpwr,1f1f1f1f  设置发射增益参数（16进制）
    $sanccd,0,0,2,0,3.1,2,3.1,0,2,3.1,3.1,2  设置基站坐标A0.X,A0.Y,A0.Z,A1.X,A1,Y,A1,Z,A2.X,A2,Y,A2,Z,A3.X,A3,Y,A3,Z
    $sdata,abcdefg
*/

//void parse_uart(uint8_t* data)
//{
//  
//    if(strchr((char*)data, ',') > 0)//带参数指令 如 $santdly,11223
//    {
//        char *ptr, *retptr;
//        ptr = (char*)data;
//        retptr = strtok(ptr, ",");//解析数据头

//        if(strcmp(retptr, "$santdly") == 0)//设置天线延时参数  $santdly,16375
//        {
//            ptr = NULL;
//            retptr = strtok(ptr, ",");
//            ant_dly = atoi(retptr);
//            if(USE_EEPROM == 1) //板载EEPROM
//            {
//                uint8_t ant_dly_write[EEP_UNIT_SIZE] = {0};
//                ant_dly_write[0] = 0xAA;
//                ant_dly_write[1] = ant_dly >> 8;
//                ant_dly_write[2] = (uint8)ant_dly;
//                E2prom_Write(ANT_DLY_ADDR, ant_dly_write, EEP_UNIT_SIZE); 
//            }
//            HAL_NVIC_SystemReset();//重启
//        }
//        else if(strcmp(retptr, "$stxpwr") == 0)//设置发射增益参数  $stxpwr,1f1f1f1f 
//        {
//            ptr = NULL;
//            retptr = strtok(ptr, ",");
//            sscanf(retptr, "%08lx", &tx_power);
//            if(USE_EEPROM == 1) //板载EEPROM
//            {
//                uint8_t tx_pwr_write[EEP_UNIT_SIZE] = {0};
//                tx_pwr_write[0] = 0xAA;
//                tx_pwr_write[1] = tx_power >> 24;
//                tx_pwr_write[2] = tx_power >> 16;
//                tx_pwr_write[3] = tx_power >> 8;
//                tx_pwr_write[4] = tx_power;
//                E2prom_Write(TX_PWR_ADDR, tx_pwr_write, EEP_UNIT_SIZE);
//            }
//            HAL_NVIC_SystemReset();//重启 
//        }
//        else if(strcmp(retptr, "$sanccd") == 0) //给标签设置基站坐标，用于标签自己三边定位输出定位结果
//        {
//            retptr[7] = ',';
//            uint8_t zero[56] = {0};
//            if(USE_EEPROM == 1) //板载EEPROM
//            {
//                E2prom_Write(ANC_COORD_ADDR, (uint8_t*)zero, 56);
//                HAL_Delay(10);
//                E2prom_Write(ANC_COORD_ADDR, (uint8_t*)retptr, strlen((char*)retptr));
//            }
//            HAL_NVIC_SystemReset();//重启
//        }
//        else if(strcmp(retptr, "$saddr") == 0) //设置标签ID
//        {
//            ptr = NULL;
//            retptr = strtok(ptr, ",");
//            if(USE_EEPROM == 1) //板载EEPROM
//            {
//                uint8_t addr_write[EEP_UNIT_SIZE] = {0};
//                addr_write[0] = 0xAA;
//                addr_write[1] = atoi(retptr);
//                E2prom_Write(DEV_ID_ADDR, addr_write, EEP_UNIT_SIZE);
//            }
//            HAL_NVIC_SystemReset();//重启
//        }

//    }
//    else //无参数指令 如$rboot
//    {
//        if(strcmp((char*)data, "$rboot\r\n") == 0)//重启
//        {
//            HAL_NVIC_SystemReset();//重启 
//        }
//        else if(strcmp((char*)data, "$rantdly\r\n") == 0)//查询天线延时参数
//        {
//            uint8_t UART_COMMAND_BUF[50];
//            uint8_t len = sprintf((char*)UART_COMMAND_BUF, "ant_dly = %d\r\n", ant_dly);
//            HAL_UART_Transmit(&huart2, &UART_COMMAND_BUF[0], len, 1000);
//        }
//        else if(strcmp((char*)data, "$reset\r\n") == 0)//恢复默认参数
//        {
//            uint8_t write_zero[256]={0};
//            E2prom_Write(0, write_zero, 256);
//            HAL_NVIC_SystemReset();//重启 
//        }
//    }
//}

//void HAL_UART_IdleCpltCallback(UART_HandleTypeDef *huart)
//{
//    //HAL_UART_Transmit(&huart2, &UART_RX_BUF[0], strlen((char*)UART_RX_BUF), 1000);
//    uart_rx_len = strlen((char*)UART_RX_BUF);

//}

