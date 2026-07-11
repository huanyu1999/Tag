#include "dw_instance.h"
#include "log.h"                                        //SEGGER RTT 调试日志（USART1 留给上位机协议）


/* poll数据帧格式 */
static uint8_t tx_poll_msg[POLL_MSG_LEN] = { 0x41, 0x88, 0, 0xCA, 0xDE, 0xFF, 0xFF, 0x00, 0x00, FUNC_CODE_POLL, 0x00, 0x00, 0x00, 0x00 };
/* final数据帧格式 */
static uint8_t tx_final_msg[FIANL_MSG_LEN] = {0x41, 0x88, 0, 0xCA, 0xDE, 0xFF, 0xFF, 0x00, 0x00, FUNC_CODE_FINAL, 0X00};
/* 接收数据buffer */
static uint8_t rx_buffer[FRAME_LEN_MAX];

static uint8_t resp_expect = MAX_AHCHOR_NUMBER;         // resp消息接收个数
static int tagSleepCorrection_ms = 0;                   // 标签时序校准，用于slot分配防冲突管理
uint32_t volatile next_period_time = 0;                 // 标签下次测距周期开始时间，用于防冲突管理
uint8_t Correction_flag = 0;                            // 该标签已经被时序校准的标志位

/* TWR时间戳，用于计算飞行时间 */
static uint64_t poll_tx_ts;                     
static uint64_t resp_rx_ts[MAX_AHCHOR_NUMBER];          // 标签接收到的基站resp帧数组
static uint64_t final_tx_ts;

/* 发送和接收数据中断标志 */
static volatile uint8_t rx_status = RX_WAIT;
static volatile uint8_t tx_status = TX_WAIT;

static uint8_t resp_valid = 0x00;               // 基站数据有效标志
uint32 diff_time;                               // 标签增加发起测距随机时间，避免冲突
static uint8_t led_flag = 0;
static uint32_t led_time = 0;

static inline uint64_t get_tx_timestamp_u64(void);
static inline uint64_t get_rx_timestamp_u64(void);
static inline void final_msg_get_ts(const uint8_t *ts_field, uint32_t *ts);
static inline void final_msg_set_ts(uint8_t *ts_field, uint64_t ts);

/* TWR 延迟收发时刻计算（TREK1000 时序模型，与基站二进制级同源，见 ../Anchor_RTOS/docs/TWR_TIMING.md）。
 * 时序参数由 dw_main.c 的 twr_set_replydelay() 按帧长公式统一算出，装在 twr_timings 里，
 * 芯片无关（不再按 DW1000/DW3000 × 速率展开六个分支），状态机只调用。
 * 32h = 40bit DW 设备时间的高 32 位（>>8），dwt_setdelayedtrxtime 的原生单位。 */

/* resp 槽 k（首个 k=0）的延迟开窗时刻 = poll TX + (k+1)×统一槽间隔 − 前导码提前量。
 * 减前导码是必须的：delayed TX 编程的是 RMARKER 时刻（基站的 resp RMARKER 正落在槽时刻），
 * 而 delayed RX 编程的是接收机开机时刻，resp 的前导码在其 RMARKER 之前就开始飞，
 * 不提前开窗就会漏掉整个前导码而收不到帧（与基站 anch_txresponse_or_rx_reenable 同款）。 */
static uint32_t tag_calc_resp_rx_time(uint64_t poll_ts, uint8_t resp_index)
{
    return (uint32_t)(poll_ts >> 8)
         + (uint32_t)(resp_index + 1) * twr_timings.fixedReplyDelayAnc32h
         - twr_timings.preambleDuration32h;
}

/* final 的 TX RMARKER 时刻 = poll TX + (N+1)×统一槽间隔。
 * 基站按同一个 pollTx2FinalTxDelay32h 开 final 接收窗，双端必须同值。 */
static uint32_t tag_calc_final_tx_time(uint64_t poll_ts)
{
    return (uint32_t)(poll_ts >> 8) + twr_timings.pollTx2FinalTxDelay32h;
}

void tag_app(void)
{
    switch (state)
    {
    case STA_SEND_POLL:  // 打包和发送poll消息
    {
        if (inst_slot_number > 1)      // 已经打开了随机时间生成
        {
            diff_time += (tag_id + 1); // 产生随机时间避免一直冲突
            if(diff_time > sfConfig.tagPeriod_ms / 2) // 如果这个冲突校准时间比整个超级帧持续时间的一半还长，就置0
            {
                diff_time = 0;
            }
        }
        else
        {
            diff_time = 0;
        }
        range_nb++;
        /* poll数据打包 */
        tx_poll_msg[SEQ_NB_IDX]    = frame_seq_nb++;  
        tx_poll_msg[PANID_IDX]     = (uint8_t)PAN_ID; 
        tx_poll_msg[PANID_IDX + 1] = (uint8_t)(PAN_ID>>8); 
        tx_poll_msg[RANGE_NB_IDX]  = range_nb;  
        tx_poll_msg[SENDER_SHORT_ADD_IDX] = tag_id;
        tx_poll_msg[FUNC_CODE_IDX]        = FUNC_CODE_POLL;
        tx_poll_msg[POLL_MSG_SOS_IDX]     = sos;
        if (alarm > 0)
        {
            alarm = 1;
        }
        tx_poll_msg[POLL_MSG_ALARM_STA_IDX] = alarm;
#if defined(ANCRANGE)  
        if(ancrange_flag > 0)
        {
            battery = 0xff;
        }
#endif
        for (int i = 0; i < 10; i++)  //打包用户字节
        {
            tx_poll_msg[POLL_MSG_USER_IDX + i] = user_data[i];
        }

        range_time = portGetTickCnt();                           // 获取测距时间，用于规划下一次TWR周期的起始时间
        dwt_writetxdata(POLL_MSG_LEN + FCS_LEN, tx_poll_msg, 0); // 数据写入DW1000数据缓冲区
        dwt_writetxfctrl(POLL_MSG_LEN + FCS_LEN, 0, 1);          // 配置TX帧控制寄存器
        tx_status = TX_WAIT;                                     // 发送状态标志，在中断回调函数变更
        int ret = dwt_starttx(DWT_START_TX_IMMEDIATE);           // 立即发送POLL消息
        if (ret == DWT_ERROR)
        {
            next_period_time = range_time + sfConfig.tagPeriod_ms + ((Correction_flag == 1)?0:(diff_time));  //如该标签未被时序校准，初次上电，增加随机时间，避免一直冲突      
            state = STA_IDLE;
            break;
        }
        while (tx_status == TX_WAIT);                       // 等待发送成功，tx_status在发送成功中断内变更状态
        tx_status   = TX_WAIT;                              // 清标志
        poll_tx_ts  = get_tx_timestamp_u64();               // 取得poll_tx时间戳
        resp_expect = MAX_AHCHOR_NUMBER;                    // 发送poll消息后，等待最大基站数量个resp消息回复
        resp_valid  = 0;                                    // resp有效校验初始化
        memset(rx_buffer, 0, sizeof(rx_buffer));
        // 发送POLL之后，延时开启接收等待resp
        dwt_setrxtimeout(twr_timings.fwto4RespFrame_sy);  // 设置接收超时时间（symbol，dwt_setrxtimeout 的原生单位）
        dwt_setpreambledetecttimeout(PRE_TIMEOUT);  // 设置前导码超时
        uint32_t resp_rx_time = tag_calc_resp_rx_time(poll_tx_ts, 0);  // 计算首个 resp 的延迟接收开启时间
        dwt_setdelayedtrxtime(resp_rx_time);        // 设置接收机开启延时时间
        ret = dwt_rxenable(DWT_START_RX_DELAYED);   // 延时开启接收机
        if (ret == DWT_ERROR)
        {
            next_period_time = range_time + sfConfig.tagPeriod_ms + ((Correction_flag == 1)?0:(diff_time));  //如该标签未被时序校准，初次上电，增加随机时间，避免一直冲突      
            state = STA_IDLE;
            break;
        }
        alarm = 0;
        for (int i = 0; i < 10; i++)  //清除用户字节
        {
            user_data[i] = 0;
        }
#if defined(ANCRANGE)        
        if(ancrange_flag == 2)
        {
            ancrange_count--;
        }
#endif
        rx_status = RX_WAIT;
        state = STA_WAIT_RESP;
        break;
    }

    case STA_WAIT_RESP:         // 等待resp数据接收
    {
        if(rx_status == RX_OK)  // 接收成功
        {
            state = STA_RECV_RESP;
            // LOG_I("rx resp ok.");
        }
        else if((rx_status == RX_TIMEOUT) || (rx_status == RX_ERROR))// 接收超时或接收错误
        {
            state = STA_RECV_RESP;
            // LOG_I("rx reception.");
        }
        if(portGetTickCnt() >= (range_time + 50))  // 超时没有中断信号，故障，重启
        {
            HAL_NVIC_SystemReset();  // 重启
        }
        break;
    }

    case STA_RECV_RESP:
    {
        static uint8_t resp_recved = 0;     // 单slot内接收到的resp个数，用于判定如果1个resp没收到则不发送final
        static uint8_t no_resp_count = 0;   // 未接收到任何基站回复的周期计数
        if ((rx_status == RX_OK) && (rx_buffer[FUNC_CODE_IDX] != FUNC_CODE_RESP))    // 接收消息成功，但是消息不是resp消息则直接进入IDLE
        {
            next_period_time = range_time + sfConfig.tagPeriod_ms + ((Correction_flag == 1) ? 0 : (diff_time));  //如该标签未被时序校准，初次上电，增加随机时间，避免一直冲突      
            state = STA_IDLE;
        }
        else
        {
            if (rx_buffer[FUNC_CODE_IDX] == FUNC_CODE_RESP)      // 正确接收到resp消息
            {
                recv_anc_id = rx_buffer[SENDER_SHORT_ADD_IDX];   // 取发送方基站ID
                if (rx_buffer[RANGE_NB_IDX] == range_nb)         // 和poll相同的range_nb
                {
                    resp_valid = resp_valid | (0x01 << recv_anc_id);  // 设置该基站resp有效，
                    resp_rx_ts[recv_anc_id] = get_rx_timestamp_u64(); // 取该基站resp_rx时间戳，用于tof计算
                    resp_recved++;                                    // 接收到的resp帧计数加一
                    /* 将resp消息内的测距信息取出，用于串口数据打包输出 */
                    distance_report[recv_anc_id]  = (int32_t)rx_buffer[RESP_MSG_PREV_DIS_IDX]   << 24;
                    distance_report[recv_anc_id] += (int32_t)rx_buffer[RESP_MSG_PREV_DIS_IDX+1] << 16;
                    distance_report[recv_anc_id] += (int32_t)rx_buffer[RESP_MSG_PREV_DIS_IDX+2] << 8;
                    distance_report[recv_anc_id] += (int32_t)rx_buffer[RESP_MSG_PREV_DIS_IDX+3];
                    group_report[recv_anc_id] = rx_buffer[RESP_MSG_GROUP_IDX] & 0x7f;           // 将最高bit置0，最高bit为校准基站标志位
                }
                alarm += rx_buffer[RESP_MSG_ALARM_IDX];
                // 当收到A0基站的resp时，取tagSleepCorrection用于校准
                // if(recv_anc_id == 0)
                // 当基站组ID的最高bit = 1时，为时序校准基站，取tagSleepCorrection用于校准
                if((rx_buffer[RESP_MSG_GROUP_IDX] != 0xff) && ((rx_buffer[RESP_MSG_GROUP_IDX] & 0x80) == 0x80))
                {
                    tagSleepCorrection_ms = (int16) (((uint16) rx_buffer[RESP_MSG_SLEEP_COR_IDX] << 8) + rx_buffer[RESP_MSG_SLEEP_COR_IDX+1]);//高8位存11  低8位存12
                    Correction_flag = 1;            // 接收到基站发送的校准时间，将该标志位置1
                    /* RSSI 诊断读取（占位，dwt_readdiagnostics 暂未启用，故 rx_power 保持 0）。
                     * 原先在此读未初始化的 dwt_rxdiag_t，rx_power 拿到的是栈上垃圾值，已移除。
                     * 启用时注意：DW1000 的 dwt_rxdiag_t 有厂家自加的 rxPower 字段，
                     * DW3000 结构不同（ipatovPower/stsPower），需分芯片实现。 */
                }
            }
            resp_expect--;
            if(resp_expect == 0)// 所有resp消息接收完成，开始判断接收情况，决定是否发送final帧
            {
                if(resp_recved == 0)// 如果一个resp也没收到，则不发final，直接进入IDLE，可在此添加异常指示
                {
                    if((sos == 0) && (alarm == 0))
                    {
//                            led_off(LED3);// 绿
//                            led_off(LED2);// 蓝
//                            led_toggle(LED1); // 红灯闪烁
                    }
                    no_resp_count++;
                    if(no_resp_count > 10)
                    {
                        Correction_flag = 0;
                        no_resp_count = 0;
                    }
                    next_period_time = range_time + sfConfig.tagPeriod_ms + ((Correction_flag == 1) ? (0) : (diff_time));  //如该标签未被时序校准，初次上电，增加随机时间，避免一直冲突
                    state = STA_IDLE;
                    range_status = RANGE_ERROR; 
                    break;
                }
                else        // 收到1个及以上resp消息
                {
                    if((sos == 0) && (alarm == 0))
                    {
//                      led_off(LED1);//红
//                      led_off(LED2);//蓝
//                      led_toggle(LED3); //绿灯闪烁
                    }
                    state = STA_SEND_FINAL;
                    resp_recved = 0;
                    no_resp_count = 0;
                }
            }
            else//继续接收其他resp消息
            {
                // 设置resp数据接收机开启时间，记得要取高23bit；resp_index = 已收/已等待的 resp 序号
                uint32_t resp_rx_time = tag_calc_resp_rx_time(poll_tx_ts, (MAX_AHCHOR_NUMBER - resp_expect));
                dwt_setdelayedtrxtime(resp_rx_time);                // 设置接收机开启延时时间
                int ret = dwt_rxenable(DWT_START_RX_DELAYED);       // 延时开启接收机
                if(ret == DWT_ERROR)
                {
                    next_period_time = range_time + sfConfig.tagPeriod_ms + ((Correction_flag == 1)?0:(diff_time)); // 如该标签未被时序校准，初次上电，增加随机时间，避免一直冲突
                    state = STA_IDLE;
                    break;
                }
                state = STA_WAIT_RESP;
            }
        }
        rx_status = RX_WAIT;
        break;
    }

    case STA_SEND_FINAL:
    {
        // 设置final发送时间（32h，即 40bit 设备时间的高32位）
        uint32_t final_tx_time = tag_calc_final_tx_time(poll_tx_ts);
        dwt_setdelayedtrxtime(final_tx_time);   // 在final_tx_time这个时间发送数据

        /* final 是延时发送，其时间戳由调度时刻手工算出而非硬件读取，必须补上 TX 天线延时，
         * 才能与 poll_tx / resp_rx（硬件读取，已含天线延时）保持同一基准。
         * 漏加会让基站侧 Da 偏小、TOF 偏大（基站实测过 ~15m 误差）。
         * 32h<<8 还原 40bit：MASK_TXDTS 本就丢弃低 9 位，bit9 以上无损。 */
        final_tx_ts = ((((uint64_t)final_tx_time) << 8) & MASK_TXDTS) + ant_dly;  // final发送时间戳

        // final数据包内写入poll_tx时间戳，final_tx时间戳，基站个数的resp_rx时间戳
        memcpy(&tx_final_msg[FINAL_MSG_POLL_TX_TS_IDX], (uint8_t *) &poll_tx_ts, 5);
        for(int i = 0; i < MAX_AHCHOR_NUMBER; i++)
        {
            memcpy(&tx_final_msg[FINAL_MSG_RESP1_RX_TS_IDX + i * (FINAL_MSG_TS_LEN + 1)], (uint8_t *) &resp_rx_ts[i], 5);
        }
        memcpy(&tx_final_msg[FINAL_MSG_FINAL_TX_TS_IDX], (uint8_t*) &final_tx_ts, 5);

        // final数据打包
        tx_final_msg[SEQ_NB_IDX] = frame_seq_nb++;
        tx_final_msg[PANID_IDX] = (uint8_t)PAN_ID; 
        tx_final_msg[PANID_IDX + 1] = (uint8_t)(PAN_ID>>8); 
        tx_final_msg[RANGE_NB_IDX] = range_nb;
        tx_final_msg[SENDER_SHORT_ADD_IDX] = tag_id;
        tx_final_msg[FINAL_MSG_FINAL_VALID_IDX] = resp_valid;
        tx_final_msg[FUNC_CODE_IDX] = FUNC_CODE_FINAL;

        for(int i = 0; i < MAX_AHCHOR_NUMBER; i++)
        {
            tx_final_msg[FINAL_MSG_A0_GROUP_ID_IDX + i*5] = group_report[i];
        }

        dwt_writetxdata(FIANL_MSG_LEN + FCS_LEN, tx_final_msg, 0); // 数据写入数据缓冲区
        dwt_writetxfctrl(FIANL_MSG_LEN + FCS_LEN, 0, 1); 

        tx_status = TX_WAIT;                                       // 发送状态标志，在中断回调函数变更
        if(dwt_starttx(DWT_START_TX_DELAYED) == DWT_ERROR)
        {
            /* 诊断(临时)：FINAL 延时发送时刻已过 → 发不出去 → 基站收不到 FINAL → 测不出距离。
             * 只在失败分支打印(本周期已放弃，不影响时序)。排查完可删。 */
            LOG_W("FINAL tx FAILED (delayed time passed), rb=%d", range_nb);
            next_period_time = range_time + sfConfig.tagPeriod_ms;//设置下个周期开始时间
            state = STA_IDLE;
            break;
        }
        while(tx_status == TX_WAIT);            // 等待发送成功，tx_status在发送成功中断内变更状态
        tx_status = TX_WAIT;                    // 清标志
        range_status = RANGE_TWR_OK;            // 设置TWR成功测距标志，在dw_main.c里判断打包串口输出       
        next_period_time = range_time + sfConfig.tagPeriod_ms + tagSleepCorrection_ms;  //设置下个周期开始时间
        tagSleepCorrection_ms = 0;
        state = STA_IDLE;
        break;
    }
        
    case STA_IDLE:
    {
        static unsigned long nowtime;
        nowtime = portGetTickCnt();
#if defined(ANCRANGE)
        if((ancrange_count == 0) && (ancrange_flag == 2)) //本次切换T0测距结束，切回基站
        {
            dev_id = temp_dev_id;
            instance_mode = ANCHOR; 
            ancrange_flag = 0;
            set_instance();
            break;
        }
        
        if(ancrange_flag == 1) //基站切换tag，第一次发送poll按同步时间slot发送
        {
            if(nowtime % (sfConfig.tagPeriod_ms) == 0) //用T0 slot发送
            {
                ancrange_flag = 2; //设置基站间测距模式开启 1=开始 2=运行中 0=停止
                dwt_forcetrxoff();
                state = STA_SEND_POLL;
                break;
            }
        }
        else
#endif
        {
            if(nowtime >= next_period_time)  //下个周期发送时间到
            {
                dwt_forcetrxoff();
//              battery = 0;
//              if(USE_CW2015 == 1)
//              {
//                  battery = cw2015_read_battery();
//              }
                state = STA_SEND_POLL;
                break;
            }
        }

        //LED控制
        if((portGetTickCnt() % 200 == 0) && (portGetTickCnt() > (led_time + 5)) && ((sos == 1) || (alarm > 0)))
        {
            led_time = portGetTickCnt();
            if(led_flag)
            {
//              led_on(LED1); //红灯
//              led_off(LED2); //绿灯
//              led_off(LED3); //蓝
            }
            else
            {
//              led_off(LED1); //红灯
//              led_on(LED2); //绿灯
//              led_off(LED3); //蓝
            }
            led_flag = !led_flag;
        }
        break;
    }

    default:
        break;
    }
}



/*! ------------------------------------------------------------------------------------------------------------------
 * @fn tag_rx_ok_cb()
 *
 * @brief Callback to process RX good frame events
 *
 * @param  cb_data  callback data
 *
 * @return  none
 */
void tag_rx_ok_cb(const dwt_cb_data_t *cb_data)
{
    rx_status = RX_OK;
    if (cb_data->datalength <= FRAME_LEN_MAX)
    {
        dwt_readrxdata(rx_buffer, cb_data->datalength, 0);
    }

    UNUSED(cb_data);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn tag_rx_to_cb()
 *
 * @brief Callback to process RX timeout events
 *
 * @param  cb_data  callback data
 *
 * @return  none
 */
void tag_rx_to_cb(const dwt_cb_data_t *cb_data)
{
    rx_status = RX_TIMEOUT;
    UNUSED(cb_data);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn tag_rx_err_cb()
 *
 * @brief Callback to process RX error events
 *
 * @param  cb_data  callback data
 *
 * @return  none
 */
void tag_rx_err_cb(const dwt_cb_data_t *cb_data)
{
    rx_status = RX_ERROR;
    UNUSED(cb_data);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn tag_tx_conf_cb()
 *
 * @brief Callback to process TX confirmation events
 *
 * @param  cb_data  callback data
 *
 * @return  none
 */
void tag_tx_conf_cb(const dwt_cb_data_t *cb_data)
{
    tx_status = TX_OK;
    UNUSED(cb_data);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn get_tx_timestamp_u64()
 *
 * @brief Get the TX time-stamp in a 64-bit variable.
 *        /!\ This function assumes that length of time-stamps is 40 bits, for both TX and RX!
 *
 * @param  none
 *
 * @return  64-bit value of the read time-stamp.
 */
static inline uint64_t get_tx_timestamp_u64(void)
{
    uint8_t ts_tab[5];
    uint64_t ts = 0;

    dwt_readtxtimestamp(ts_tab);
    memcpy(&ts, ts_tab, 4); // 拷贝低32bit
    ts |= (uint64_t)ts_tab[4] << 32; // 添加上高8 bit
    return ts;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn get_rx_timestamp_u64()
 *
 * @brief Get the RX time-stamp in a 64-bit variable.
 *        /!\ This function assumes that length of time-stamps is 40 bits, for both TX and RX!
 *
 * @param  none
 *
 * @return  64-bit value of the read time-stamp.
 */
static inline uint64_t get_rx_timestamp_u64(void)
{
    uint8_t ts_tab[5];
    uint64_t ts = 0;

    dwt_readrxtimestamp(ts_tab);
    memcpy(&ts, ts_tab, 4); // 拷贝低32bit
    ts |= (uint64_t)ts_tab[4] << 32; // 添加上高8 bit
    return ts;
}


/*! ------------------------------------------------------------------------------------------------------------------
 * @fn final_msg_get_ts()
 *
 * @brief Read a given timestamp value from the final message. In the timestamp fields of the final message, the least
 *        significant byte is at the lower address.
 *
 * @param  ts_field  pointer on the first byte of the timestamp field to read
 *         ts  timestamp value
 *
 * @return none
 */
static inline void final_msg_get_ts(const uint8_t *ts_field, uint32_t *ts)
{
    memcpy(ts, ts_field, 4);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn final_msg_set_ts()
 *
 * @brief Fill a given timestamp field in the final message with the given value. In the timestamp fields of the final
 *        message, the least significant byte is at the lower address.
 *
 * @param  ts_field  pointer on the first byte of the timestamp field to fill
 *         ts  timestamp value
 *
 * @return none
 */
static inline void final_msg_set_ts(uint8_t *ts_field, uint64_t ts)
{
    memcpy(ts_field, &ts, 4);
} 
