/******************************************************************************************************
 * dw_power.c —— DW 芯片深睡 / 唤醒封装（Phase 2 低功耗）
 *
 * 标签一个测距周期只占超帧 600ms 里的 ~9ms，其余时间收发机纯耗电。本文件把 DW1000/DW3000 两颗
 * 芯片的深睡与唤醒差异封装掉，状态机（dw_instance_tag.c）只调 enter/wakeup 两个函数，不感知芯片。
 *
 * 关键约束（踩过的坑，改动前务必读）：
 * 1. 深睡**不保留**天线延时 / TX 功率 / PA-LNA / 中断掩码，唤醒后必须整套重下。漏下天线延时的
 *    表现是"睡醒后第一轮测距距离系统性偏移数米"，而不是报错 —— 极难归因。
 *    重下动作统一走 dw_main.c 的 dw_apply_runtime_config()，与 init 同一份，防两处漂移。
 * 2. DW1000：IRQ(PB0) 与 RSTn(PA0) 共用 EXTI0，而 port_wakeup_IC_fast() 内部最后一步是
 *    setup_DW1000RSTnIRQ(0) —— 它会 HAL_NVIC_DisableIRQ(EXTI0_IRQn)。唤醒后不重新使能，
 *    所有 dwt_* 回调都不会再触发，标签直接卡死在 STA_WAIT_RESP 的看门狗上。
 *    （与 dw1000_init() 末尾那句 HAL_NVIC_EnableIRQ(EXTI0_IRQn) 是同一个坑。）
 * 3. 进深睡前必须 dwt_forcetrxoff()：收发机开着时芯片不会进入 SLEEP。
 ******************************************************************************************************/
#include "dw_instance.h"
#include "log.h"

/* 深睡参数配置，init 时调用一次即可（配置写进 AON，掉电域保持） */
void tag_dw_sleep_config(void)
{
#if defined(USE_DW1000)
    /* mode: PRESRV_SLEEP=唤醒后保留 SLEEP 使能位；CONFIG=唤醒时把 AON 阵列回灌到主机接口寄存器；
     *       TANDV=唤醒时采一次温度/电压（TX 功率随温补用）。
     * wake: WAKE_CS=拉低 CS 唤醒（port_wakeup_IC_fast 就是这么干的）；SLP_EN=使能睡眠功能。 */
    dwt_configuresleep(DWT_PRESRV_SLEEP | DWT_CONFIG | DWT_TANDV,
                       DWT_WAKE_CS | DWT_SLP_EN);
#elif defined(USE_DW3000)
    /* DW3000: CONFIG=唤醒回灌 AON；PGFCAL=唤醒重跑 PG 校准（DW3000 深睡后必须，否则 TX 频谱跑偏）。
     * wake: 同时允许 CSN 与 WAKEUP 引脚(PB1) 唤醒，本工程用 WAKEUP 引脚。 */
    dwt_configuresleep(DWT_CONFIG | DWT_PGFCAL,
                       DWT_PRES_SLEEP | DWT_WAKE_CSN | DWT_WAKE_WUP | DWT_SLP_EN);
#endif
}

/* 进入深睡：本轮测距收尾时调用（tag_round_end）*/
void tag_dw_entersleep(void)
{
    dwt_forcetrxoff();          //收发机开着进不了深睡，必须先关
#if defined(USE_DW1000)
    dwt_entersleep();
#elif defined(USE_DW3000)
    dwt_entersleep(DWT_DW_IDLE);//唤醒后停在 IDLE_PLL，可直接收发
#endif
}

/* 唤醒并把深睡丢掉的配置全部重下：下一轮 poll 发送前 DW_WAKEUP_LEAD_MS 调用 */
void tag_dw_wakeup(void)
{
#if defined(USE_DW1000)
    port_wakeup_IC_fast();      //CS 拉低唤醒 + 轮询 RSTn 拉高，~2.2ms（含晶振起振）
    /* 见文件头约束 2：port_wakeup_IC_fast() 末尾关掉了 EXTI0，这里必须重新打开，
     * 否则 DW1000 的 IRQ(PB0) 永久失效 —— 唤醒成功但收发回调再也不来。 */
    HAL_NVIC_EnableIRQ(EXTI0_IRQn);
#elif defined(USE_DW3000)
    wakeup_device_with_io();    //WAKEUP 引脚(PB1) 拉高 500us 唤醒
    while (!dwt_checkidlerc())  //等芯片回到 IDLE_RC 才能下配置
    { };
    dwt_restoreconfig();        //回灌深睡前保存的配置
#endif

    /* 深睡不保留的那一套（天线延时/TX功率/PANID/帧过滤/PA-LNA/中断掩码），与 init 共用同一份实现 */
    dw_apply_runtime_config();

    /* 短地址与 PAN ID 同在 PANADR 寄存器：深睡是否保留不确定，重下一次最省事（一条 SPI 写）。
     * 丢了的表现是帧过滤把基站发给本标签的 resp 全部拒掉 —— "睡醒后再也测不出距离"。
     * 不放进 dw_apply_runtime_config()：init 里 tag_id 要到该函数之后才赋值。 */
    dwt_setaddress16(tag_id);
}
