/*! ----------------------------------------------------------------------------
 * @file    log.h
 * @brief   基于 SEGGER RTT 的调试日志封装。
 *
 *          约定：USART1 留给上位机协议通信，调试日志一律走 RTT(buffer 0)。
 *          通过 ST-Link V2 + OpenOCD 的内置 RTT 即可查看(rtt setup / rtt server start)，
 *          无需 J-Link。
 *
 *          用法：
 *              LOG_INIT();                       // 上电时调一次(可选，首次打印会自动初始化)
 *              LOG_I("dev_id = 0x%08X", id);     // 信息
 *              LOG_W("resp lost: %d", cnt);      // 警告
 *              LOG_E("dwt_initialise failed");   // 错误
 *              LOG_RAW("%d\r\n", x);             // 不带等级/换行前缀的原样输出
 *
 *          关闭日志：把 LOG_ENABLE 改为 0，所有 LOG_* 编译为空，零开销。
 * --------------------------------------------------------------------------- */

#ifndef LOG_H
#define LOG_H

#ifdef __cplusplus
extern "C" {
#endif

#include "SEGGER_RTT.h"

/* 日志总开关：1=启用(走 RTT)，0=全部编译为空 */
#define LOG_ENABLE      1

/* RTT 上行缓冲区索引（buffer 0 是预初始化的 Terminal 通道）*/
#define LOG_RTT_BUFFER  0

#if (LOG_ENABLE)

    #define LOG_INIT()          SEGGER_RTT_Init()

    /* 原样输出：自带格式，不加等级前缀/换行 */
    #define LOG_RAW(...)        SEGGER_RTT_printf(LOG_RTT_BUFFER, __VA_ARGS__)

    /* 带等级与颜色，自动补 \r\n */
    #define LOG_I(fmt, ...)     SEGGER_RTT_printf(LOG_RTT_BUFFER, \
                                    RTT_CTRL_TEXT_BRIGHT_GREEN  "[I] " RTT_CTRL_RESET fmt "\r\n", ##__VA_ARGS__)
    #define LOG_W(fmt, ...)     SEGGER_RTT_printf(LOG_RTT_BUFFER, \
                                    RTT_CTRL_TEXT_BRIGHT_YELLOW "[W] " RTT_CTRL_RESET fmt "\r\n", ##__VA_ARGS__)
    #define LOG_E(fmt, ...)     SEGGER_RTT_printf(LOG_RTT_BUFFER, \
                                    RTT_CTRL_TEXT_BRIGHT_RED    "[E] " RTT_CTRL_RESET fmt "\r\n", ##__VA_ARGS__)

#else   /* 关闭：全部空操作 */

    #define LOG_INIT()          ((void)0)
    #define LOG_RAW(...)        ((void)0)
    #define LOG_I(fmt, ...)     ((void)0)
    #define LOG_W(fmt, ...)     ((void)0)
    #define LOG_E(fmt, ...)     ((void)0)

#endif /* LOG_ENABLE */

#ifdef __cplusplus
}
#endif

#endif /* LOG_H */
