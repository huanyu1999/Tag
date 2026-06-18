/*! ----------------------------------------------------------------------------
 * @file    dw3000_port.h
 * @brief   HW specific definitions and functions for portability (STM32F103 / F1 HAL)
 *
 * @attention
 *
 * Copyright 2015-2020 (c) DecaWave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 * @author DecaWave
 *
 * 说明：本文件由 Anchor_RTOS(STM32F4) 版移植到标签板 STM32F103(F1)。
 *      标签板单芯片在位（DW1000 或 DW3000），二者共用同一组物理引脚，
 *      因此直接复用 CubeMX 在 main.h 中生成的引脚标签（名字带 DW1000_ 前缀，
 *      实为板载 UWB 引脚）。LED/蜂鸣器 API 与 port_dw1000.h 保持一致，供共用应用层调用。
 */


#ifndef DW3000_PORT_H_
#define DW3000_PORT_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <string.h>
#include "compiler.h"

#include "stm32f1xx.h"
#include "stm32f1xx_hal.h"
#include "main.h"

/* SPI1 read temp buffer. */
#define BUFFLEN     (128)
#define BUF_SIZE    (64)

typedef uint64_t        uint64;
typedef int64_t         int64;

typedef enum
{
    RUN_LED1,
    RUN_LED2,
    LED_ALL
} led_t;

extern volatile int32_t sys_time_diff;
extern uint8_t bee_flag;

/* DW IC IRQ (EXTI) handler type。中断里通过它分发到 dwt_isr() */
typedef void (*port_dwic_isr_t)(void);
void port_set_dwic_isr(port_dwic_isr_t isr);

/****************************************************************************//**
 *                              引脚映射（板载 UWB 引脚，沿用 main.h 标签）
 *  IRQ  = PB0 (EXTI0)   RSTn = PA12   NSS/CS = PA4   WAKEUP = PB1
 *******************************************************************************/
#define DECAIRQ_EXTI_IRQn           (DW1000_IRQ_EXTI_IRQn)      /* EXTI0_IRQn */

#define DW_RSTn                     DW1000_RSTn_Pin
#define DW_RSTn_GPIO                DW1000_RSTn_GPIO_Port

#define DECAIRQ                     DW1000_IRQ_Pin
#define DECAIRQ_GPIO                DW1000_IRQ_GPIO_Port

#define DW_NSS_Pin                  GPIO_PIN_4
#define DW_NSS_GPIO_Port            GPIOA

#define DW_WAKEUP_Pin               DW1000_WakeUp_Pin
#define DW_WAKEUP_GPIO_Port         DW1000_WakeUp_GPIO_Port


#define GPIO_ResetBits(x,y)                 HAL_GPIO_WritePin(x,y, RESET)
#define GPIO_SetBits(x,y)                   HAL_GPIO_WritePin(x,y, SET)
#define GPIO_ReadInputDataBit(x,y)          HAL_GPIO_ReadPin (x,y)

/* NSS pin is SW controllable (PA4) */
#define port_SPIx_set_chip_select()         HAL_GPIO_WritePin(DW_NSS_GPIO_Port, DW_NSS_Pin, GPIO_PIN_SET)
#define port_SPIx_clear_chip_select()       HAL_GPIO_WritePin(DW_NSS_GPIO_Port, DW_NSS_Pin, GPIO_PIN_RESET)

/* Wakeup pin IO control（DW3000 用板载 WakeUp 引脚 PB1 唤醒） */
#define SET_WAKEUP_PIN_IO_LOW     HAL_GPIO_WritePin(DW_WAKEUP_GPIO_Port, DW_WAKEUP_Pin, GPIO_PIN_RESET)
#define SET_WAKEUP_PIN_IO_HIGH    HAL_GPIO_WritePin(DW_WAKEUP_GPIO_Port, DW_WAKEUP_Pin, GPIO_PIN_SET)

#define WAIT_500uSEC    Sleep(1) /* This should be a delay of 500uSec at least */


void Sleep(uint32_t Delay);
unsigned long portGetTickCnt(void);
int usleep(unsigned int usec);

void port_set_dw_ic_spi_slowrate(void);
void port_set_dw_ic_spi_fastrate(void);

void process_deca_irq(void);

int  peripherals_init(void);

void reset_DW3000(void);

void wakeup_device_with_io(void);
void make_very_short_wakeup_io(void);

/* LED / 蜂鸣器：与 port_dw1000.h 同名同义，供共用应用层(dw_main.c)调用 */
void led_on(led_t led);
void led_off(led_t led);
void led_toggle(led_t led);
void bee_toggle(void);
void bee_on(void);
void bee_close(void);

ITStatus EXTI_GetITEnStatus(IRQn_Type x);

uint32_t port_GetEXT_IRQStatus(void);
uint32_t port_CheckEXT_IRQ(void);
void port_DisableEXT_IRQ(void);
void port_EnableEXT_IRQ(void);
extern uint32_t HAL_GetTick(void);

#ifdef __cplusplus
}
#endif

#endif /* DW3000_PORT_H_ */
