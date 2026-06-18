/*! ----------------------------------------------------------------------------
 * @file    dw3000_port.c
 * @brief   HW specific definitions and functions for portability (STM32F103 / F1 HAL)
 *
 * @attention
 *
 * Copyright 2016-2020 (c) DecaWave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 * @author DecaWave
 *
 * 由 Anchor_RTOS(STM32F4) 版移植到标签板 STM32F103(F1)。引脚沿用 main.h 中的板载
 * UWB 引脚标签；LED/蜂鸣器实现与 port_dw1000.c 一致，保证共用应用层行为不变。
 */

#include "dw3000_port.h"
#include "deca_device_api.h"
#include "stm32f1xx_hal_conf.h"
#include "main.h"

/****************************************************************************//**
 *                              APP global variables
 *******************************************************************************/
extern SPI_HandleTypeDef hspi1;
volatile int32_t sys_time_diff = 0;

/* DW IC IRQ handler definition. */
static port_dwic_isr_t port_dwic_isr = NULL;

/****************************************************************************//**
 *                              Time section
 *******************************************************************************/

/* @fn    portGetTickCnt
 * @brief wrapper for to read a SysTickTimer, which is incremented with
 *        CLOCKS_PER_SEC frequency. The resolution is usually 1/1000 sec.
 * */
unsigned long portGetTickCnt(void)
{
    return HAL_GetTick() - sys_time_diff;
}

/* @fn    usleep
 * @brief precise usleep() delay
 * */
#pragma GCC optimize ("O0")
int usleep(unsigned int usec)
{
    unsigned int i, j;
#pragma GCC ivdep
    for(i = 0; i < usec; i++)
    {
#pragma GCC ivdep
        for(j = 0; j < 2; j++)
        {
            __NOP();
            __NOP();
        }
    }
    return 0;
}

/* @fn    Sleep
 * @brief Sleep delay in ms using SysTick timer
 * */
__INLINE void
Sleep(uint32_t x)
{
    HAL_Delay(x);
}

/****************************************************************************//**
 *                              Configuration section
 *******************************************************************************/

/* @fn    peripherals_init
 * */
int peripherals_init(void)
{
    /* All has been initialized in the CubeMx code, see main.c */
    return 0;
}

/**
  * @brief  Checks whether the specified IRQn line is enabled or not.
  * @param  IRQn: specifies the IRQn line to check.
  * @return "RESET" when IRQn is "not enabled" and "SET" otherwise
  */
ITStatus EXTI_GetITEnStatus(IRQn_Type IRQn)
{
    return ((NVIC->ISER[(((uint32_t)(int32_t)IRQn) >> 5UL)] &
        (uint32_t)(1UL << (((uint32_t)(int32_t)IRQn) & 0x1FUL))) == (uint32_t)RESET) ? (RESET) : (SET);
}

/****************************************************************************//**
 *                          DW IC reset / wakeup section
 *******************************************************************************/

/* @fn      reset_DW3000
 * @brief   通过 RSTn 引脚硬复位 DW3000：开漏输出拉低复位，再回到高阻(由上拉拉高)。
 *          注意：RSTn 不能被外部强行拉高。
 * */
void reset_DW3000(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* RSTn 配置为开漏输出，拉低进入复位 */
    GPIO_InitStruct.Pin   = DW_RSTn;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(DW_RSTn_GPIO, &GPIO_InitStruct);

    HAL_GPIO_WritePin(DW_RSTn_GPIO, DW_RSTn, GPIO_PIN_RESET);
    Sleep(2);

    /* 复位脚回到输入(高阻)，释放复位 */
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(DW_RSTn_GPIO, &GPIO_InitStruct);

    Sleep(2);
}

/*! ------------------------------------------------------------------------------------------------
 * @fn wakeup_device_with_io()
 * @brief 通过拉高 WAKEUP 引脚一段时间唤醒 DW3000。
 */
void wakeup_device_with_io(void)
{
    SET_WAKEUP_PIN_IO_HIGH;
    WAIT_500uSEC;
    SET_WAKEUP_PIN_IO_LOW;
}

/*! ------------------------------------------------------------------------------------------------
 * @fn make_very_short_wakeup_io()
 * @brief 极短地拉高 WAKEUP 引脚，器件不应被唤醒（用于自检）。
 */
void make_very_short_wakeup_io(void)
{
    uint8_t cnt;

    SET_WAKEUP_PIN_IO_HIGH;
    for(cnt = 0; cnt < 10; cnt++)
    {
        __NOP();
    }
    SET_WAKEUP_PIN_IO_LOW;
}

/* @fn      port_set_dw_ic_spi_slowrate
 * @brief   set 4.5MHz (hspi1 clocked from 72MHz)
 * */
void port_set_dw_ic_spi_slowrate(void)
{
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
    HAL_SPI_Init(&hspi1);
}

/* @fn      port_set_dw_ic_spi_fastrate
 * @brief   set 18MHz (hspi1 clocked from 72MHz) —— 与板载 DW1000 同速率，保证信号完整性
 * */
void port_set_dw_ic_spi_fastrate(void)
{
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
    HAL_SPI_Init(&hspi1);
}

/****************************************************************************//**
 *                              IRQ section
 *******************************************************************************/

/* @fn      HAL_GPIO_EXTI_Callback
 * @brief   所有 EXTI 行的 HAL 回调（DW3000 IRQ = PB0，蜂鸣器按键 = PB6）
 * */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    switch(GPIO_Pin)
    {
        case GPIO_PIN_0:        /* DW3000 IRQ (PB0) */
            process_deca_irq();
            break;

        case GPIO_PIN_6:        /* 蜂鸣器控制按键 (PB6) */
            if(bee_flag == 0)
            {
                bee_flag = 1;
            }
            else
            {
                bee_flag = 0;
            }
            break;

        default:
            break;
    }
}

/* @fn      process_deca_irq
 * @brief   main call-back for processing of DW3000 IRQ. 重入 ISR 处理所有事件，
 *          处理完后 DW3000 会清除 IRQ 线。
 * */
__INLINE void process_deca_irq(void)
{
    while(port_CheckEXT_IRQ() != 0)
    {
        if(port_dwic_isr)
        {
            port_dwic_isr();
        }
    }
}

/* @fn      port_DisableEXT_IRQ */
__INLINE void port_DisableEXT_IRQ(void)
{
    NVIC_DisableIRQ(DECAIRQ_EXTI_IRQn);
}

/* @fn      port_EnableEXT_IRQ */
__INLINE void port_EnableEXT_IRQ(void)
{
    NVIC_EnableIRQ(DECAIRQ_EXTI_IRQn);
}

/* @fn      port_GetEXT_IRQStatus */
__INLINE uint32_t port_GetEXT_IRQStatus(void)
{
    return EXTI_GetITEnStatus(DECAIRQ_EXTI_IRQn);
}

/* @fn      port_CheckEXT_IRQ
 * @brief   read DW_IRQ input pin state
 * */
__INLINE uint32_t port_CheckEXT_IRQ(void)
{
    return HAL_GPIO_ReadPin(DECAIRQ_GPIO, DECAIRQ);
}

/*! ------------------------------------------------------------------------------------------------
 * @fn port_set_dwic_isr()
 * @brief 安装 DW IC IRQ 的处理函数。安装期间会临时关闭 IRQ 线。
 */
void port_set_dwic_isr(port_dwic_isr_t dwic_isr)
{
    /* 原子安装：先存当前使能态，安装期间临时屏蔽 EXTI0，写完指针后“恢复”原态。
     * 即：原来开着才重新打开；原来关着就保持关闭——使能时机由调用方(dw3000_init)掌控。*/
    ITStatus en = port_GetEXT_IRQStatus();

    port_DisableEXT_IRQ();

    port_dwic_isr = dwic_isr;

    if(en)
    {
        port_EnableEXT_IRQ();
    }
}

/****************************************************************************//**
 *                          LED / 蜂鸣器（与 port_dw1000.c 一致）
 *******************************************************************************/

void bee_toggle(void)
{
    HAL_GPIO_TogglePin(BEE_GPIO_Port, BEE_Pin);
}
void bee_on(void)
{
    HAL_GPIO_WritePin(BEE_GPIO_Port, BEE_Pin, GPIO_PIN_SET);
}
void bee_close(void)
{
    HAL_GPIO_WritePin(BEE_GPIO_Port, BEE_Pin, GPIO_PIN_RESET);
}

void led_toggle(led_t led)
{
    switch (led)
    {
        case RUN_LED1:
            HAL_GPIO_TogglePin(RUN_LED1_GPIO_Port, RUN_LED1_Pin);
            break;
        case RUN_LED2:
            HAL_GPIO_TogglePin(RUN_LED2_GPIO_Port, RUN_LED2_Pin);
            break;
        case LED_ALL:
            HAL_GPIO_TogglePin(RUN_LED1_GPIO_Port, RUN_LED1_Pin);
            HAL_GPIO_TogglePin(RUN_LED2_GPIO_Port, RUN_LED2_Pin);
            break;
        default:
            break;
    }
}

void led_on(led_t led)
{
    switch (led)
    {
        case RUN_LED1:
            HAL_GPIO_WritePin(RUN_LED1_GPIO_Port, RUN_LED1_Pin, GPIO_PIN_SET);
            break;
        case RUN_LED2:
            HAL_GPIO_WritePin(RUN_LED2_GPIO_Port, RUN_LED2_Pin, GPIO_PIN_SET);
            break;
        case LED_ALL:
            HAL_GPIO_WritePin(RUN_LED1_GPIO_Port, RUN_LED1_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(RUN_LED2_GPIO_Port, RUN_LED2_Pin, GPIO_PIN_SET);
            break;
        default:
            break;
    }
}

void led_off(led_t led)
{
    switch (led)
    {
        case RUN_LED1:
            HAL_GPIO_WritePin(RUN_LED1_GPIO_Port, RUN_LED1_Pin, GPIO_PIN_RESET);
            break;
        case RUN_LED2:
            HAL_GPIO_WritePin(RUN_LED2_GPIO_Port, RUN_LED2_Pin, GPIO_PIN_RESET);
            break;
        case LED_ALL:
            HAL_GPIO_WritePin(RUN_LED1_GPIO_Port, RUN_LED1_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(RUN_LED2_GPIO_Port, RUN_LED2_Pin, GPIO_PIN_RESET);
            break;
        default:
            break;
    }
}
