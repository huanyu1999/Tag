/**
 ******************************************************************************
 * @file      sysmem.c
 * @author    STM32CubeIDE 风格的 _sbrk 堆管理实现（GCC newlib 用）
 * @brief     提供 malloc/printf 浮点格式化等所需的堆增长函数 _sbrk。
 *            堆从链接脚本里的 _end（end）向上生长，触及 _estack 预留的
 *            最小栈空间 _Min_Stack_Size 时返回失败，避免堆栈对撞。
 ******************************************************************************
 */

#include <errno.h>
#include <stdint.h>

extern int errno;

/* 来自链接脚本 STM32F103XX_FLASH.ld */
extern uint8_t _end;             /* 堆起点（= end）  */
extern uint8_t _estack;          /* RAM 顶 = 栈底    */
extern uint8_t _Min_Stack_Size;  /* 预留最小栈空间   */

void *_sbrk(ptrdiff_t incr)
{
    static uint8_t *heap_end = 0;
    const uint8_t *max_heap = (uint8_t *)((uint32_t)&_estack - (uint32_t)&_Min_Stack_Size);
    uint8_t *prev_heap_end;

    if (heap_end == 0) {
        heap_end = &_end;
    }
    prev_heap_end = heap_end;

    if (heap_end + incr > max_heap) {
        errno = ENOMEM;
        return (void *)-1;
    }

    heap_end += incr;

    return (void *)prev_heap_end;
}
