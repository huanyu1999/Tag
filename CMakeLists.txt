# 设置cmake的最小需求版本
cmake_minimum_required(VERSION 3.22)

set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# 设置C语言标准为C99
set(CMAKE_C_STANDARD 99)

# 显示详细编译命令
set(CMAKE_VERBOSE_MAKEFILE ON)

# 生成 compile_commands.json，供 clangd 代码补全使用
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_TOOLCHAIN_FILE
    ${CMAKE_CURRENT_LIST_DIR}/toolchain.cmake
    CACHE FILEPATH "Toolchain file"
)

# 使用工具链文件
if(NOT CMAKE_TOOLCHAIN_FILE)
    message(FATAL_ERROR "请使用 -DCMAKE_TOOLCHAIN_FILE=toolchain.cmake")
endif()

# 配置项目名称以及使用的语言
project(Label C CXX ASM)

# 两个可执行目标：分别对应 DW1000 和 DW3000（标签 TAG 节点）
set(TARGETS TAG_DW1000 TAG_DW3000)

# 启动文件（GCC 语法，独立于 Keil 的 MDK-ARM/startup_stm32f103xb.s）
set(STARTUP_FILE startup_stm32f103xb.s)

# ---- 公共源文件（与 DW 无关）----
# CubeMX 生成的 HAL 外设初始化 + 系统文件（main / gpio / spi / usart / adc / tim / dma / it / msp / system）
file(GLOB COMMON_CORE_SRC_FILES
    Core/Src/*.c
)

# UWB 应用层 —— 仅 TAG 角色；旧副本 instance.h / instance_tag.c 不参与编译（Phase 4 清理）
set(APP_SRC_FILES
    Core/application/dw_main.c
    Core/application/dw_instance_tag.c
    Core/application/dw_power.c
)

# 卡尔曼滤波
set(KALMAN_SRC_FILES
    Core/kalmanfilter/kalman.c
)

# SEGGER RTT 调试日志（厂家代码）。USART1 留给上位机协议，调试日志走 RTT。
file(GLOB RTT_SRC_FILES
    Core/Segger_RTT/*.c
)

# HAL 驱动（厂家代码，整目录收集，排除模板文件）
file(GLOB HAL_SRC_FILES
    Drivers/STM32F1xx_HAL_Driver/Src/*.c
)
list(FILTER HAL_SRC_FILES EXCLUDE REGEX "_template\\.c$")

# DW1000 / DW3000 厂家驱动 + 平台层（按目标分别编译，各自独立存放于 Core/Dw1000 与 Core/Dw3000）
# 注意：顶层旧的 Core/decadriver 与 Core/platform 已弃用，不在此引用。
file(GLOB DW1000_DRIVER_SRC_FILES
    Core/Dw1000/decadriver/*.c
    Core/Dw1000/platform/*.c
)
file(GLOB DW3000_DRIVER_SRC_FILES
    Core/Dw3000/decadriver/*.c
    Core/Dw3000/platform/*.c
)

# 厂家 / 第三方代码关闭所有警告（-w）
set_source_files_properties(
    ${HAL_SRC_FILES}
    ${DW1000_DRIVER_SRC_FILES}
    ${DW3000_DRIVER_SRC_FILES}
    ${RTT_SRC_FILES}
    PROPERTIES COMPILE_OPTIONS "-w"
)

# 添加两个可执行目标
add_executable(TAG_DW1000.elf
    ${STARTUP_FILE}
    ${COMMON_CORE_SRC_FILES}
    ${APP_SRC_FILES}
    ${KALMAN_SRC_FILES}
    ${RTT_SRC_FILES}
    ${HAL_SRC_FILES}
    ${DW1000_DRIVER_SRC_FILES}
)

add_executable(TAG_DW3000.elf
    ${STARTUP_FILE}
    ${COMMON_CORE_SRC_FILES}
    ${APP_SRC_FILES}
    ${KALMAN_SRC_FILES}
    ${RTT_SRC_FILES}
    ${HAL_SRC_FILES}
    ${DW3000_DRIVER_SRC_FILES}
)

# 公共头文件目录（不含 DW 厂家驱动 / 平台，那些按目标分别添加）
set(COMMON_INCLUDE_DIRS
    Core/Inc
    Core/Application
    Core/Compiler
    Core/kalmanfilter
    Core/Segger_RTT
    Drivers/STM32F1xx_HAL_Driver/Inc
    Drivers/STM32F1xx_HAL_Driver/Inc/Legacy
    Drivers/CMSIS/Device/ST/STM32F1xx/Include
    Drivers/CMSIS/Include
)

# 公共编译选项（Cortex-M3，无 FPU → 软浮点）
set(COMMON_COMPILE_OPTIONS
    -mcpu=cortex-m3
    -mthumb
    -ffunction-sections
    -fdata-sections
    -fdiagnostics-color=always
    -g
    -Wall
    -Wextra
    # 默认(build/，无 build type)用 -O2；Debug 构建(build-debug，-DCMAKE_BUILD_TYPE=Debug)用 -Og 便于单步调试
    $<$<NOT:$<CONFIG:Debug>>:-O2>
    $<$<CONFIG:Debug>:-Og>
)

# 公共链接选项
set(COMMON_LINK_OPTIONS
    -T${CMAKE_SOURCE_DIR}/STM32F103XX_FLASH.ld
    -mcpu=cortex-m3
    -mthumb
    -specs=nano.specs
    # 不再用 -specs=nosys.specs：系统调用桩由 Core/Src/syscalls.c + sysmem.c 自行提供，
    # 避免 libnosys 在链接期发出 "_close/_fstat/... is not implemented" 警告。
    -u _printf_float      # nano.specs 默认裁掉 printf/vsnprintf 的浮点支持，需显式拉入（卡尔曼距离 %f）
    -lc
    -lm
    -Wl,--cref,--gc-sections,--print-memory-usage
)

# 为两个目标分别配置
foreach(TGT IN LISTS TARGETS)
    set(ELF ${TGT}.elf)

    # 宏定义：HAL + 芯片选择（C8 与 CB 共用中容量器件头，仍为 STM32F103xB）
    target_compile_definitions(${ELF} PRIVATE
        USE_HAL_DRIVER
        STM32F103xB
    )

    # 头文件：公共 + 对应 DW 厂家驱动 / 平台
    if(TGT STREQUAL "TAG_DW1000")
        target_compile_definitions(${ELF} PRIVATE USE_DW1000)
        target_include_directories(${ELF} PRIVATE
            ${COMMON_INCLUDE_DIRS}
            Core/Dw1000/decadriver
            Core/Dw1000/platform
        )
    else()
        target_compile_definitions(${ELF} PRIVATE USE_DW3000)
        target_include_directories(${ELF} PRIVATE
            ${COMMON_INCLUDE_DIRS}
            Core/Dw3000/decadriver
            Core/Dw3000/platform
        )
    endif()

    # 编译选项
    target_compile_options(${ELF} PRIVATE ${COMMON_COMPILE_OPTIONS})

    # 链接选项（各自生成 .map 文件）
    target_link_options(${ELF} PRIVATE
        ${COMMON_LINK_OPTIONS}
        -Wl,-Map=${TGT}.map
    )

    # 打印可执行文件大小
    add_custom_command(TARGET ${ELF}
        POST_BUILD
        COMMAND arm-none-eabi-size ${ELF}
    )

    # 生成 hex 和 bin 文件
    add_custom_command(TARGET ${ELF}
        POST_BUILD
        COMMAND arm-none-eabi-objcopy -O ihex ${ELF} ${TGT}.hex
        COMMAND arm-none-eabi-objcopy -O binary ${ELF} ${TGT}.bin
    )

    # 清除 target 时顺带清除 bin hex map 文件
    set_target_properties(${ELF} PROPERTIES ADDITIONAL_CLEAN_FILES
        "${TGT}.bin;${TGT}.hex;${TGT}.map")
endforeach()
