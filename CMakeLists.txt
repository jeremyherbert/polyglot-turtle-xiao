cmake_minimum_required(VERSION 3.15)
set(CMAKE_TOOLCHAIN_FILE ${CMAKE_CURRENT_LIST_DIR}/cmake/arm-gcc-toolchain.cmake)
project(polyglot_turtle_xiao_proj)
set(CMAKE_VERBOSE_MAKEFILE ON)

enable_language(ASM)

set(CMAKE_C_STANDARD 99)
set(CMAKE_EXECUTABLE_SUFFIX ".elf")

include(cmake/utils.cmake)

set(CPU_FLAGS "-flto -mthumb -mabi=aapcs-linux -mcpu=cortex-m0plus -fdata-sections -ffunction-sections -ffreestanding -nostartfiles -lc -lnosys --specs=nano.specs")

set(GCC_FLAGS "-fsingle-precision-constant -fno-strict-aliasing -Wdouble-promotion -Wno-endif-labels
                -Wall -Wextra -Werror-implicit-function-declaration -Wfatal-errors -Wfloat-equal -Wundef
                -Wshadow -Wwrite-strings -Wsign-compare -Wmissing-format-attribute -Wno-deprecated-declarations
                -Wnested-externs -Wunreachable-code -Wno-error=lto-type-mismatch -Wno-unused-parameter
                -Wno-error=float-equal -Wno-error=double-promotion")

add_definitions(${CPU_FLAGS} ${GCC_FLAGS} -Og -g
        -D__SAMD21G18A__ -DCONF_DFLL_OVERWRITE_CALIBRATION=0 -DCFG_TUSB_MCU=OPT_MCU_SAMD21)

set(LINKER_FILE ${CMAKE_CURRENT_LIST_DIR}/libs/tinyusb/hw/bsp/seeeduino_xiao/samd21g18a_flash.ld)
string(APPEND CMAKE_EXE_LINKER_FLAGS " ${CPU_FLAGS} -Wl,-T ${LINKER_FILE}")
string(APPEND CMAKE_EXE_LINKER_FLAGS " -Wl,--gc-sections -Wl,-Map=output.map -fshort-enums")
string(APPEND CMAKE_EXE_LINKER_FLAGS " -Wl,--undefined=vTaskSwitchContext") # workaround for FreeRTOS/linker issue

set(RTT_FILES
        libs/rtt/SEGGER_RTT.c
        libs/rtt/SEGGER_RTT_printf.c)

set(TINYUSB_FILES
        libs/tinyusb/src/class/cdc/cdc_device.c
        libs/tinyusb/src/class/hid/hid_device.c
        libs/tinyusb/src/common/tusb_fifo.c
        libs/tinyusb/src/device/usbd.c
        libs/tinyusb/src/device/usbd_control.c
        libs/tinyusb/src/portable/microchip/samd/dcd_samd.c
        libs/tinyusb/src/tusb.c)

set(FREERTOS_ROOT libs/tinyusb/lib/FreeRTOS/FreeRTOS/Source)
set(FREERTOS_FILES
        ${FREERTOS_ROOT}/portable/GCC/ARM_CM0/port.c
        ${FREERTOS_ROOT}/croutine.c
        ${FREERTOS_ROOT}/event_groups.c
        ${FREERTOS_ROOT}/list.c
        ${FREERTOS_ROOT}/queue.c
        ${FREERTOS_ROOT}/stream_buffer.c
        ${FREERTOS_ROOT}/tasks.c
        ${FREERTOS_ROOT}/timers.c
        )

set(HAL_SRC_DIR libs/tinyusb/hw/mcu/microchip/samd/asf4/samd21)
set(HAL_FILES
        ${HAL_SRC_DIR}/gcc/gcc/startup_samd21.c
        ${HAL_SRC_DIR}/gcc/system_samd21.c

        ${HAL_SRC_DIR}/hpl/gclk/hpl_gclk.c
        ${HAL_SRC_DIR}/hpl/pm/hpl_pm.c
        ${HAL_SRC_DIR}/hpl/sysctrl/hpl_sysctrl.c

        ${HAL_SRC_DIR}/hal/src/hal_atomic.c

        ${HAL_SRC_DIR}/hal/utils/src/utils_syscalls.c
        )

set(SIMPLEHDLC_FILES
        libs/simplehdlc/simplehdlc.c
        libs/simplehdlc/simplehdlc_crc32.c
        )

set(SIMPLECBORRPC_FILES
        libs/simplecborrpc/simplecborrpc.c
        libs/simplecborrpc/default_functions.c
        libs/simplecborrpc/tinycbor/src/cborencoder.c
        libs/simplecborrpc/tinycbor/src/cborerrorstrings.c
        libs/simplecborrpc/tinycbor/src/cborparser.c
        libs/simplecborrpc/tinycbor/src/cborvalidation.c
        )

set(SRC_FILES
        libs/ringbuf/ringbuf.c

        src/rpc/rpc_api.c

        src/main.c
        src/descriptors.c
        src/dma.c
        src/rpc_gpio.c
        src/rpc_i2c.c
        src/rpc_spi.c
        src/rpc_pwm.c
        src/rpc_dac.c
        src/hid_rpc.c
        src/leds.c
        src/pins.c

        src/freertos_hook.c src/cdc_usart.c src/cdc_usart.h)

include_directories(
        src
        src/rpc

        libs/rtt
        libs/ringbuf

        libs/tinyusb/hw
        libs/tinyusb/src

        libs/simplehdlc
        libs/simplecborrpc
        libs/simplecborrpc/tinycbor/src

        ${FREERTOS_ROOT}/include
        ${FREERTOS_ROOT}/portable/GCC/ARM_CM0

        ${HAL_SRC_DIR}/
        ${HAL_SRC_DIR}/config
        ${HAL_SRC_DIR}/include
        ${HAL_SRC_DIR}/hal/include
        ${HAL_SRC_DIR}/hal/utils/include
        ${HAL_SRC_DIR}/hpl/pm/
        ${HAL_SRC_DIR}/hpl/port
        ${HAL_SRC_DIR}/hri
        ${HAL_SRC_DIR}/CMSIS/Include
)

add_executable(polyglot_turtle_xiao ${SRC_FILES} ${SIMPLEHDLC_FILES} ${SIMPLECBORRPC_FILES} ${RTT_FILES} ${TINYUSB_FILES} ${FREERTOS_FILES} ${HAL_FILES})

firmware_size(polyglot_turtle_xiao)
generate_object(polyglot_turtle_xiao .bin binary)

add_custom_command( OUTPUT ${CMAKE_CURRENT_LIST_DIR}/src/rpc/rpc_api.c ${CMAKE_CURRENT_LIST_DIR}/src/rpc/rpc_api.h
        COMMAND python3 ${CMAKE_CURRENT_LIST_DIR}/src/rpc/rpc_gen.py
        WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/src/rpc/
        DEPENDS ${CMAKE_CURRENT_LIST_DIR}/src/rpc/rpc_gen.py
        VERBATIM)