/* SPDX-License-Identifier: MIT */

#define POLYGLOT_VERSION "0.3.1"
#define POLYGLOT_HW "seeeduino-xiao"


#include "sam.h"
#include "compiler.h"
#include "utils.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "event_groups.h"

#include "hal/include/hal_gpio.h"
#include "hal/include/hal_init.h"
#include "hri/hri_nvmctrl_d21.h"

#include "hpl/gclk/hpl_gclk_base.h"
#include "hpl_pm_config.h"
#include "hpl/pm/hpl_pm_base.h"
#include "SEGGER_RTT.h"

#include "tusb.h"
#include "ringbuf.h"
#include "simplehdlc.h"

#include "simplecborrpc.h"
#include "rpc/rpc_api.h"

#include "cdc_usart.h"
#include "hid_rpc.h"
#include "dma.h"
#include "leds.h"

#include "pins.h"

/*****************************************************
 * Hardware setup
 ****************************************************/

/* Referenced GCLKs, should be initialized firstly */
#define _GCLK_INIT_1ST (1 << 0 | 1 << 1)

/* Not referenced GCLKs, initialized last */
#define _GCLK_INIT_LAST (~_GCLK_INIT_1ST)

void clock_init() {
    // Clock init ( follow hpl_init.c )
    hri_nvmctrl_set_CTRLB_RWS_bf(NVMCTRL, 2);

    _pm_init();
    _sysctrl_init_sources();
    _gclk_init_generators_by_fref(_GCLK_INIT_1ST);
    _sysctrl_init_referenced_generators();
    _gclk_init_generators_by_fref(_GCLK_INIT_LAST);


    // init GCLK2 for ADC (must be less than 2.1MHz with /4 prescaler inside ADC => less than 8.4MHz out of GCLK3)

    // Even though the datasheet says it can be clocked up to 48MHz in the "Maximum Clock Frequencies" table, it later
    //   says that the maximum clock rate is 2.1MHz for the ADC in the "Analog-to-Digital (ADC) characteristics" table.
    // Driving it at 48MHz doesn't work, the output is garbage (presumably because the ADC cannot complete the
    //   conversion fast enough)

    // 48MHz / 6 = 8MHz (which will be prescaled by 4 in the ADC => 2MHz)
    hri_gclk_write_GENDIV_reg(GCLK, GCLK_GENDIV_DIV(6) | GCLK_GENDIV_ID(2));
    hri_gclk_write_GENCTRL_reg(GCLK,
                               (0 << GCLK_GENCTRL_RUNSTDBY_Pos)
                               | (0 << GCLK_GENCTRL_DIVSEL_Pos)
                               | (1 << GCLK_GENCTRL_OE_Pos)
                               | (0 << GCLK_GENCTRL_OOV_Pos)
                               | (0 << GCLK_GENCTRL_IDC_Pos)
                               | (1 << GCLK_GENCTRL_GENEN_Pos)
                               | GCLK_GENCTRL_SRC_DFLL48M
                               | GCLK_GENCTRL_ID(2));


    // init GCLK3 for DAC (must be less than 350kHz according to the datasheet "Maximum Clock Frequencies" table)
    hri_gclk_write_GENDIV_reg(GCLK, GCLK_GENDIV_DIV(138) | GCLK_GENDIV_ID(3)); // 48MHz / 138 = ~347kHz
    hri_gclk_write_GENCTRL_reg(GCLK,
                               (0 << GCLK_GENCTRL_RUNSTDBY_Pos)
                               | (0 << GCLK_GENCTRL_DIVSEL_Pos)
                               | (1 << GCLK_GENCTRL_OE_Pos)
                               | (0 << GCLK_GENCTRL_OOV_Pos)
                               | (0 << GCLK_GENCTRL_IDC_Pos)
                               | (1 << GCLK_GENCTRL_GENEN_Pos)
                               | GCLK_GENCTRL_SRC_DFLL48M
                               | GCLK_GENCTRL_ID(3));

    // Update SystemCoreClock since it is hard coded with asf4 and not correct
    // Init 1ms tick timer (samd SystemCoreClock may not correct)
    SystemCoreClock = CONF_CPU_FREQUENCY;

    NVIC_SetPriority(USB_IRQn, 1);

    // enable USB clock (GCLK0)
    _pm_enable_bus_clock(PM_BUS_APBB, USB);
    _pm_enable_bus_clock(PM_BUS_AHB, USB);
    _gclk_enable_channel(USB_GCLK_ID, GCLK_CLKCTRL_GEN_GCLK0_Val);

    // enable UART clock (GCLK0)
    _pm_enable_bus_clock(PM_BUS_APBC, SERCOM4);
    _gclk_enable_channel(SERCOM4_GCLK_ID_CORE, GCLK_CLKCTRL_GEN_GCLK0_Val);

    // enable I2C clock (GCLK0)
    _pm_enable_bus_clock(PM_BUS_APBC, SERCOM2);
    _gclk_enable_channel(SERCOM2_GCLK_ID_CORE, GCLK_CLKCTRL_GEN_GCLK0_Val);

    // enable SPI clock (GCLK0)
    _pm_enable_bus_clock(PM_BUS_APBC, SERCOM0);
    _gclk_enable_channel(SERCOM0_GCLK_ID_CORE, GCLK_CLKCTRL_GEN_GCLK0_Val);

    // enable TCC0 clock (GCLK0)
    _pm_enable_bus_clock(PM_BUS_APBC, TCC0);
    _gclk_enable_channel(TCC0_GCLK_ID, GCLK_CLKCTRL_GEN_GCLK0_Val);

    // enable TCC1 clock (GCLK0)
    _pm_enable_bus_clock(PM_BUS_APBC, TCC1);
    _gclk_enable_channel(TCC1_GCLK_ID, GCLK_CLKCTRL_GEN_GCLK0_Val);

    // enable ADC clock (GCLK2)
    _pm_enable_bus_clock(PM_BUS_APBC, ADC);
    _gclk_enable_channel(ADC_GCLK_ID, GCLK_CLKCTRL_GEN_GCLK2_Val);

    // enable DAC clock (GCLK3)
    _pm_enable_bus_clock(PM_BUS_APBC, DAC);
    _gclk_enable_channel(DAC_GCLK_ID, GCLK_CLKCTRL_GEN_GCLK3_Val);

    // enable DMA clock
    _pm_enable_bus_clock(PM_BUS_APBB, DMAC);
    _pm_enable_bus_clock(PM_BUS_AHB, DMAC);
    // DMAC doesn't have a GCLK channel
}

void gpio_init() {
    // USB
    gpio_set_pin_direction(PIN_PA24, GPIO_DIRECTION_OUT);
    gpio_set_pin_level(PIN_PA24, false);
    gpio_set_pin_pull_mode(PIN_PA24, GPIO_PULL_OFF);

    gpio_set_pin_direction(PIN_PA25, GPIO_DIRECTION_OUT);
    gpio_set_pin_level(PIN_PA25, false);
    gpio_set_pin_pull_mode(PIN_PA25, GPIO_PULL_OFF);

    gpio_set_pin_function(PIN_PA24, PINMUX_PA24G_USB_DM);
    gpio_set_pin_function(PIN_PA25, PINMUX_PA25G_USB_DP);

    // LEDs (active low)
    gpio_set_pin_direction(LED_BUSY_PIN, GPIO_DIRECTION_OUT);
    gpio_set_pin_level(LED_BUSY_PIN, true);
    gpio_set_pin_pull_mode(LED_BUSY_PIN, GPIO_PULL_OFF);

    gpio_set_pin_direction(LED_TX_PIN, GPIO_DIRECTION_OUT);
    gpio_set_pin_level(LED_TX_PIN, true);
    gpio_set_pin_pull_mode(LED_TX_PIN, GPIO_PULL_OFF);

    gpio_set_pin_direction(LED_RX_PIN, GPIO_DIRECTION_OUT);
    gpio_set_pin_level(LED_RX_PIN, true);
    gpio_set_pin_pull_mode(LED_RX_PIN, GPIO_PULL_OFF);

    // UART
    gpio_set_pin_function(UART_TX_PIN, PINMUX_PB08D_SERCOM4_PAD0);
    gpio_set_pin_function(UART_RX_PIN, PINMUX_PB09D_SERCOM4_PAD1);
    gpio_set_pin_pull_mode(UART_RX_PIN, GPIO_PULL_UP);

    // I2C
    gpio_set_pin_direction(I2C_SDA_PIN, GPIO_DIRECTION_IN);
    gpio_set_pin_direction(I2C_SCL_PIN, GPIO_DIRECTION_IN);
    gpio_set_pin_pull_mode(I2C_SDA_PIN, GPIO_PULL_UP);
    gpio_set_pin_pull_mode(I2C_SCL_PIN, GPIO_PULL_UP);
    gpio_set_pin_function(I2C_SDA_PIN, PINMUX_PA08D_SERCOM2_PAD0);
    gpio_set_pin_function(I2C_SCL_PIN, PINMUX_PA09D_SERCOM2_PAD1);

    // SPI
    gpio_set_pin_function(SPI_SCK_PIN, PINMUX_PA07D_SERCOM0_PAD3);
    gpio_set_pin_function(SPI_MOSI_PIN, PINMUX_PA06D_SERCOM0_PAD2);
    gpio_set_pin_function(SPI_MISO_PIN, PINMUX_PA05D_SERCOM0_PAD1);
}

/*****************************************************
 * RPC interface
 ****************************************************/

rpc_error_t
rpc_polyglot_version(const CborValue *args_iterator, CborEncoder *result, const char **error_msg, void *user_ptr) {
    const char *version = POLYGLOT_VERSION;
    cbor_encode_text_string(result, version, strlen(version));

    return RPC_OK;
}

rpc_error_t
rpc_polyglot_hw(const CborValue *args_iterator, CborEncoder *result, const char **error_msg, void *user_ptr) {
    const char *hw = POLYGLOT_HW;
    cbor_encode_text_string(result, hw, strlen(hw));

    return RPC_OK;
}

// static task for usbd
// Increase stack size when debug log is enabled
#if CFG_TUSB_DEBUG
#define USBD_STACK_SIZE     (3*configMINIMAL_STACK_SIZE)
#else
#define USBD_STACK_SIZE     (3*configMINIMAL_STACK_SIZE/2)
#endif

StackType_t  usb_device_stack[USBD_STACK_SIZE];
StaticTask_t usb_device_taskdef;
void usb_device_task(void* param)
{
    (void) param;

    // RTOS forever loop
    while (1)
    {
        // tinyusb device task
        tud_task();
    }
}

int main() {
    clock_init();

    gpio_init();
    tusb_init();

    (void) xTaskCreateStatic( usb_device_task,
            "usbd",
            USBD_STACK_SIZE,
            NULL,
            configMAX_PRIORITIES-1,
            usb_device_stack,
            &usb_device_taskdef);

    led_blink_init();

    dma_init();

    start_cdc_usart_task();
    start_hid_rpc_task();

    vTaskStartScheduler();
    NVIC_SystemReset();
    return 0;
}

void HardFault_Handler(void) {
    while(1);
}

void USB_Handler(void)
{
    tud_int_handler(0);
}

ssize_t _write (int fd, const void * buf, size_t count) {
    while(1);
}

ssize_t _read (int fd, void * buf, size_t count) {
    while(1);
}

void vAssertCalled(const char * pcFile, unsigned long ulLine) {
    while(1);
}
