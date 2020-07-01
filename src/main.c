/* SPDX-License-Identifier: MIT */

#include "sam.h"
#include "compiler.h"

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

#define LED_BUSY_PIN PIN_PA17
#define LED_TX_PIN PIN_PA19
#define LED_RX_PIN PIN_PA18

#define UART_TX_PIN PIN_PB08 // SERCOM4_PAD0
#define UART_RX_PIN PIN_PB09 // SERCOM4_PAD1

volatile uint32_t busy_led_blink_ms = 500;
volatile uint32_t tx_led_blink_ms = 0;
volatile uint32_t rx_led_blink_ms = 0;

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

    // Update SystemCoreClock since it is hard coded with asf4 and not correct
    // Init 1ms tick timer (samd SystemCoreClock may not correct)
    SystemCoreClock = CONF_CPU_FREQUENCY;
    SysTick_Config(CONF_CPU_FREQUENCY / 1000);

    // enable USB clock
    _pm_enable_bus_clock(PM_BUS_APBB, USB);
    _pm_enable_bus_clock(PM_BUS_AHB, USB);
    _gclk_enable_channel(USB_GCLK_ID, GCLK_CLKCTRL_GEN_GCLK0_Val);

    // enable UART clock
    _pm_enable_bus_clock(PM_BUS_APBC, SERCOM4);
    _gclk_enable_channel(SERCOM4_GCLK_ID_CORE, GCLK_CLKCTRL_GEN_GCLK0_Val);

    // enable SPI clock
    _pm_enable_bus_clock(PM_BUS_APBC, SERCOM0);
    _gclk_enable_channel(SERCOM0_GCLK_ID_CORE, GCLK_CLKCTRL_GEN_GCLK0_Val);

    // enable DMA clock
    _pm_enable_bus_clock(PM_BUS_APBB, DMAC);
    _pm_enable_bus_clock(PM_BUS_AHB, DMAC);
    // DMAC doesn't have a GCLK channel

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
}

/*****************************************************
 * Main system code
 ****************************************************/

uint8_t hdlc_rx_buffer[4096];
uint8_t hdlc_tx_buffer[70];

simplehdlc_context_t context;
ringbuf_t hdlc_tx_ringbuf;


void hdlc_rx_packet_callback(const uint8_t *payload, uint16_t len, void *user_ptr) {

}

void hdlc_tx_flush_buffer_callback(void *user_ptr) {
    const uint8_t padding = 0x7E;
    while (ringbuf_len(&hdlc_tx_ringbuf) < 64) {
        ringbuf_push(&hdlc_tx_ringbuf, &padding, 1);
    }

    uint8_t tmp[64];
    ringbuf_pop(&hdlc_tx_ringbuf, tmp, 64);

    tud_hid_report(0, tmp, 64);
}

void hdlc_tx_byte_callback(uint8_t byte, void *user_ptr) {
    ringbuf_push(&hdlc_tx_ringbuf, &byte, 1);

    if (ringbuf_len(&hdlc_tx_ringbuf) == 64) {
        hdlc_tx_flush_buffer_callback(NULL);
    }
}

simplehdlc_callbacks_t hdlc_callbacks = {
        .rx_packet_callback = &hdlc_rx_packet_callback,
        .tx_byte_callback = &hdlc_tx_byte_callback,
        .tx_flush_buffer_callback = &hdlc_tx_flush_buffer_callback,
};

// HID get report callback
uint16_t tud_hid_get_report_cb(uint8_t report_id, hid_report_type_t report_type, uint8_t *buffer, uint16_t reqlen) {
    // dummy implementation, should never be used anyway
    SEGGER_RTT_printf(0, "tud_hid_get_report_cb: report_id: %d, report_type: %d, reqlen: %d\n", report_id, report_type,
                      reqlen);

    for (int i = 0; i < reqlen; i++) {
        buffer[i] = SIMPLEHDLC_BOUNDARY_MARKER;
    }

    return reqlen;
}

// HID set report callback
void tud_hid_set_report_cb(uint8_t report_id, hid_report_type_t report_type, uint8_t const *buffer, uint16_t bufsize) {
    // unimplemented as this is not needed

    (void) buffer;

    SEGGER_RTT_printf(0, "tud_hid_set_report_cb: report_id: %d, report_type: %d, bufsize: %d\n", report_id, report_type,
                      bufsize);
}

volatile uint32_t system_ticks = 0;



int main() {
    clock_init();

    simplehdlc_init(&context, hdlc_rx_buffer, sizeof(hdlc_rx_buffer), &hdlc_callbacks, NULL);
    ringbuf_init(&hdlc_tx_ringbuf, hdlc_tx_buffer, sizeof(hdlc_tx_buffer));
    gpio_init();
    tusb_init();

    while(1) {
        tud_task(); // handle USB packets
    }
}

volatile bool led_state = false;
void SysTick_Handler (void)
{
    if (busy_led_blink_ms) {
        busy_led_blink_ms--;
        gpio_set_pin_level(LED_BUSY_PIN, led_state);
    } else {
        gpio_set_pin_level(LED_BUSY_PIN, true);
    }

    if (tx_led_blink_ms) {
        tx_led_blink_ms--;
        gpio_set_pin_level(LED_TX_PIN, led_state);
    } else {
        gpio_set_pin_level(LED_TX_PIN, true);
    }

    if (rx_led_blink_ms) {
        rx_led_blink_ms--;
        gpio_set_pin_level(LED_RX_PIN, led_state);
    } else {
        gpio_set_pin_level(LED_RX_PIN, true);
    }

    if ((system_ticks % 25) == 0) led_state = !led_state;
    system_ticks++;
}

void HardFault_Handler(void) {
    while(1);
}

void USB_Handler(void)
{
    tud_int_handler(0);
}