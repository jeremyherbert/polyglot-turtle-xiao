/* SPDX-License-Identifier: MIT */

#include "sam.h"
#include "compiler.h"
#include "utils.h"

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

#define LED_BUSY_PIN PIN_PA17
#define LED_TX_PIN PIN_PA19
#define LED_RX_PIN PIN_PA18

#define UART_TX_PIN PIN_PB08 // SERCOM4_PAD0
#define UART_RX_PIN PIN_PB09 // SERCOM4_PAD1

#define I2C_SDA_PIN PIN_PA08
#define I2C_SCL_PIN PIN_PA09

#define SPI_SCK_PIN PIN_PA07
#define SPI_MOSI_PIN PIN_PA06
#define SPI_MISO_PIN PIN_PA05

#define GPIO_PIN_COUNT 4
static const size_t gpio_pin_map[GPIO_PIN_COUNT] = {
        PIN_PA02,
        PIN_PA04,
        PIN_PA10,
        PIN_PA11
};

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

    NVIC_SetPriority(SysTick_IRQn, 2);
    NVIC_SetPriority(USB_IRQn, 1);

    SysTick_Config(CONF_CPU_FREQUENCY / 1000);

    // enable USB clock
    _pm_enable_bus_clock(PM_BUS_APBB, USB);
    _pm_enable_bus_clock(PM_BUS_AHB, USB);
    _gclk_enable_channel(USB_GCLK_ID, GCLK_CLKCTRL_GEN_GCLK0_Val);

    // enable UART clock
    _pm_enable_bus_clock(PM_BUS_APBC, SERCOM4);
    _gclk_enable_channel(SERCOM4_GCLK_ID_CORE, GCLK_CLKCTRL_GEN_GCLK0_Val);

    // enable I2C clock
    _pm_enable_bus_clock(PM_BUS_APBC, SERCOM2);
    _gclk_enable_channel(SERCOM2_GCLK_ID_CORE, GCLK_CLKCTRL_GEN_GCLK0_Val);

    // enable SPI clock
    _pm_enable_bus_clock(PM_BUS_APBC, SERCOM0);
    _gclk_enable_channel(SERCOM0_GCLK_ID_CORE, GCLK_CLKCTRL_GEN_GCLK0_Val);

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
    gpio_set_pin_function(I2C_SDA_PIN, PINMUX_PA08D_SERCOM2_PAD0);
    gpio_set_pin_function(I2C_SCL_PIN, PINMUX_PA09D_SERCOM2_PAD1);
    gpio_set_pin_pull_mode(I2C_SDA_PIN, GPIO_PULL_UP);
    gpio_set_pin_pull_mode(I2C_SCL_PIN, GPIO_PULL_UP);

    // SPI
    gpio_set_pin_function(SPI_SCK_PIN, PINMUX_PA07D_SERCOM0_PAD3);
    gpio_set_pin_function(SPI_MOSI_PIN, PINMUX_PA06D_SERCOM0_PAD2);
    gpio_set_pin_function(SPI_MISO_PIN, PINMUX_PA05D_SERCOM0_PAD1);
}

/*****************************************************
 * RPC interface
 ****************************************************/

rpc_error_t
rpc_echo(const CborValue *args_iterator, CborEncoder *result, const char **error_msg, void *user_ptr) {
    size_t string_length = 0;
    cbor_value_get_string_length(args_iterator, &string_length);
    if (string_length > 64) {
        *error_msg = "String too long";
        return RPC_ERROR_INVALID_ARGS;
    }

    char echobuf[64];
    size_t echobuflen = sizeof(echobuf);
    cbor_value_copy_text_string(args_iterator, echobuf, &echobuflen, NULL);
    cbor_encode_text_string(result, echobuf, echobuflen);

    return RPC_OK;
}

rpc_error_t
rpc_gpio_set_dir(const CborValue *args_iterator, CborEncoder *result, const char **error_msg, void *user_ptr) {
    CborValue private_args_it = *args_iterator;
    uint64_t pin_number;
    uint64_t direction;

    cbor_value_get_uint64(&private_args_it, &pin_number);
    cbor_value_advance(&private_args_it);
    cbor_value_get_uint64(&private_args_it, &direction);

    if (pin_number >= GPIO_PIN_COUNT) {
        *error_msg = "Invalid pin";
        return RPC_ERROR_INVALID_ARGS;
    }

    if (direction > 2) {
        *error_msg = "Invalid direction";
        return RPC_ERROR_INVALID_ARGS;
    }

    switch(direction) {
        case 0:
            gpio_set_pin_direction(gpio_pin_map[pin_number], GPIO_DIRECTION_IN);
            break;
        case 1:
            gpio_set_pin_direction(gpio_pin_map[pin_number], GPIO_DIRECTION_OUT);
            break;
        case 2:
            gpio_set_pin_direction(gpio_pin_map[pin_number], GPIO_DIRECTION_OFF);
            break;
        default:
            return RPC_ERROR_INTERNAL_ERROR; // this should never occur
    }

    cbor_encode_null(result);
    return RPC_OK;
}

rpc_error_t
rpc_gpio_set_pull(const CborValue *args_iterator, CborEncoder *result, const char **error_msg, void *user_ptr) {
    CborValue private_args_it = *args_iterator;
    uint64_t pin_number;
    uint64_t pull_type;

    cbor_value_get_uint64(&private_args_it, &pin_number);
    cbor_value_advance(&private_args_it);
    cbor_value_get_uint64(&private_args_it, &pull_type);

    if (pin_number >= GPIO_PIN_COUNT) {
        *error_msg = "Invalid pin";
        return RPC_ERROR_INVALID_ARGS;
    }

    if (pull_type > 2) {
        *error_msg = "Invalid pull type";
        return RPC_ERROR_INVALID_ARGS;
    }

    switch(pull_type) {
        case 0:
            gpio_set_pin_pull_mode(gpio_pin_map[pin_number], GPIO_PULL_OFF);
            break;
        case 1:
            gpio_set_pin_pull_mode(gpio_pin_map[pin_number], GPIO_PULL_UP);
            break;
        case 2:
            gpio_set_pin_pull_mode(gpio_pin_map[pin_number], GPIO_PULL_DOWN);
            break;
        default:
            return RPC_ERROR_INTERNAL_ERROR; // this should never occur
    }

    cbor_encode_null(result);
    return RPC_OK;
}

rpc_error_t
rpc_gpio_set_level(const CborValue *args_iterator, CborEncoder *result, const char **error_msg, void *user_ptr) {
    CborValue private_args_it = *args_iterator;

    uint64_t pin_number;
    bool value;

    cbor_value_get_uint64(&private_args_it, &pin_number);
    cbor_value_advance(&private_args_it);
    cbor_value_get_boolean(&private_args_it, &value);

    if (pin_number >= GPIO_PIN_COUNT) {
        *error_msg = "Invalid pin";
        return RPC_ERROR_INVALID_ARGS;
    } else {
        gpio_set_pin_level(gpio_pin_map[pin_number], value);
    }

    cbor_encode_null(result);
    return RPC_OK;
}

rpc_error_t
rpc_gpio_get_level(const CborValue *args_iterator, CborEncoder *result, const char **error_msg, void *user_ptr) {
    uint64_t pin_number;
    cbor_value_get_uint64(args_iterator, &pin_number);

    if (pin_number >= GPIO_PIN_COUNT) {
        *error_msg = "Invalid pin";
        return RPC_ERROR_INVALID_ARGS;
    }

    bool read_value = gpio_get_pin_level(gpio_pin_map[pin_number]);
    cbor_encode_boolean(result, read_value);

    return RPC_OK;
}

rpc_error_t
rpc_i2c_exchange(const CborValue *args_iterator, CborEncoder *result, const char **error_msg, void *user_ptr) {

    return RPC_OK;
}

rpc_error_t
rpc_spi_exchange(const CborValue *args_iterator, CborEncoder *result, const char **error_msg, void *user_ptr) {

    return RPC_OK;
}

/*****************************************************
 * HID/HDLC interface
 ****************************************************/

uint8_t hdlc_rx_buffer[4096];
uint8_t hdlc_tx_buffer[4096];
uint8_t rpc_output_buffer[4096];

simplehdlc_context_t hdlc_context;
ringbuf_t hdlc_tx_ringbuf;


void hid_flush() {
    if (tud_hid_ready() && ringbuf_len(&hdlc_tx_ringbuf)) {
        uint8_t tmp[64];
        size_t count = ringbuf_pop(&hdlc_tx_ringbuf, tmp, 64);

        // add padding
        for (size_t i=count; i<64; i++) {
            tmp[i] = SIMPLEHDLC_BOUNDARY_MARKER;
        }
        tud_hid_report(0, tmp, 64);
        SEGGER_RTT_printf(0, "[RPC] HDLC partial flush, %u bytes\n", count);
    }
}

void hdlc_rx_packet_callback(const uint8_t *payload, uint16_t len, void *user_ptr) {
    SEGGER_RTT_printf(0, "[RPC] HDLC decode success!\n");
    size_t output_length = sizeof(rpc_output_buffer);
    rpc_error_t err = execute_rpc_call(rpc_function_table, SIMPLECBORRPC_FUNCTION_COUNT, payload, len, rpc_output_buffer, &output_length, NULL);
    SEGGER_RTT_printf(0, "[RPC] returning %u bytes, return code: %d\n", output_length, (int) err);
    simplehdlc_encode_to_callback(&hdlc_context, rpc_output_buffer, output_length, true);
}

void hdlc_tx_flush_buffer_callback(void *user_ptr) {
    // do nothing
}

void hdlc_tx_byte_callback(uint8_t byte, void *user_ptr) {
    ringbuf_push(&hdlc_tx_ringbuf, &byte, 1);

    if (ringbuf_len(&hdlc_tx_ringbuf) >= 64 && tud_hid_ready()) {
        hid_flush();
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

    SEGGER_RTT_printf(0, "[HID] got report from host: report_id: %d, report_type: %d, bufsize: %d\n", report_id, report_type,
                      bufsize);

    simplehdlc_parse(&hdlc_context, buffer, bufsize);
}

/*****************************************************
 * UART/CDC
 ****************************************************/

uint8_t usart_to_pc_ringbuf_buffer[512];
ringbuf_t usart_to_pc_ringbuf;

volatile cdc_line_coding_t cdc_line_coding = {
        .bit_rate = 9600,
        .stop_bits = 0,
        .parity = 0,
        .data_bits = 8
};
volatile bool cdc_line_coding_changed = false;

volatile bool usart_parity_enabled = false;

COMPILER_ALIGNED(16)
static uint8_t dma_buffer[64];

COMPILER_ALIGNED(16)
static DmacDescriptor _descriptor_section[DMAC_CH_NUM] SECTION_DMAC_DESCRIPTOR;

COMPILER_ALIGNED(16)
static DmacDescriptor _descriptor_writeback_section[DMAC_CH_NUM] SECTION_DMAC_DESCRIPTOR;

// USART interrupt
void SERCOM4_Handler() {
    if (hri_sercomusart_get_interrupt_RXC_bit(SERCOM4)) {
        if (hri_sercomusart_read_STATUS_reg(SERCOM4)
            & (SERCOM_USART_STATUS_PERR | SERCOM_USART_STATUS_FERR | SERCOM_USART_STATUS_BUFOVF
               | SERCOM_USART_STATUS_ISF
               | SERCOM_USART_STATUS_COLL)) {
            hri_sercomusart_clear_STATUS_reg(SERCOM4, SERCOM_USART_STATUS_MASK);
            return;
        }
        uint8_t tmp = hri_sercomusart_read_DATA_reg(SERCOM4);
//        if (usart_parity_enabled) tmp >>= 1;
        ringbuf_push(&usart_to_pc_ringbuf, &tmp, 1);
        rx_led_blink_ms = 100;
    }
}

void usart_update_configuration_from_line_coding() {
    uint32_t ctrla_tmp = SERCOM_USART_CTRLA_DORD | // send LSB first (this is the normal way for UART)
                         SERCOM_USART_CTRLA_RXPO(1) | // RX uses PAD1
                         SERCOM_USART_CTRLA_TXPO(0) | // TX uses PAD0
                         SERCOM_USART_CTRLA_MODE(1); // use internal clock

    uint32_t ctrlb_tmp = SERCOM_USART_CTRLB_RXEN | SERCOM_USART_CTRLB_TXEN;

    uint32_t baud_tmp;
    //uint32_t fp_tmp;

    // if parity == 0, no parity so don't set any bits

    // if parity == 1 or 2, SERCOM_USART_CTRLB_PMODE should be set to 0 (default) but enable parity in CTRLA
    if (cdc_line_coding.parity != 0) {
        ctrla_tmp |= SERCOM_USART_CTRLA_FORM(1); // enable parity
        usart_parity_enabled = true;

        // if parity == 2, set SERCOM_USART_CTRLB_PMODE to enable odd parity
        if (cdc_line_coding.parity == 2) ctrlb_tmp |= SERCOM_USART_CTRLB_PMODE;
    } else {
        usart_parity_enabled = false;
    }

    // if stop_bits == 0, one stop bit
    // otherwise set SERCOM_USART_CTRLB_SBMODE for 2 stop bits
    if (cdc_line_coding.stop_bits == 2) ctrlb_tmp |= SERCOM_USART_CTRLB_SBMODE;

    switch (cdc_line_coding.data_bits) {
        default:
        case 8:
            break; // bits are set to 0

        case 9:
            ctrlb_tmp |= SERCOM_USART_CTRLB_CHSIZE(1);
            break;
        case 5:
            ctrlb_tmp |= SERCOM_USART_CTRLB_CHSIZE(5);
            break;
        case 6:
            ctrlb_tmp |= SERCOM_USART_CTRLB_CHSIZE(6);
            break;
        case 7:
            ctrlb_tmp |= SERCOM_USART_CTRLB_CHSIZE(7);
            break;
    }

    const uint64_t refclk = 48000000ULL;

    if (cdc_line_coding.bit_rate < 10000) {
        ctrla_tmp |= SERCOM_USART_CTRLA_SAMPR(1); // 16x oversampling with fractional baud
        baud_tmp = (refclk / (16ULL * cdc_line_coding.bit_rate));
        // fp = 0
    } else if (cdc_line_coding.bit_rate < 500000) {
        ctrla_tmp |= SERCOM_USART_CTRLA_SAMPR(2); // 8x oversampling with arithmetic baud
        baud_tmp = 65536ULL - (65536ULL * 8ULL * cdc_line_coding.bit_rate) / refclk;
    } else {
        ctrla_tmp |= SERCOM_USART_CTRLA_SAMPR(4); // 3x oversampling with arithmetic baud
        baud_tmp = 65536ULL - (65536ULL * 3ULL * cdc_line_coding.bit_rate) / refclk;
    }

    hri_sercomusart_write_BAUD_reg(SERCOM4, baud_tmp);
    hri_sercomusart_write_CTRLA_reg(SERCOM4, ctrla_tmp);
    hri_sercomusart_write_CTRLB_reg(SERCOM4, ctrlb_tmp);
}

void usart_init() {
    ringbuf_init(&usart_to_pc_ringbuf, usart_to_pc_ringbuf_buffer, sizeof(usart_to_pc_ringbuf_buffer));

    // USART is via SERCOM4
    hri_sercomusart_wait_for_sync(SERCOM4, SERCOM_USART_SYNCBUSY_SWRST);
    hri_sercomusart_set_CTRLA_SWRST_bit(SERCOM4);
    hri_sercomusart_wait_for_sync(SERCOM4, SERCOM_USART_SYNCBUSY_SWRST);

    usart_update_configuration_from_line_coding();

//    hri_sercomusart_write_BAUD_FRAC_BAUD_bf(SERCOM4, 312);
//    hri_sercomusart_write_BAUD_FRAC_FP_bf(SERCOM4, 3); // 9600 baud

    hri_sercomusart_write_INTEN_RXC_bit(SERCOM4, true); // enable RX interrupt
    NVIC_EnableIRQ(SERCOM4_IRQn);


    // enable SERCOM peripheral
    hri_sercomusart_wait_for_sync(SERCOM4, SERCOM_USART_SYNCBUSY_SWRST);
    hri_sercomusart_set_CTRLA_ENABLE_bit(SERCOM4);
    hri_sercomusart_wait_for_sync(SERCOM4, SERCOM_USART_SYNCBUSY_SWRST);

    // clear all DMA descriptors
    memset(_descriptor_section, 0, sizeof(_descriptor_section));
    memset(_descriptor_writeback_section, 0, sizeof(_descriptor_writeback_section));

    // reset DMA
    hri_dmac_clear_CTRL_DMAENABLE_bit(DMAC);
    hri_dmac_clear_CTRL_CRCENABLE_bit(DMAC);
    hri_dmac_set_CHCTRLA_SWRST_bit(DMAC);

    // enable only priority 0 as we are only doing a single request
    hri_dmac_write_CTRL_LVLEN0_bit(DMAC, true);

    // set up DMA descriptor addresses
    hri_dmac_write_BASEADDR_reg(DMAC, (uint32_t)_descriptor_section);
    hri_dmac_write_WRBADDR_reg(DMAC, (uint32_t)_descriptor_writeback_section);

    // set DMA target as the USART data register
    hri_dmacdescriptor_write_DSTADDR_reg(&_descriptor_section[0], (uint32_t) &(SERCOM4->USART.DATA));

    // trigger a beat if USART TX is complete
    hri_dmac_write_CHID_reg(DMAC, 0);
    hri_dmac_write_CHCTRLB_TRIGACT_bf(DMAC, DMAC_CHCTRLB_TRIGACT_BEAT_Val);
    hri_dmac_write_CHCTRLB_TRIGSRC_bf(DMAC, SERCOM4_DMAC_ID_TX);

    hri_dmacdescriptor_set_BTCTRL_SRCINC_bit(&_descriptor_section[0]); // increment source address
    hri_dmacdescriptor_set_BTCTRL_STEPSEL_bit(&_descriptor_section[0]); // stepsize applies to SRC
    hri_dmacdescriptor_set_BTCTRL_VALID_bit(&_descriptor_section[0]); // mark descriptor as valid

    // start DMA
    hri_dmac_set_CTRL_DMAENABLE_bit(DMAC);
}

void tud_cdc_line_coding_cb(uint8_t itf, cdc_line_coding_t const* p_line_coding) {
    if (p_line_coding->parity <= 2) {
        // 3 and 4 (mark/space) are not supported
        cdc_line_coding.parity = p_line_coding->parity;
    }
    if (p_line_coding->data_bits >= 5 && p_line_coding->data_bits <= 8) {
        cdc_line_coding.data_bits = p_line_coding->data_bits;
    }
    if (p_line_coding->stop_bits == 0 || p_line_coding->stop_bits == 2) {
        cdc_line_coding.stop_bits = p_line_coding->stop_bits;
    }
    if (p_line_coding->bit_rate >= 1200 && p_line_coding->bit_rate <= 1000000) {
        cdc_line_coding.bit_rate = p_line_coding->bit_rate;
    }

    cdc_line_coding_changed = true;
}


uint8_t tmp_tx[] = "ABCDEF";
void cdc_process() {
    uint8_t buf[64];

    __disable_irq();
    size_t len = ringbuf_pop(&usart_to_pc_ringbuf, buf, 64);
    __enable_irq();

    if (tud_cdc_connected()) {
        if (len) { // if data needs to be sent to CDC
            tud_cdc_write(buf, len);
            tud_cdc_write_flush();
        }

        if (!hri_dmac_get_CHCTRLA_ENABLE_bit(DMAC) && cdc_line_coding_changed) {
            // enable SERCOM peripheral
            hri_sercomusart_clear_CTRLA_ENABLE_bit(SERCOM4);
            __disable_irq();
            hri_sercomusart_wait_for_sync(SERCOM4, SERCOM_USART_SYNCBUSY_ENABLE);

            usart_update_configuration_from_line_coding();
            cdc_line_coding_changed = false;
            __enable_irq();

            hri_sercomusart_set_CTRLA_ENABLE_bit(SERCOM4);
            hri_sercomusart_wait_for_sync(SERCOM4, SERCOM_USART_SYNCBUSY_ENABLE);
        }

        if (tud_cdc_available()) { // if data has been received from CDC
            hri_dmac_write_CHID_reg(DMAC, 0);
            if (!hri_dmac_get_CHCTRLA_ENABLE_bit(DMAC)) { // is DMA transaction complete?

                uint32_t tx_len = tud_cdc_read(dma_buffer, 64);

                if (tx_len) {
                    // set pointer to the final beat of the transaction
                    hri_dmacdescriptor_write_SRCADDR_reg(&_descriptor_section[0], (uint32_t)(dma_buffer+tx_len));

                    // set beat count
                    hri_dmacdescriptor_write_BTCNT_reg(&_descriptor_section[0], (uint16_t)tx_len);

                    // enable DMA channel
                    hri_dmac_set_CHCTRLA_ENABLE_bit(DMAC);
                    tx_led_blink_ms = 100;
                }
            }
        }
    }
}

/*****************************************************
 * Main system code
 ****************************************************/

volatile uint32_t system_ticks = 0;

int main() {
    clock_init();

    simplehdlc_init(&hdlc_context, hdlc_rx_buffer, sizeof(hdlc_rx_buffer), &hdlc_callbacks, NULL);
    ringbuf_init(&hdlc_tx_ringbuf, hdlc_tx_buffer, sizeof(hdlc_tx_buffer));

    usart_init();
    gpio_init();
    tusb_init();

    while(1) {
        tud_task(); // handle USB packets
        cdc_process();
        hid_flush();
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

ssize_t _write (int fd, const void * buf, size_t count) {
    while(1);
}

ssize_t _read (int fd, void * buf, size_t count) {
    while(1);
}