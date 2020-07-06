#include "sam.h"
#include "compiler.h"
#include "utils.h"

#include "FreeRTOS.h"
#include "event_groups.h"
#include "limits.h"

#include "SEGGER_RTT.h"

#include "tusb.h"

#include "cdc_usart.h"
#include "config.h"

#include "pins.h"
#include "dma.h"

#define EVENT_USART_RX                  (1 << 0)
#define EVENT_CDC_RX                    (1 << 1)
#define EVENT_CDC_LINE_CODING_CHANGED   (1 << 2)
#define EVENT_DMA_COMPLETE              (1 << 3)

#define CDC_USART_STACK_SIZE 128
StackType_t cdc_usart_task_stack[CDC_USART_STACK_SIZE];
StaticTask_t cdc_usart_task_taskdef;
TaskHandle_t cdc_usart_task_handle;

#define USART_MICROBUF_SIZE 128
static uint8_t usart_microbuf[2][USART_MICROBUF_SIZE];
static size_t usart_microbuf_index[2] = {0, 0};
static size_t usart_microbuf_selected = 0;

volatile cdc_line_coding_t cdc_line_coding = {
        .bit_rate = 9600,
        .stop_bits = 0,
        .parity = 0,
        .data_bits = 8
};

COMPILER_ALIGNED(16)
static uint8_t dma_buffer[64];

void usart_update_configuration_from_line_coding();

void usart_dma_callback();

static DmacDescriptor *dma_descriptor;

void cdc_usart_init() {
    // USART is via SERCOM4
    hri_sercomusart_wait_for_sync(SERCOM4, SERCOM_USART_SYNCBUSY_SWRST);
    hri_sercomusart_set_CTRLA_SWRST_bit(SERCOM4);
    hri_sercomusart_wait_for_sync(SERCOM4, SERCOM_USART_SYNCBUSY_SWRST);

    usart_update_configuration_from_line_coding();

    hri_sercomusart_write_INTEN_RXC_bit(SERCOM4, true); // enable RX interrupt
    NVIC_EnableIRQ(SERCOM4_IRQn);

    // enable SERCOM peripheral
    hri_sercomusart_wait_for_sync(SERCOM4, SERCOM_USART_SYNCBUSY_SWRST);
    hri_sercomusart_set_CTRLA_ENABLE_bit(SERCOM4);
    hri_sercomusart_wait_for_sync(SERCOM4, SERCOM_USART_SYNCBUSY_SWRST);

    dma_set_callback(DMA_CHANNEL_USART, usart_dma_callback);
    dma_descriptor = dma_get_descriptor_for_channel(DMA_CHANNEL_USART);
    dma_configure_channel(DMA_CHANNEL_USART, DMAC_CHCTRLB_TRIGACT_BEAT_Val, SERCOM4_DMAC_ID_TX);

    // set DMA target as the USART data register
    hri_dmacdescriptor_write_DSTADDR_reg(dma_descriptor, (uint32_t) &(SERCOM4->USART.DATA));
    hri_dmacdescriptor_set_BTCTRL_SRCINC_bit(dma_descriptor); // increment source address
    hri_dmacdescriptor_set_BTCTRL_STEPSEL_bit(dma_descriptor); // stepsize applies to SRC
    hri_dmacdescriptor_set_BTCTRL_VALID_bit(dma_descriptor); // mark descriptor as valid
}

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

        if (usart_microbuf_index[usart_microbuf_selected] < USART_MICROBUF_SIZE) {
            usart_microbuf[usart_microbuf_selected][usart_microbuf_index[usart_microbuf_selected]++] = tmp;
        }
        xTaskNotifyFromISR(cdc_usart_task_handle, EVENT_USART_RX, eSetBits, NULL);
    }
}

void usart_update_configuration_from_line_coding() {
    uint32_t ctrla_tmp = SERCOM_USART_CTRLA_DORD | // send LSB first (this is the normal way for UART)
                         SERCOM_USART_CTRLA_RXPO(1) | // RX uses PAD1
                         SERCOM_USART_CTRLA_TXPO(0) | // TX uses PAD0
                         SERCOM_USART_CTRLA_MODE(1); // use internal clock

    uint32_t ctrlb_tmp = SERCOM_USART_CTRLB_RXEN | SERCOM_USART_CTRLB_TXEN;
    uint32_t baud_tmp;

    // if parity == 0, no parity so don't set any bits

    // if parity == 1 or 2, SERCOM_USART_CTRLB_PMODE should be set to 0 (default) but enable parity in CTRLA
    if (cdc_line_coding.parity != 0) {
        ctrla_tmp |= SERCOM_USART_CTRLA_FORM(1); // enable parity

        // if parity == 2, set SERCOM_USART_CTRLB_PMODE to enable odd parity
        if (cdc_line_coding.parity == 2) ctrlb_tmp |= SERCOM_USART_CTRLB_PMODE;
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

    xTaskNotify(cdc_usart_task_handle, EVENT_CDC_LINE_CODING_CHANGED, eSetBits);
}

void tud_cdc_rx_cb(uint8_t itf) {
    xTaskNotify(cdc_usart_task_handle, EVENT_CDC_RX, eSetBits);
}

void usart_dma_callback() {
    xTaskNotify(cdc_usart_task_handle, EVENT_DMA_COMPLETE, eSetBits);
}

void cdc_usart_task(void *param) {
    cdc_usart_init();

    bool line_coding_needs_change = false;
    bool cdc_data_waiting = false;
    bool dma_running = false;

    while (1) {
        uint32_t event;

        BaseType_t result = xTaskNotifyWait(pdTRUE,ULONG_MAX, &event, portMAX_DELAY);

        if (result == pdTRUE) {
            if (event & EVENT_CDC_LINE_CODING_CHANGED) {
                line_coding_needs_change = true;
            }

            if (event & EVENT_USART_RX) {
                portENTER_CRITICAL();
                usart_microbuf_selected ^= 1; // flip buffers
                portEXIT_CRITICAL();

                const size_t microbuf_selected = usart_microbuf_selected ^ 1;
                uint8_t *microbuf = &usart_microbuf[microbuf_selected][0];
                const size_t microbuf_index = usart_microbuf_index[microbuf_selected];

                if (microbuf_index > 0) {
                    tud_cdc_write(microbuf, microbuf_index);
                    tud_cdc_write_flush();

                    usart_microbuf_index[microbuf_selected] = 0;
                }
            }

            if (event & EVENT_CDC_RX) {
                cdc_data_waiting = true;
            }

            if (event & EVENT_DMA_COMPLETE) {
                dma_running = false;
            }

            if (line_coding_needs_change && !dma_running) {
                hri_sercomusart_clear_CTRLA_ENABLE_bit(SERCOM4); // disable SERCOM4
                hri_sercomusart_wait_for_sync(SERCOM4, SERCOM_USART_SYNCBUSY_ENABLE);

                usart_update_configuration_from_line_coding();
                line_coding_needs_change = false;

                hri_sercomusart_set_CTRLA_ENABLE_bit(SERCOM4);
                hri_sercomusart_wait_for_sync(SERCOM4, SERCOM_USART_SYNCBUSY_ENABLE);
            }

            if (cdc_data_waiting && !dma_running) {
                uint32_t tx_len = tud_cdc_read(dma_buffer, 64);
                if (tx_len) {
                    // set pointer to the final beat of the transaction
                    hri_dmacdescriptor_write_SRCADDR_reg(dma_descriptor, (uint32_t) (dma_buffer + tx_len));

                    // set beat count
                    hri_dmacdescriptor_write_BTCNT_reg(dma_descriptor, (uint16_t) tx_len);

                    // enable DMA channel
                    dma_start_channel(DMA_CHANNEL_USART);
                    dma_running = true;
                }

                cdc_data_waiting = tud_cdc_available() != 0;
            }
        }
    }
}

void start_cdc_usart_task() {
    cdc_usart_task_handle =  xTaskCreateStatic( cdc_usart_task,
            "cdc",
            CDC_USART_STACK_SIZE,
            NULL,
            configMAX_PRIORITIES-1,
            cdc_usart_task_stack,
            &cdc_usart_task_taskdef);
}