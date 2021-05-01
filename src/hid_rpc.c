#include "ringbuf.h"
#include "simplehdlc.h"
#include "simplecborrpc.h"
#include "utils.h"
#include "rpc_api.h"

#include "SEGGER_RTT.h"
#include "tusb.h"

#include "leds.h"
#include "hid_rpc.h"
#include "rpc_i2c.h"
#include "rpc_spi.h"
#include "rpc_pwm.h"
#include "rpc_dac.h"

#define EVENT_HID_RX                  (1 << 0)

#define HID_RPC_STACK_SIZE 256
StackType_t hid_rpc_task_stack[HID_RPC_STACK_SIZE];
StaticTask_t hid_rpc_task_taskdef;
TaskHandle_t hid_rpc_task_handle;

COMPILER_ALIGNED(16)
uint8_t i2c_spi_transaction_buffer[I2C_SPI_TRANSACTION_BUFFER_SIZE];

uint8_t hdlc_rx_buffer[5*I2C_SPI_TRANSACTION_BUFFER_SIZE/2];
uint8_t rpc_output_buffer[5*I2C_SPI_TRANSACTION_BUFFER_SIZE/2];

uint8_t hid_rx_buffer[256];
uint8_t hid_tx_buffer[128];

simplehdlc_context_t hdlc_context;
ringbuf_t hid_rx_ringbuffer;
ringbuf_t hid_tx_ringbuffer;

void hdlc_rx_packet_callback(const uint8_t *payload, uint16_t len, void *user_ptr);
void hdlc_tx_flush_buffer_callback(void *user_ptr);
void hdlc_tx_byte_callback(uint8_t byte, void *user_ptr);

simplehdlc_callbacks_t hdlc_callbacks = {
        .rx_packet_callback = &hdlc_rx_packet_callback,
        .tx_byte_callback = &hdlc_tx_byte_callback,
        .tx_flush_buffer_callback = &hdlc_tx_flush_buffer_callback,
};

void hid_rpc_init() {
    rpc_i2c_init();
    rpc_spi_init();
    rpc_pwm_init();
    rpc_dac_init();

    simplehdlc_init(&hdlc_context, hdlc_rx_buffer, sizeof(hdlc_rx_buffer), &hdlc_callbacks, NULL);
    ringbuf_init(&hid_rx_ringbuffer, hid_rx_buffer, sizeof(hid_rx_buffer));
    ringbuf_init(&hid_tx_ringbuffer, hid_tx_buffer, sizeof(hid_tx_buffer));
}

static uint8_t flush_tmp[64];
void hid_flush() {
    if (ringbuf_len(&hid_tx_ringbuffer)) {
        set_led_counter(LED_BUSY_IDX, 2);

        size_t count = ringbuf_pop(&hid_tx_ringbuffer, flush_tmp, 64);

        // add padding
        for (size_t i=count; i<64; i++) {
            flush_tmp[i] = SIMPLEHDLC_BOUNDARY_MARKER;
        }
//
//        SEGGER_RTT_printf(0, "[RPC] HID flush: ");
//        for (int i=0; i<64; i++) {
//            SEGGER_RTT_printf(0, "%02X ", flush_tmp[i]);
//        }
//        SEGGER_RTT_printf(0, "\n");

        for (int i=0; i<100; i++) {
            if (tud_hid_report(0, flush_tmp, 64) == true) {
                return;
            } else {
                vTaskDelay(1);
            }
        }
//        SEGGER_RTT_printf(0, "[RPC] Failed to deliver HID report\n");
    }
}

void hdlc_rx_packet_callback(const uint8_t *payload, uint16_t len, void *user_ptr) {
//    SEGGER_RTT_printf(0, "[RPC] HDLC decode success!\n");
    size_t output_length = sizeof(rpc_output_buffer);
    rpc_error_t err = execute_rpc_call(rpc_function_table, SIMPLECBORRPC_FUNCTION_COUNT, payload, len, rpc_output_buffer, &output_length, NULL);
//    SEGGER_RTT_printf(0, "[RPC] returning %u bytes, return code: %d\n", output_length, (int) err);
    simplehdlc_encode_to_callback(&hdlc_context, rpc_output_buffer, output_length, true);
}

void hdlc_tx_flush_buffer_callback(void *user_ptr) {
    hid_flush();
}

void hdlc_tx_byte_callback(uint8_t byte, void *user_ptr) {
    ringbuf_push(&hid_tx_ringbuffer, &byte, 1);

    if (ringbuf_len(&hid_tx_ringbuffer) >= 64) {
        hid_flush();
    }
}

// HID get report callback
uint16_t tud_hid_get_report_cb(uint8_t report_id, hid_report_type_t report_type, uint8_t *buffer, uint16_t reqlen) {
    // dummy implementation, should never be used anyway
//    SEGGER_RTT_printf(0, "tud_hid_get_report_cb: report_id: %d, report_type: %d, reqlen: %d\n", report_id, report_type,
//                      reqlen);

    for (int i = 0; i < reqlen; i++) {
        buffer[i] = SIMPLEHDLC_BOUNDARY_MARKER;
    }

    return reqlen;
}

// HID set report callback
void tud_hid_set_report_cb(uint8_t report_id, hid_report_type_t report_type, uint8_t const *buffer, uint16_t bufsize) {
    // unimplemented as this is not needed

//    SEGGER_RTT_printf(0, "[HID] got report from host: report_id: %d, report_type: %d, bufsize: %d\n", report_id, report_type,
//                      bufsize);
//
//    if (bufsize != 64) {
//        SEGGER_RTT_printf(0, "Got buffer with invalid size: %d, contents:\n", bufsize);
//        for (int i=0; i<bufsize; i++) {
//            SEGGER_RTT_printf(0, "%02X ", buffer[i]);
//        }
//        SEGGER_RTT_printf(0, "\n");
//    }

    ringbuf_push(&hid_rx_ringbuffer, buffer, bufsize);
    set_led_counter(LED_BUSY_IDX, 2);
    xTaskNotify(hid_rpc_task_handle, EVENT_HID_RX, eSetBits);
}

rpc_error_t
rpc_max_size(const CborValue *args_iterator, CborEncoder *result, const char **error_msg, void *user_ptr) {
    cbor_encode_int(result, sizeof(hdlc_rx_buffer));

    return RPC_OK;
}

static uint8_t tmp_buf[64];

void hid_rpc_task(void *param) {
    hid_rpc_init();

    while (1) {
        uint32_t event;

        BaseType_t result = xTaskNotifyWait(pdTRUE,ULONG_MAX, &event, portMAX_DELAY);

        if (result == pdTRUE) {
            if (event & EVENT_HID_RX) {
                while (ringbuf_len(&hid_rx_ringbuffer)) {
                    size_t count = ringbuf_pop(&hid_rx_ringbuffer, tmp_buf, sizeof(tmp_buf));
                    simplehdlc_parse(&hdlc_context, tmp_buf, count);
                }
            }
        }
    }
}

void start_hid_rpc_task() {
    hid_rpc_task_handle =  xTaskCreateStatic( hid_rpc_task,
                                                "hid",
                                                HID_RPC_STACK_SIZE,
                                                NULL,
                                                configMAX_PRIORITIES-1,
                                                hid_rpc_task_stack,
                                                &hid_rpc_task_taskdef);
}