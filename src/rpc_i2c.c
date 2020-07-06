#include "simplecborrpc.h"

#include "hal/include/hal_init.h"
#include "hri/hri_nvmctrl_d21.h"
#include "hal/include/hal_gpio.h"

#include "FreeRTOS.h"
#include "task.h"
#include "event_groups.h"
#include "SEGGER_RTT.h"

#include "pins.h"

#include "rpc_i2c.h"
#include "hid_rpc.h"

#define RPC_ERROR_I2C_FAILED -32650
#define RPC_ERROR_I2C_READ_FAILED -32651

#define EVENT_I2C_TRANSACTION_DONE  (1 << 0)
#define EVENT_I2C_ERROR             (1 << 1)

#define I2C_BAUD_100K   235
#define I2C_BAUD_400K   53
#define I2C_BAUD_1M     16

extern uint8_t i2c_spi_transaction_buffer[I2C_SPI_TRANSACTION_BUFFER_SIZE];

static EventGroupHandle_t i2c_events;
static StaticEventGroup_t i2c_events_mem;

static volatile bool write_phase;
static volatile bool write_should_stop;
static volatile size_t write_index;
static volatile size_t write_len;

static volatile size_t read_index;
static volatile size_t read_len;

void SERCOM2_Handler() {
    if (hri_sercomi2cm_get_INTFLAG_MB_bit(SERCOM2) || hri_sercomi2cm_get_INTFLAG_SB_bit(SERCOM2)) {
        if (hri_sercomi2cm_get_STATUS_RXNACK_bit(SERCOM2)) {
            hri_sercomi2cm_clear_INTFLAG_reg(SERCOM2, SERCOM_I2CM_INTFLAG_SB | SERCOM_I2CM_INTFLAG_MB);
            hri_sercomi2cm_set_CTRLB_ACKACT_bit(SERCOM2); // send NACK
            hri_sercomi2cm_write_CTRLB_CMD_bf(SERCOM2, 3); // nack and stop

            xEventGroupSetBitsFromISR(i2c_events, EVENT_I2C_ERROR, NULL);
        } else {
            hri_sercomi2cm_clear_INTFLAG_reg(SERCOM2, SERCOM_I2CM_INTFLAG_SB | SERCOM_I2CM_INTFLAG_MB);

            if (write_phase) {
                if (write_len == write_index) {
                    if (write_should_stop) {
                        hri_sercomi2cm_set_CTRLB_ACKACT_bit(SERCOM2); // send NACK
                        hri_sercomi2cm_write_CTRLB_CMD_bf(SERCOM2, 3); // nack and stop
                        xEventGroupSetBitsFromISR(i2c_events, EVENT_I2C_TRANSACTION_DONE, NULL);
                    } else {
                        hri_sercomi2cm_write_ADDR_reg(SERCOM2, hri_sercomi2cm_read_ADDR_reg(SERCOM2) | 1); // switch to read
                        write_phase = false;
                    }
                } else {
                    hri_sercomi2cm_write_DATA_reg(SERCOM2, i2c_spi_transaction_buffer[write_index++]);
                    hri_sercomi2cm_write_CTRLB_CMD_bf(SERCOM2, 2); // ack and continue
                }
            } else {
                i2c_spi_transaction_buffer[read_index++] = hri_sercomi2cm_read_DATA_reg(SERCOM2);

                if (read_len == read_index) {
                    xEventGroupSetBitsFromISR(i2c_events, xEventGroupSetBitsFromISR(i2c_events, EVENT_I2C_TRANSACTION_DONE, NULL), NULL);

                    hri_sercomi2cm_set_CTRLB_ACKACT_bit(SERCOM2); // send NACK
                    hri_sercomi2cm_write_CTRLB_CMD_bf(SERCOM2, 3); // nack and stop
                } else {
                    hri_sercomi2cm_write_CTRLB_CMD_bf(SERCOM2, 2); // continue
                }
            }
        }
    }

    if (hri_sercomi2cm_get_INTFLAG_ERROR_bit(SERCOM2)) {
        xEventGroupSetBitsFromISR(i2c_events, EVENT_I2C_ERROR, NULL);
        hri_sercomi2cm_clear_INTFLAG_ERROR_bit(SERCOM2);
    }
}

static void i2c_reinit_with_baudrate(uint8_t baud_rate) {
    hri_sercomi2cm_wait_for_sync(SERCOM2, SERCOM_I2CM_SYNCBUSY_SWRST);
    hri_sercomi2cm_set_CTRLA_SWRST_bit(SERCOM2);
    hri_sercomi2cm_wait_for_sync(SERCOM2, SERCOM_I2CM_SYNCBUSY_SWRST);

    hri_sercomi2cm_write_CTRLA_MODE_bf(SERCOM2, SERCOM_I2CM_CTRLA_MODE_I2C_MASTER_Val);
    hri_sercomi2cm_write_CTRLA_SPEED_bf(SERCOM2, 1); // FM+ up to 1MHz

    hri_sercomi2cm_write_BAUD_reg(SERCOM2, baud_rate);

    hri_sercomi2cm_set_INTEN_MB_bit(SERCOM2);
    hri_sercomi2cm_set_INTEN_SB_bit(SERCOM2);
    hri_sercomi2cm_set_INTEN_ERROR_bit(SERCOM2);
    NVIC_EnableIRQ(SERCOM2_IRQn);

    hri_sercomi2cm_wait_for_sync(SERCOM2, SERCOM_I2CM_SYNCBUSY_ENABLE);
    hri_sercomi2cm_set_CTRLA_ENABLE_bit(SERCOM2);
    hri_sercomi2cm_wait_for_sync(SERCOM2, SERCOM_I2CM_SYNCBUSY_ENABLE);

    // force bus into idle state
    hri_sercomi2cm_write_STATUS_BUSSTATE_bf(SERCOM2, 1);
}

void rpc_i2c_init() {
    i2c_events = xEventGroupCreateStatic(&i2c_events_mem);
    i2c_reinit_with_baudrate(I2C_BAUD_100K);
}

void i2c_hard_unlock_bus() {
    SEGGER_RTT_printf(0, "I2C hard bus unlock\n");

    gpio_set_pin_function(I2C_SCL_PIN, GPIO_PIN_FUNCTION_OFF);
    gpio_set_pin_direction(I2C_SCL_PIN, GPIO_DIRECTION_OUT);

    for (int i=0; i<9; i++) {
        for (int j=0; j<25; j++) gpio_set_pin_level(I2C_SCL_PIN, false); // approx 5us delay
        for (int j=0; j<25; j++) gpio_set_pin_level(I2C_SCL_PIN, true); // approx 5us delay
    }

    gpio_set_pin_function(I2C_SCL_PIN, PINMUX_PA09D_SERCOM2_PAD1);
    gpio_set_pin_pull_mode(I2C_SCL_PIN, GPIO_PULL_UP);
}

rpc_error_t
rpc_i2c_exchange(const CborValue *args_iterator, CborEncoder *result, const char **error_msg, void *user_ptr) {
    CborValue private_args_it = *args_iterator;
    uint64_t device_address;
    uint64_t rx_byte_count;
    uint64_t clock_rate;
    uint64_t transaction_timeout_ms;

    cbor_value_get_uint64(&private_args_it, &device_address);
    if (device_address > 127) {
        *error_msg = "Invalid device address";
        return RPC_ERROR_INVALID_ARGS;
    }
    cbor_value_advance(&private_args_it);

    size_t length;
    cbor_value_calculate_string_length(&private_args_it, &length);
    if (length > I2C_SPI_TRANSACTION_BUFFER_SIZE) {
        *error_msg = "TX buffer too large";
        return RPC_ERROR_INVALID_ARGS;
    } else {
        write_len = length;
    }
    length = I2C_SPI_TRANSACTION_BUFFER_SIZE;
    cbor_value_copy_byte_string(&private_args_it, i2c_spi_transaction_buffer, &length, &private_args_it);

    cbor_value_get_uint64(&private_args_it, &rx_byte_count);
    if (rx_byte_count > I2C_SPI_TRANSACTION_BUFFER_SIZE) {
        *error_msg = "RX size too large";
        return RPC_ERROR_INVALID_ARGS;
    } else {
        read_len = rx_byte_count;
    }
    cbor_value_advance(&private_args_it);

    cbor_value_get_uint64(&private_args_it, &clock_rate);
    if (clock_rate == 100000) {
        i2c_reinit_with_baudrate(I2C_BAUD_100K);
    } else if (clock_rate == 400000) {
        i2c_reinit_with_baudrate(I2C_BAUD_400K);
    } else if (clock_rate == 1000000) {
        i2c_reinit_with_baudrate(I2C_BAUD_1M);
    } else {
        *error_msg = "Invalid clock rate";
        return RPC_ERROR_INVALID_ARGS;
    }
    cbor_value_advance(&private_args_it);

    cbor_value_get_uint64(&private_args_it, &transaction_timeout_ms);
    if (transaction_timeout_ms > 1000) {
        *error_msg = "Transaction timeout too long";
        return RPC_ERROR_INVALID_ARGS;
    }

    if (write_len == 0 && read_len == 0) {
        cbor_encode_byte_string(result, i2c_spi_transaction_buffer, 0);
        return RPC_OK;
    }

    write_index = 0;
    read_index = 0;

    bool error_occurred = true;

    hri_sercomi2cm_clear_CTRLB_ACKACT_bit(SERCOM2); // send ACKs

    // write phase
    if (write_len) {
        write_phase = true;

        write_should_stop = read_len == 0;

        hri_sercomi2cm_write_ADDR_reg(SERCOM2, SERCOM_I2CM_ADDR_ADDR((device_address << 1)));
    } else {
        write_phase = false;
        hri_sercomi2cm_write_ADDR_reg(SERCOM2, SERCOM_I2CM_ADDR_ADDR((device_address << 1) | 1));
    }

    EventBits_t event_result = xEventGroupWaitBits(i2c_events, EVENT_I2C_TRANSACTION_DONE | EVENT_I2C_ERROR, pdTRUE, pdTRUE, transaction_timeout_ms/portTICK_PERIOD_MS);

    if (event_result & EVENT_I2C_TRANSACTION_DONE) {
        error_occurred = false;
    } else if (event_result & EVENT_I2C_ERROR) {
        error_occurred = true;
    } else {
        SEGGER_RTT_printf(0, "I2C timeout\n");
    }

    if (error_occurred) {
        hri_sercomi2cm_write_CTRLB_CMD_bf(SERCOM2, 3); // generate a stop condition
        i2c_reinit_with_baudrate(clock_rate); // unlock the bus

        *error_msg = "I2C transaction failed";
        return RPC_ERROR_I2C_FAILED;
    }

    cbor_encode_byte_string(result, i2c_spi_transaction_buffer, read_len);
    return RPC_OK;
}