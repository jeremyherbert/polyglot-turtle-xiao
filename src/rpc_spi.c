#include "simplecborrpc.h"

#include "utils.h"
#include "FreeRTOS.h"
#include "event_groups.h"
#include "hri/hri_nvmctrl_d21.h"
#include "SEGGER_RTT.h"
#include "hal/include/hal_gpio.h"

#include "pins.h"
#include "rpc_spi.h"
#include "dma.h"
#include "config.h"
#include "hid_rpc.h"

#define EVENT_SPI_WRITE_DONE (1 << 0)
#define EVENT_SPI_READ_DONE (1 << 1)

static EventGroupHandle_t spi_events;
static StaticEventGroup_t spi_events_mem;

DmacDescriptor *spi_dma_write_descriptor;
DmacDescriptor *spi_dma_read_descriptor;

void spi_write_dma_callback() {
    xEventGroupSetBits(spi_events, EVENT_SPI_WRITE_DONE);
}

void spi_read_dma_callback() {
    xEventGroupSetBits(spi_events, EVENT_SPI_READ_DONE);
}

void spi_reinit_with_clock_rate(uint32_t clock_rate, uint8_t spi_mode) {
    hri_sercomspi_wait_for_sync(SERCOM0, SERCOM_SPI_SYNCBUSY_SWRST);
    hri_sercomspi_set_CTRLA_SWRST_bit(SERCOM0);
    hri_sercomspi_wait_for_sync(SERCOM0, SERCOM_SPI_SYNCBUSY_SWRST);

    hri_sercomspi_set_CTRLA_MODE_bf(SERCOM0, 3); // select SPI master

    switch(spi_mode) {
        case 0:
            hri_sercomspi_clear_CTRLA_CPHA_bit(SERCOM0);
            hri_sercomspi_clear_CTRLA_CPOL_bit(SERCOM0);
            break;
        case 1:
            hri_sercomspi_set_CTRLA_CPHA_bit(SERCOM0);
            hri_sercomspi_clear_CTRLA_CPOL_bit(SERCOM0);
            break;
        case 2:
            hri_sercomspi_clear_CTRLA_CPHA_bit(SERCOM0);
            hri_sercomspi_set_CTRLA_CPOL_bit(SERCOM0);
            break;
        case 3:
            hri_sercomspi_set_CTRLA_CPHA_bit(SERCOM0);
            hri_sercomspi_set_CTRLA_CPOL_bit(SERCOM0);
            break;
        default:
            configASSERT(false);
    }

    hri_sercomspi_set_CTRLA_DIPO_bf(SERCOM0, 1); // PAD1 is MISO
    hri_sercomspi_set_CTRLA_DOPO_bf(SERCOM0, 1); // PAD2 MOSI, PAD3 is SCK

    hri_sercomspi_set_CTRLB_RXEN_bit(SERCOM0);

    uint64_t refclk = 48000000;
    hri_sercomspi_set_BAUD_BAUD_bf(SERCOM0, (refclk / (2ULL * clock_rate)) - 1);

    hri_sercomspi_set_CTRLA_ENABLE_bit(SERCOM0);
    hri_sercomspi_wait_for_sync(SERCOM0, SERCOM_SPI_SYNCBUSY_ENABLE);

    // configure TX DMA
    dma_set_callback(DMA_CHANNEL_SPI_WRITE, spi_write_dma_callback);
    spi_dma_write_descriptor = dma_get_descriptor_for_channel(DMA_CHANNEL_SPI_WRITE);
    dma_configure_channel(DMA_CHANNEL_SPI_WRITE, DMAC_CHCTRLB_TRIGACT_BEAT_Val, SERCOM0_DMAC_ID_TX);

    // set DMA target as the SPI data register
    hri_dmacdescriptor_write_DSTADDR_reg(spi_dma_write_descriptor, (uint32_t) &(SERCOM0->SPI.DATA));
    hri_dmacdescriptor_set_BTCTRL_SRCINC_bit(spi_dma_write_descriptor); // increment source address
    hri_dmacdescriptor_set_BTCTRL_STEPSEL_bit(spi_dma_write_descriptor); // stepsize applies to SRC
    hri_dmacdescriptor_set_BTCTRL_VALID_bit(spi_dma_write_descriptor); // mark descriptor as valid

    // configure RX DMA
    dma_set_callback(DMA_CHANNEL_SPI_READ, spi_read_dma_callback);
    spi_dma_read_descriptor = dma_get_descriptor_for_channel(DMA_CHANNEL_SPI_READ);
    dma_configure_channel(DMA_CHANNEL_SPI_READ, DMAC_CHCTRLB_TRIGACT_BEAT_Val, SERCOM0_DMAC_ID_RX);

    // set DMA target as the SPI data register
    hri_dmacdescriptor_write_SRCADDR_reg(spi_dma_read_descriptor, (uint32_t) &(SERCOM0->SPI.DATA));
    hri_dmacdescriptor_set_BTCTRL_DSTINC_bit(spi_dma_read_descriptor); // increment source address
    hri_dmacdescriptor_set_BTCTRL_STEPSEL_bit(spi_dma_read_descriptor); // stepsize applies to DST
    hri_dmacdescriptor_set_BTCTRL_VALID_bit(spi_dma_read_descriptor); // mark descriptor as valid
}

void rpc_spi_init() {
    spi_events = xEventGroupCreateStatic(&spi_events_mem);

    spi_reinit_with_clock_rate(100000, 0);
}

rpc_error_t
rpc_spi_exchange(const CborValue *args_iterator, CborEncoder *result, const char **error_msg, void *user_ptr) {
    CborValue private_args_it = *args_iterator;
    size_t tx_len;
    uint64_t spi_index;
    uint64_t rx_len;
    uint64_t spi_mode;
    uint64_t clock_rate;
    uint64_t transaction_timeout_ms;
    uint64_t cs_pin_number;

    // clear the buffer
    memset(transaction_buffer, 0, I2C_SPI_TRANSACTION_BUFFER_SIZE);

    cbor_value_get_uint64(&private_args_it, &spi_index);
    if (spi_index != 0) {
        *error_msg = "Invalid SPI index";
        return RPC_ERROR_INVALID_ARGS;
    }
    cbor_value_advance(&private_args_it);

    size_t length;
    cbor_value_calculate_string_length(&private_args_it, &length);
    if (length > I2C_SPI_TRANSACTION_BUFFER_SIZE) {
        *error_msg = "TX buffer too large";
        return RPC_ERROR_INVALID_ARGS;
    } else {
        tx_len = length;
    }
    length = I2C_SPI_TRANSACTION_BUFFER_SIZE;
    cbor_value_copy_byte_string(&private_args_it, transaction_buffer, &length, &private_args_it);

    cbor_value_get_uint64(&private_args_it, &rx_len);
    if (rx_len > I2C_SPI_TRANSACTION_BUFFER_SIZE) {
        *error_msg = "RX request too large";
        return RPC_ERROR_INVALID_ARGS;
    }
    cbor_value_advance(&private_args_it);

    cbor_value_get_uint64(&private_args_it, &clock_rate);
    if (clock_rate > 10000000) {
        *error_msg = "Clock rate too high";
        return RPC_ERROR_INVALID_ARGS;
    }
    if (clock_rate < 100000) {
        *error_msg = "Clock rate too low";
        return RPC_ERROR_INVALID_ARGS;
    }
    cbor_value_advance(&private_args_it);

    cbor_value_get_uint64(&private_args_it, &spi_mode);
    if (spi_mode > 3) {
        *error_msg = "Invalid mode";
        return RPC_ERROR_INVALID_ARGS;
    }
    cbor_value_advance(&private_args_it);

    cbor_value_get_uint64(&private_args_it, &transaction_timeout_ms);
    if (transaction_timeout_ms > 1000) {
        *error_msg = "Transaction timeout too long";
        return RPC_ERROR_INVALID_ARGS;
    }
    cbor_value_advance(&private_args_it);


    cbor_value_get_uint64(&private_args_it, &cs_pin_number);
    if (cs_pin_number != 0xFF) {
        if (cs_pin_number > GPIO_PIN_COUNT) {
            *error_msg = "Invalid CS pin";
            return RPC_ERROR_INVALID_ARGS;
        }
    }

    if (tx_len == 0 && rx_len == 0) {
        cbor_encode_byte_string(result, transaction_buffer, 0);
        return RPC_OK;
    }

    if (rx_len > tx_len) {
        tx_len = rx_len;
    }

    if (cs_pin_number != 0xFF) {
        configure_gpio_function(gpio_pin_map[cs_pin_number], GPIO_NO_ALTERNATE_FUNCTION);
        gpio_set_pin_direction(gpio_pin_map[cs_pin_number], GPIO_DIRECTION_OUT);
        gpio_set_pin_level(gpio_pin_map[cs_pin_number], true);
    }

    spi_reinit_with_clock_rate(clock_rate, spi_mode);

    if (cs_pin_number != 0xFF) {
        gpio_set_pin_level(gpio_pin_map[cs_pin_number], false);
    }
    
    xEventGroupClearBits(spi_events, EVENT_SPI_READ_DONE | EVENT_SPI_WRITE_DONE);

    // setup DMA read
    // set pointer to the final beat of the transaction
    hri_dmacdescriptor_write_DSTADDR_reg(spi_dma_read_descriptor, (uint32_t) (transaction_buffer + tx_len));
    hri_dmacdescriptor_write_BTCNT_reg(spi_dma_read_descriptor, (uint16_t) tx_len); // set beat count

    // setup DMA write
    // set pointer to the final beat of the transaction
    hri_dmacdescriptor_write_SRCADDR_reg(spi_dma_write_descriptor, (uint32_t) (transaction_buffer + tx_len));
    hri_dmacdescriptor_write_BTCNT_reg(spi_dma_write_descriptor, (uint16_t) tx_len); // set beat count

    // enable DMA channels
    dma_start_channel(DMA_CHANNEL_SPI_READ);
    dma_start_channel(DMA_CHANNEL_SPI_WRITE);

    EventBits_t events = xEventGroupWaitBits(spi_events, EVENT_SPI_WRITE_DONE | EVENT_SPI_READ_DONE, pdTRUE, pdTRUE, transaction_timeout_ms/portTICK_PERIOD_MS);

    if (cs_pin_number != 0xFF) {
        gpio_set_pin_level(gpio_pin_map[cs_pin_number], true);
    }

    if (events & EVENT_SPI_WRITE_DONE && events & EVENT_SPI_READ_DONE) {
        cbor_encode_byte_string(result, transaction_buffer, rx_len);
        return RPC_OK;
    } else {
        *error_msg = "SPI transaction timed out";
        return RPC_ERROR_INTERNAL_ERROR;
    }
}
