#include "simplecborrpc.h"

#include "hal/include/hal_init.h"
#include "hri/hri_nvmctrl_d21.h"
#include "hal/include/hal_gpio.h"
#include "hri/hri_dac_d21.h"

#include "FreeRTOS.h"
#include "task.h"
#include "event_groups.h"
#include "SEGGER_RTT.h"

#include "pins.h"

#include "rpc_dac.h"
#include "hid_rpc.h"

void rpc_dac_init() {
    hri_dac_wait_for_sync(DAC);
    hri_dac_clear_CTRLA_ENABLE_bit(DAC);
    hri_dac_wait_for_sync(DAC);

    hri_dac_set_CTRLA_SWRST_bit(DAC);
    hri_dac_wait_for_sync(DAC);

    hri_dac_write_CTRLB_REFSEL_bf(DAC, DAC_CTRLB_REFSEL_AVCC_Val);
    hri_dac_wait_for_sync(DAC);

    hri_dac_set_CTRLB_EOEN_bit(DAC);
    hri_dac_wait_for_sync(DAC);
}

void dac_disable() {
    if (hri_dac_get_CTRLA_ENABLE_bit(DAC)) {
        hri_dac_clear_CTRLA_ENABLE_bit(DAC);
        hri_dac_wait_for_sync(DAC);
    }
}

void dac_enable() {
    if (!hri_dac_get_CTRLA_ENABLE_bit(DAC)) {
        hri_dac_set_CTRLA_ENABLE_bit(DAC);
        hri_dac_wait_for_sync(DAC);
    }
}

rpc_error_t rpc_dac_set(const CborValue *args_iterator, CborEncoder *result, const char **error_msg, void *user_ptr) {
    CborValue private_args_it = *args_iterator;
    uint64_t gpio_num;
    uint64_t dac_level;

    cbor_value_get_uint64(&private_args_it, &gpio_num);
    if (gpio_num != 0) {
        *error_msg = "DAC output is only supported on GPIO 0";
        return RPC_ERROR_INVALID_ARGS;
    }
    cbor_value_advance(&private_args_it);

    cbor_value_get_uint64(&private_args_it, &dac_level);
    if (dac_level > 1023) {
        *error_msg = "Maximum DAC level is 1023";
        return RPC_ERROR_INVALID_ARGS;
    }
    cbor_value_advance(&private_args_it);

    dac_enable();

    hri_dac_wait_for_sync(DAC);
    hri_dac_write_DATA_DATA_bf(DAC, dac_level);
    hri_dac_wait_for_sync(DAC);

    configure_gpio_function(gpio_pin_map[gpio_num], GPIO_DAC);

    cbor_encode_null(result);
    return RPC_OK;
}

rpc_error_t rpc_dac_get_info(const CborValue *args_iterator, CborEncoder *result, const char **error_msg, void *user_ptr) {
    CborEncoder array;

    cbor_encoder_create_array(result, &array, 2);
    cbor_encode_uint(&array, 1023);
    cbor_encode_double(&array, 3.3);
    cbor_encoder_close_container(result, &array);

    return RPC_OK;
}
