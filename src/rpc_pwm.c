#include "simplecborrpc.h"

#include "hal/include/hal_init.h"
#include "hri/hri_nvmctrl_d21.h"
#include "hal/include/hal_gpio.h"
#include "hri/hri_tcc_d21.h"

#include "FreeRTOS.h"
#include "task.h"
#include "event_groups.h"
#include "SEGGER_RTT.h"

#include "pins.h"

#include "rpc_pwm.h"
#include "hid_rpc.h"

static void timer_init(Tcc *tcc, uint8_t index) {
    hri_tcc_wait_for_sync(tcc, TCC_SYNCBUSY_ENABLE);
    hri_tcc_clear_CTRLA_ENABLE_bit(tcc);
    hri_tcc_wait_for_sync(tcc, TCC_SYNCBUSY_ENABLE);

    hri_tcc_wait_for_sync(tcc, TCC_SYNCBUSY_SWRST);
    hri_tcc_set_CTRLA_SWRST_bit(tcc);
    hri_tcc_wait_for_sync(tcc, TCC_SYNCBUSY_SWRST);

    hri_tcc_set_CTRLA_PRESCALER_bf(tcc, TCC_CTRLA_PRESCALER_DIV1_Val);
    hri_tcc_set_WAVE_WAVEGEN_bf(tcc, TCC_WAVE_WAVEGEN_NPWM_Val);

    hri_tcc_wait_for_sync(tcc, TCC_SYNCBUSY_ENABLE);
    hri_tcc_set_CTRLA_ENABLE_bit(tcc);
    hri_tcc_wait_for_sync(tcc, TCC_SYNCBUSY_ENABLE);

    hri_tcc_set_CTRLB_LUPD_bit(tcc);

    hri_tcc_wait_for_sync(tcc, TCC_SYNCBUSY_CC0);
    hri_tcc_write_CC_CC_bf(tcc, index, 0xFFFFF);
    hri_tcc_wait_for_sync(tcc, TCC_SYNCBUSY_CC0);
}

void rpc_pwm_init() {
    timer_init(TCC1, 0);
    timer_init(TCC0, 3);
}

rpc_error_t rpc_pwm_set(const CborValue *args_iterator, CborEncoder *result, const char **error_msg, void *user_ptr) {
    CborValue private_args_it = *args_iterator;
    uint64_t gpio_num;
    uint64_t prescaler_index;
    uint64_t counter_period;
    uint64_t duty_cycle;

    cbor_value_get_uint64(&private_args_it, &gpio_num);
    if (gpio_num < 2 || gpio_num > 3) {
        *error_msg = "PWM is only supported on GPIO 2 and 3";
        return RPC_ERROR_INVALID_ARGS;
    }
    cbor_value_advance(&private_args_it);

    cbor_value_get_uint64(&private_args_it, &prescaler_index);
    if (prescaler_index > 7) {
        *error_msg = "Maximum prescaler index is 7";
        return RPC_ERROR_INVALID_ARGS;
    }
    cbor_value_advance(&private_args_it);

    cbor_value_get_uint64(&private_args_it, &counter_period);
    if (counter_period > 0xFFFFFF) {
        *error_msg = "Counter period out of range";
        return RPC_ERROR_INVALID_ARGS;
    }
    cbor_value_advance(&private_args_it);

    cbor_value_get_uint64(&private_args_it, &duty_cycle);
    if (duty_cycle > 0xFFFFFF) {
        *error_msg = "Duty cycle out of range";
        return RPC_ERROR_INVALID_ARGS;
    }
    if (duty_cycle > counter_period) {
        *error_msg = "Duty cycle is larger than counter period";
        return RPC_ERROR_INVALID_ARGS;
    }
    cbor_value_advance(&private_args_it);

    uint8_t cc_index;
    Tcc *tcc;
    if (gpio_num == 2) {
        cc_index = 0;
        tcc = TCC1;
    } else {
        cc_index = 3;
        tcc = TCC0;
    }

    if (hri_tcc_read_CTRLA_PRESCALER_bf(tcc) != prescaler_index) {
        hri_tcc_wait_for_sync(tcc, TCC_SYNCBUSY_ENABLE);
        hri_tcc_clear_CTRLA_ENABLE_bit(tcc);
        hri_tcc_wait_for_sync(tcc, TCC_SYNCBUSY_ENABLE);

        hri_tcc_write_CTRLA_PRESCALER_bf(tcc, prescaler_index);

        hri_tcc_wait_for_sync(tcc, TCC_SYNCBUSY_ENABLE);
        hri_tcc_set_CTRLA_ENABLE_bit(tcc);
        hri_tcc_wait_for_sync(tcc, TCC_SYNCBUSY_ENABLE);
    }

    if (hri_tcc_read_PER_PER_bf(tcc) != counter_period) {
        hri_tcc_wait_for_sync(tcc, TCC_SYNCBUSY_PER);
        hri_tcc_write_PER_PER_bf(tcc, counter_period);
        hri_tcc_wait_for_sync(tcc, TCC_SYNCBUSY_PER);
    }

    if (hri_tcc_read_CC_CC_bf(tcc, cc_index) != duty_cycle) {
        hri_tcc_wait_for_sync(tcc, TCC_SYNCBUSY_CC(cc_index));
        hri_tcc_write_CC_CC_bf(tcc, cc_index, duty_cycle);
        hri_tcc_wait_for_sync(tcc, TCC_SYNCBUSY_CC(cc_index));
    }

    gpio_set_pin_direction(gpio_pin_map[gpio_num], GPIO_DIRECTION_OUT);
    configure_gpio_function(gpio_pin_map[gpio_num], GPIO_PWM);

    cbor_encode_null(result);
    return RPC_OK;
}

rpc_error_t rpc_pwm_get_info(const CborValue *args_iterator, CborEncoder *result, const char **error_msg, void *user_ptr) {
    CborEncoder info, clock_rates;

    cbor_encoder_create_array(result, &info, 3);

    cbor_encode_uint(&info, 0xFFFFFF); // max counter value
    cbor_encode_uint(&info, 0xFFFFFF); // max compare value

    cbor_encoder_create_array(&info, &clock_rates, 8);
    const uint64_t clock_rate = 48000000;
    cbor_encode_uint(&clock_rates, clock_rate/1);
    cbor_encode_uint(&clock_rates, clock_rate/2);
    cbor_encode_uint(&clock_rates, clock_rate/4);
    cbor_encode_uint(&clock_rates, clock_rate/8);
    cbor_encode_uint(&clock_rates, clock_rate/16);
    cbor_encode_uint(&clock_rates, clock_rate/64);
    cbor_encode_uint(&clock_rates, clock_rate/256);
    cbor_encode_uint(&clock_rates, clock_rate/1024);
    cbor_encoder_close_container(&info, &clock_rates);

    cbor_encoder_close_container(result, &info);

    return RPC_OK;
}

