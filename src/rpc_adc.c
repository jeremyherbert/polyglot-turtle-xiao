#include "simplecborrpc.h"

#include "hal/include/hal_init.h"
#include "hri/hri_nvmctrl_d21.h"
#include "hal/include/hal_gpio.h"
#include "hri/hri_adc_d21.h"

#include "FreeRTOS.h"

#include "pins.h"

#include "rpc_adc.h"

void rpc_adc_init() {
    hri_adc_wait_for_sync(ADC);
    hri_adc_clear_CTRLA_ENABLE_bit(ADC);

    hri_adc_wait_for_sync(ADC);
    hri_adc_set_CTRLA_SWRST_bit(ADC);

    hri_adc_wait_for_sync(ADC);
    hri_adc_write_INPUTCTRL_GAIN_bf(ADC, ADC_INPUTCTRL_GAIN_DIV2_Val);

    hri_adc_wait_for_sync(ADC);
    hri_adc_write_REFCTRL_REFSEL_bf(ADC, ADC_REFCTRL_REFSEL_INTVCC1_Val);

    hri_adc_wait_for_sync(ADC);
    hri_adc_write_CTRLB_RESSEL_bf(ADC, ADC_CTRLB_RESSEL_12BIT_Val);

    hri_adc_wait_for_sync(ADC);
    hri_adc_clear_CTRLB_DIFFMODE_bit(ADC);

    hri_adc_wait_for_sync(ADC);
    // ADC clock must be less than 2.1MHz
    hri_adc_write_CTRLB_PRESCALER_bf(ADC, ADC_CTRLB_PRESCALER_DIV4_Val); // 8MHz / 4 = 2MHz

    hri_adc_wait_for_sync(ADC);
    hri_adc_write_CALIB_LINEARITY_CAL_bf(ADC,
                                         (((*((uint32_t*) ADC_FUSES_LINEARITY_1_ADDR) & ADC_FUSES_LINEARITY_1_Msk) >> ADC_FUSES_LINEARITY_1_Pos) << 5) |
                                               ((*((uint32_t*) ADC_FUSES_LINEARITY_0_ADDR) & ADC_FUSES_LINEARITY_0_Msk) >> ADC_FUSES_LINEARITY_0_Pos));

    hri_adc_wait_for_sync(ADC);
    hri_adc_write_CALIB_BIAS_CAL_bf(ADC, (*((uint32_t*) ADC_FUSES_BIASCAL_ADDR) & ADC_FUSES_BIASCAL_Msk) >> ADC_FUSES_BIASCAL_Pos);

    hri_adc_wait_for_sync(ADC);
    hri_adc_set_CTRLA_ENABLE_bit(ADC);

    hri_adc_wait_for_sync(ADC);
}


rpc_error_t rpc_adc_get(const CborValue *args_iterator, CborEncoder *result, const char **error_msg, void *user_ptr) {
    CborValue private_args_it = *args_iterator;
    uint64_t pin_number;

    cbor_value_get_uint64(&private_args_it, &pin_number);
    cbor_value_advance(&private_args_it);

    if (pin_number >= GPIO_PIN_COUNT) {
        *error_msg = "Invalid pin";
        return RPC_ERROR_INVALID_ARGS;
    }


    configure_gpio_function(gpio_pin_map[pin_number], GPIO_ADC);

    hri_adc_wait_for_sync(ADC);
    // this needs to be selected even in single ended according to the datasheet
    hri_adc_write_INPUTCTRL_MUXNEG_bf(ADC, ADC_INPUTCTRL_MUXNEG_IOGND_Val);

    hri_adc_wait_for_sync(ADC);
    switch (pin_number) {
        case 0:
            hri_adc_write_INPUTCTRL_MUXPOS_bf(ADC, ADC_INPUTCTRL_MUXPOS_PIN0_Val);
            break;

        case 1:
            hri_adc_write_INPUTCTRL_MUXPOS_bf(ADC, ADC_INPUTCTRL_MUXPOS_PIN4_Val);
            break;

        case 2:
            hri_adc_write_INPUTCTRL_MUXPOS_bf(ADC, ADC_INPUTCTRL_MUXPOS_PIN18_Val);
            break;

        case 3:
            hri_adc_write_INPUTCTRL_MUXPOS_bf(ADC, ADC_INPUTCTRL_MUXPOS_PIN19_Val);
            break;

        default:
            configASSERT(false);
    }
    hri_adc_wait_for_sync(ADC);



    volatile uint32_t adc_reading;
    for (int i=0; i<3; i++) { // throw away the first samples to improve accuracy
        hri_adc_wait_for_sync(ADC);
        hri_adc_clear_INTFLAG_RESRDY_bit(ADC);
        hri_adc_wait_for_sync(ADC);
        hri_adc_set_SWTRIG_START_bit(ADC);
        hri_adc_wait_for_sync(ADC);

        while (!hri_adc_get_INTFLAG_RESRDY_bit(ADC));

        adc_reading = hri_adc_get_RESULT_RESULT_bf(ADC, ADC_RESULT_MASK);
    }


    CborEncoder reading;
    cbor_encoder_create_array(result, &reading, 3);
    cbor_encode_uint(&reading, adc_reading);
    cbor_encode_uint(&reading, 4095); // maximum value
    cbor_encode_double(&reading, 3.3); // maximum voltage
    cbor_encoder_close_container(result, &reading);

    return RPC_OK;
}
