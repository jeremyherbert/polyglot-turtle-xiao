#include "hal/include/hal_gpio.h"

#include "rpc_dac.h"
#include "pins.h"

int configure_gpio_function(uint32_t pin_number, gpio_type_t type) {
    uint32_t pin_function = GPIO_PIN_FUNCTION_OFF;

    if (pin_number == PIN_PA02) {
        if (type != GPIO_DAC) {
            dac_disable();
        }

        if (type == GPIO_DAC) pin_function = PINMUX_PA02B_DAC_VOUT;
        else if (type == GPIO_PWM) return -1;
        else if (type == GPIO_ADC) return -1;

    } else if (pin_number == PIN_PA04) {
        if (type == GPIO_DAC) return -1;
        else if (type == GPIO_PWM) return -1;
        else if (type == GPIO_ADC) return -1;

    } else if (pin_number == PIN_PA10) {
        if (type == GPIO_DAC) return -1;
        else if (type == GPIO_PWM) pin_function = PINMUX_PA10E_TCC1_WO0;
        else if (type == GPIO_ADC) return -1;

    } else if (pin_number == PIN_PA11) {
        if (type == GPIO_DAC) return -1;
        else if (type == GPIO_PWM) pin_function = PINMUX_PA11F_TCC0_WO3;
        else if (type == GPIO_ADC) return -1;
    } else {
        return -2;
    }

    switch (type) {
        case GPIO_OFF:
        case GPIO_ADC:
        case GPIO_DAC:
            gpio_set_pin_direction(pin_number, GPIO_DIRECTION_OFF);
            break;

        default:
            break;
    }
    gpio_set_pin_function(pin_number, pin_function);

    return 0;
}