#include "hal/include/hal_gpio.h"

#include "FreeRTOS.h"

#include "rpc_dac.h"
#include "pins.h"

void configure_gpio_function(uint32_t pin_number, gpio_type_t type) {
    uint32_t pin_function = GPIO_PIN_FUNCTION_OFF;

    if (pin_number == PIN_PA02) {
        if (type != GPIO_DAC) {
            dac_disable(); // DAC output buffer interferes with other functionality on this pin
        }

        if (type == GPIO_DAC) {
            pin_function = PINMUX_PA02B_DAC_VOUT;
        }
        else if (type == GPIO_PWM) {
            configASSERT(0);
        }
        else if (type == GPIO_ADC) {
            pin_function = PINMUX_PA02B_ADC_AIN0;
        }

    } else if (pin_number == PIN_PA04) {
        if (type == GPIO_DAC) {
            configASSERT(0);
        }
        else if (type == GPIO_PWM) {
            configASSERT(0);
        }
        else if (type == GPIO_ADC) {
            pin_function = PINMUX_PA04B_ADC_AIN4;
        }

    } else if (pin_number == PIN_PA10) {
        if (type == GPIO_DAC) {
            configASSERT(0);
        }
        else if (type == GPIO_PWM) {
            pin_function = PINMUX_PA10E_TCC1_WO0;
        }
        else if (type == GPIO_ADC) {
            pin_function = PINMUX_PA10B_ADC_AIN18;
        }

    } else if (pin_number == PIN_PA11) {
        if (type == GPIO_DAC) {
            configASSERT(0);
        }
        else if (type == GPIO_PWM) {
            pin_function = PINMUX_PA11F_TCC0_WO3;
        }
        else if (type == GPIO_ADC) {
            pin_function = PINMUX_PA11B_ADC_AIN19;
        }
    } else {
        configASSERT(0);
    }

    gpio_set_pin_function(pin_number, pin_function);
}