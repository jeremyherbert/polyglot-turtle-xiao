#ifndef POLYGLOT_TURTLE_XIAO_PROJ_PINS_H
#define POLYGLOT_TURTLE_XIAO_PROJ_PINS_H

#include "sam.h"
#include <stddef.h>


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
        PIN_PA04, // PWM: TCC3/WO[2], alt function F
        PIN_PA10, // PWM: TCC1/WO[0], alt function E
        PIN_PA11  // PWM: TCC0/WO[3], alt function F
};

typedef enum {
    GPIO_NO_ALTERNATE_FUNCTION,
    GPIO_DAC,
    GPIO_PWM,
    GPIO_ADC,
    GPIO_OFF
} gpio_type_t;

int configure_gpio_function(uint32_t pin_number, gpio_type_t type);

#endif //POLYGLOT_TURTLE_XIAO_PROJ_PINS_H
