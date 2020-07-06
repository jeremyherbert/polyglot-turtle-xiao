#ifndef POLYGLOT_TURTLE_XIAO_PROJ_LEDS_H
#define POLYGLOT_TURTLE_XIAO_PROJ_LEDS_H

#include <stdint.h>

#define LED_TX_IDX 0
#define LED_RX_IDX 1
#define LED_BUSY_IDX 2

void set_led_counter(uint8_t index, uint32_t value);
void led_blink_init();

#endif //POLYGLOT_TURTLE_XIAO_PROJ_LEDS_H
