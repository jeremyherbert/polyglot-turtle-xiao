#include "pins.h"
#include "FreeRTOS.h"
#include "task.h"
#include "string.h"
#include "hal/include/hal_gpio.h"

static size_t led_lookup[3] = {LED_TX_PIN, LED_RX_PIN, LED_BUSY_PIN};
static uint32_t led_counters[3];

#define LED_BLINK_STACK_SIZE 128
StackType_t  led_blink_stack[128];
StaticTask_t led_blink_taskdef;
TaskHandle_t led_blink_task_handle;

void set_led_counter(uint8_t index, uint32_t value) {
    configASSERT(index < 3);
    led_counters[index] = value;

    if (value) {
        xTaskNotify(led_blink_task_handle, 1, eSetBits);
    }
}

void led_blink_task(void *param) {
    while (1) {
        bool toggle = false;

        for (int i=0; i<3; i++) {
            gpio_set_pin_level(led_lookup[i], true);
        }
        uint32_t notification_value;
        xTaskNotifyWait(1, 1, &notification_value, portMAX_DELAY);
        while (1) {
            bool finished = true;

            for (int i=0; i<3; i++) {
                if (led_counters[i]) {
                    gpio_set_pin_level(led_lookup[i], toggle);
                    led_counters[i]--;
                    finished = false;
                } else {
                    gpio_set_pin_level(led_lookup[i], true);
                }
            }
            toggle = !toggle;

            if (finished) break;

            vTaskDelay(50/portTICK_PERIOD_MS);
        }
    }
}

void led_blink_init() {
    memset(led_counters, 0, 3);
    led_blink_task_handle = xTaskCreateStatic(led_blink_task,
                                              "led",
                                              LED_BLINK_STACK_SIZE,
                                              NULL,
                                              configMAX_PRIORITIES-3,
                                              led_blink_stack,
                                              &led_blink_taskdef);
}