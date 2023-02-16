#include "simplecborrpc.h"
#include "pins.h"
#include "hal/include/hal_gpio.h"
#include "hid_rpc.h"
#include "delay.h"

rpc_error_t
rpc_openocd_jtag(const CborValue *args_iterator, CborEncoder *result, const char **error_msg, void *user_ptr) {
    CborValue private_args_it = *args_iterator;
    size_t command_count = 0;
    size_t read_count = 0;

    uint64_t tdi_pin, tdo_pin, tms_pin, tck_pin;

    cbor_value_get_uint64(&private_args_it, &tdi_pin);
    cbor_value_advance(&private_args_it);
    cbor_value_get_uint64(&private_args_it, &tdo_pin);
    cbor_value_advance(&private_args_it);
    cbor_value_get_uint64(&private_args_it, &tms_pin);
    cbor_value_advance(&private_args_it);
    cbor_value_get_uint64(&private_args_it, &tck_pin);
    cbor_value_advance(&private_args_it);

    if (tdi_pin > 3 || tdo_pin > 3 || tms_pin > 3 || tck_pin > 3) {
        *error_msg = "Invalid pin numbers";
        return RPC_ERROR_INVALID_ARGS;
    }

    size_t length;
    cbor_value_calculate_string_length(&private_args_it, &length);
    if (length > I2C_SPI_TRANSACTION_BUFFER_SIZE) {
        *error_msg = "Command buffer too large";
        return RPC_ERROR_INVALID_ARGS;
    } else {
        command_count = length;
    }
    length = I2C_SPI_TRANSACTION_BUFFER_SIZE;

    cbor_value_copy_byte_string(&private_args_it, transaction_buffer, &length, &private_args_it);

    for (size_t i=0; i<command_count; i++) {
        uint8_t command = transaction_buffer[i];
        uint8_t number;

        switch (command) {
            case 'R':
                if (gpio_get_pin_level(gpio_pin_map[tdo_pin])) {
                    transaction_buffer[read_count++] = '1';
                } else {
                    transaction_buffer[read_count++] = '0';
                }
                break;

            case '0':
            case '1':
            case '2':
            case '3':
            case '4':
            case '5':
            case '6':
            case '7':
                number = command - 48;
                gpio_set_pin_level(gpio_pin_map[tdi_pin], number & (1 << 0));
                gpio_set_pin_level(gpio_pin_map[tms_pin], number & (1 << 1));
                gpio_set_pin_level(gpio_pin_map[tck_pin], number & (1 << 2));
                break;

            case 'B':
            case 'b':
            case 'Q':
            case 'r':
            case 's':
            case 't':
            case 'u':
            case 'v':
                break;

            case 'd':
                ddDelay_us(1);
                break;

            case 'D':
                ddDelay_ms(1);
                break;

            default:
                *error_msg = "Invalid openocd JTAG command";
                return RPC_ERROR_INVALID_ARGS;
        }
    }

    cbor_encode_byte_string(result, transaction_buffer, read_count);
    return RPC_OK;
}