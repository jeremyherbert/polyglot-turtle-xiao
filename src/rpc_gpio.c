#include "simplecborrpc.h"
#include "pins.h"
#include "rpc_dac.h"
#include "hal/include/hal_gpio.h"

rpc_error_t
rpc_gpio_set_dir(const CborValue *args_iterator, CborEncoder *result, const char **error_msg, void *user_ptr) {
    CborValue private_args_it = *args_iterator;
    uint64_t pin_number;
    uint64_t direction;

    cbor_value_get_uint64(&private_args_it, &pin_number);
    cbor_value_advance(&private_args_it);
    cbor_value_get_uint64(&private_args_it, &direction);

    if (pin_number >= GPIO_PIN_COUNT) {
        *error_msg = "Invalid pin";
        return RPC_ERROR_INVALID_ARGS;
    }

    if (direction > 2) {
        *error_msg = "Invalid direction";
        return RPC_ERROR_INVALID_ARGS;
    }

    configure_gpio_function(gpio_pin_map[pin_number], GPIO_NO_ALTERNATE_FUNCTION);

    switch(direction) {
        case 0:
            gpio_set_pin_direction(gpio_pin_map[pin_number], GPIO_DIRECTION_IN);
            break;
        case 1:
            gpio_set_pin_direction(gpio_pin_map[pin_number], GPIO_DIRECTION_OUT);
            break;
        case 2:
            gpio_set_pin_direction(gpio_pin_map[pin_number], GPIO_DIRECTION_OFF);
            break;
        default:
            return RPC_ERROR_INTERNAL_ERROR; // this should never occur
    }

    cbor_encode_null(result);
    return RPC_OK;
}

rpc_error_t
rpc_gpio_set_pull(const CborValue *args_iterator, CborEncoder *result, const char **error_msg, void *user_ptr) {
    CborValue private_args_it = *args_iterator;
    uint64_t pin_number;
    uint64_t pull_type;

    cbor_value_get_uint64(&private_args_it, &pin_number);
    cbor_value_advance(&private_args_it);
    cbor_value_get_uint64(&private_args_it, &pull_type);

    if (pin_number >= GPIO_PIN_COUNT) {
        *error_msg = "Invalid pin";
        return RPC_ERROR_INVALID_ARGS;
    }

    if (pull_type > 2) {
        *error_msg = "Invalid pull type";
        return RPC_ERROR_INVALID_ARGS;
    }

    configure_gpio_function(gpio_pin_map[pin_number], GPIO_NO_ALTERNATE_FUNCTION);
    switch(pull_type) {
        case 0:
            gpio_set_pin_pull_mode(gpio_pin_map[pin_number], GPIO_PULL_OFF);
            break;
        case 1:
            gpio_set_pin_pull_mode(gpio_pin_map[pin_number], GPIO_PULL_UP);
            break;
        case 2:
            gpio_set_pin_pull_mode(gpio_pin_map[pin_number], GPIO_PULL_DOWN);
            break;
        default:
            return RPC_ERROR_INTERNAL_ERROR; // this should never occur
    }

    cbor_encode_null(result);
    return RPC_OK;
}

rpc_error_t
rpc_gpio_set_level(const CborValue *args_iterator, CborEncoder *result, const char **error_msg, void *user_ptr) {
    CborValue private_args_it = *args_iterator;

    uint64_t pin_number;
    bool value;

    cbor_value_get_uint64(&private_args_it, &pin_number);
    cbor_value_advance(&private_args_it);
    cbor_value_get_boolean(&private_args_it, &value);

    if (pin_number >= GPIO_PIN_COUNT) {
        *error_msg = "Invalid pin";
        return RPC_ERROR_INVALID_ARGS;
    } else {
        configure_gpio_function(gpio_pin_map[pin_number], GPIO_NO_ALTERNATE_FUNCTION);
        gpio_set_pin_level(gpio_pin_map[pin_number], value);
    }

    cbor_encode_null(result);
    return RPC_OK;
}

rpc_error_t
rpc_gpio_get_level(const CborValue *args_iterator, CborEncoder *result, const char **error_msg, void *user_ptr) {
    uint64_t pin_number;
    cbor_value_get_uint64(args_iterator, &pin_number);

    if (pin_number >= GPIO_PIN_COUNT) {
        *error_msg = "Invalid pin";
        return RPC_ERROR_INVALID_ARGS;
    }

    configure_gpio_function(gpio_pin_map[pin_number], GPIO_NO_ALTERNATE_FUNCTION);
    bool read_value = gpio_get_pin_level(gpio_pin_map[pin_number]);
    cbor_encode_boolean(result, read_value);

    return RPC_OK;
}