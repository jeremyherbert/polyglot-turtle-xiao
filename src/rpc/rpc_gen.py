import os
import sys
current_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(os.path.join(current_path, "..", "..", "libs"))

from simplecborrpc.api_gen import generate_api, CborTypes

generate_api(current_path, {
    "echo": [CborTypes.CBOR_TYPE_TEXT_STRING],

    "gpio_set_dir": [
        CborTypes.CBOR_TYPE_UNSIGNED_INTEGER,  # pin number
        CborTypes.CBOR_TYPE_UNSIGNED_INTEGER   # direction (0 = input, 1 = output, 2 = disconnected)
    ],

    "gpio_set_pull": [
        CborTypes.CBOR_TYPE_UNSIGNED_INTEGER,  # pin number
        CborTypes.CBOR_TYPE_UNSIGNED_INTEGER   # pull type (0 = none, 1 = up, 2 = down)
    ],

    "gpio_set_level": [
        CborTypes.CBOR_TYPE_UNSIGNED_INTEGER,  # pin number
        CborTypes.CBOR_TYPE_BOOL               # level
    ],

    "gpio_get_level": [
        CborTypes.CBOR_TYPE_UNSIGNED_INTEGER   # pin number
    ],

    "i2c_exchange": [
        CborTypes.CBOR_TYPE_UNSIGNED_INTEGER,  # device address (right aligned)
        CborTypes.CBOR_TYPE_BYTE_STRING,       # bytes to send (can be empty)
        CborTypes.CBOR_TYPE_UNSIGNED_INTEGER,  # number of bytes to receive
        CborTypes.CBOR_TYPE_UNSIGNED_INTEGER,  # clock rate (Hz)
        CborTypes.CBOR_TYPE_UNSIGNED_INTEGER   # transaction timeout (ms)
    ],

    "spi_exchange": [
        CborTypes.CBOR_TYPE_BYTE_STRING,       # bytes to send (can be empty)
        CborTypes.CBOR_TYPE_UNSIGNED_INTEGER,  # number of bytes to receive
        CborTypes.CBOR_TYPE_UNSIGNED_INTEGER   # clock rate (Hz)
    ]
})