import os
import sys
current_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(os.path.join(current_path, "..", "..", "libs"))

from simplecborrpc.api_gen import generate_api, CborTypes

generate_api(current_path, {
    "max_size": [],

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

    "openocd_jtag": [
        CborTypes.CBOR_TYPE_UNSIGNED_INTEGER,  # TDI GPIO number
        CborTypes.CBOR_TYPE_UNSIGNED_INTEGER,  # TDO GPIO number
        CborTypes.CBOR_TYPE_UNSIGNED_INTEGER,  # TMS GPIO number
        CborTypes.CBOR_TYPE_UNSIGNED_INTEGER,  # TCK GPIO number
        CborTypes.CBOR_TYPE_BYTE_STRING,       # jtag data to write
    ],

    "i2c_exchange": [
        CborTypes.CBOR_TYPE_UNSIGNED_INTEGER,  # I2C interface index
        CborTypes.CBOR_TYPE_UNSIGNED_INTEGER,  # device address (right aligned)
        CborTypes.CBOR_TYPE_BYTE_STRING,       # bytes to send (can be empty)
        CborTypes.CBOR_TYPE_UNSIGNED_INTEGER,  # number of bytes to receive
        CborTypes.CBOR_TYPE_UNSIGNED_INTEGER,  # clock rate (Hz)
        CborTypes.CBOR_TYPE_UNSIGNED_INTEGER   # transaction timeout (ms)
    ],

    "spi_exchange": [
        CborTypes.CBOR_TYPE_UNSIGNED_INTEGER,  # SPI interface index
        CborTypes.CBOR_TYPE_BYTE_STRING,       # bytes to send (can be empty)
        CborTypes.CBOR_TYPE_UNSIGNED_INTEGER,  # read size
        CborTypes.CBOR_TYPE_UNSIGNED_INTEGER,  # clock rate (Hz)
        CborTypes.CBOR_TYPE_UNSIGNED_INTEGER,  # SPI mode
        CborTypes.CBOR_TYPE_UNSIGNED_INTEGER,  # transaction timeout (ms)
        CborTypes.CBOR_TYPE_UNSIGNED_INTEGER   # output pin number to use as CS (set to 0xFF to disable)
    ],

    "pwm_set": [
        CborTypes.CBOR_TYPE_UNSIGNED_INTEGER,  # pin number
        CborTypes.CBOR_TYPE_UNSIGNED_INTEGER,  # prescaler index (from pwm_get_info)
        CborTypes.CBOR_TYPE_UNSIGNED_INTEGER,  # counter period (in ticks)
        CborTypes.CBOR_TYPE_UNSIGNED_INTEGER   # duty cycle (in ticks)
    ],

    "pwm_get_info": [],

    "dac_set": [
        CborTypes.CBOR_TYPE_UNSIGNED_INTEGER,  # pin number
        CborTypes.CBOR_TYPE_UNSIGNED_INTEGER   # DAC level
    ],

    "dac_get_info": [],

    "adc_get": [
        CborTypes.CBOR_TYPE_UNSIGNED_INTEGER,  # pin number
    ],

    "polyglot_version": [],

    "polyglot_hw": [],
})