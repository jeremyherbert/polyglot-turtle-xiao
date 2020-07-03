import os
import sys
current_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(os.path.join(current_path, "..", "..", "libs"))

from simplecborrpc.api_gen import generate_api, CborTypes

generate_api(current_path, {
    "echo": [CborTypes.CBOR_TYPE_TEXT_STRING],
})