from typing import Dict, Union


def decode_header(b: bytes) -> Dict[str, Union[int, str]]:
    contents: Dict[str, Union[int, str]] = {}
    length_header = int.from_bytes(b[0:4], byteorder='little')
    offset = 4

    # decode each field
    while offset < length_header:
        length = int.from_bytes(b[offset:offset + 4], byteorder='little')
        offset += 4
        name, _, val = b[offset:offset + length].decode('utf-8').partition('=')
        contents[name] = val
        offset += length

    return contents
