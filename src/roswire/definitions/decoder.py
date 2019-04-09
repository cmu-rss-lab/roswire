SIMPLE_TYPE_TO_STRUCT = {
    'int8': 'b',
    'uint8': 'B',
    'bool': 'B',
    'int16': 'h',
    'uint16': 'H',
    'int32': 'i',
    'uint32': 'I',
    'int64': 'q',
    'uint64': 'Q',
    'float32': 'f',
    'float64': 'd',
    # deprecated types
    'char': 'B',  # unsigned
    'byte': 'b'  # signed
}


def is_simple(typ: str) -> bool:
    """Determines whether a given type is simple."""
    return typ in SIMPLE_TYPE_TO_STRUCT.keys()


def get_struct_pattern(typs: List[str]) -> str:
    pass


def simple_array(base_typ: str,
                 size: Optional[int] = None
                 ) -> Callable[[BytesIO], List[Any]]:
    # treat char and uint8 arrays as special strings

    if size is not None:
        def get_size(bfr: BytesIO) -> int:
            return size
    else:
        def get_size(bfr: BytesIO) -> int:
            return read_uint32(bfr)

    # obtain a struct pattern for the base type
    if is_simple(base_type):
        size = get_size()
        pattern = f'{size}{get_struct_pattern([base_type])}'
        num_bytes = struct.calcsize(f'<{pattern}')
        contents = unpack(pattern, bfr.read(num_bytes))

    # if boolean, transform from byte to boolean
    contents  = [bool(e) for e in contents]


    return decode


def complex_array(factory,
                  size: Optional[int] = None
                  ) -> Callable[[BytesIO], List[Any]]:
    def decode_fixed(bfr: BytesIO) -> List[Any]:
        return [factory(bfr) for i in range(size)]

    def decode_variable(bfr: BytesIO) -> List[Any]:
        s = read_uint32(bfr)
        return [factory(bfr) for i in range(s)]

    return decode_fixed if size is not None else decode_variable


def string(base_type: str, size: Optional[int] = None):
    def decode_fixed(bfr: BytesIO) -> str:
        # apparently we still read a size var in some circumstances
        return bfr.read(size).decode('utf-8')

    def decode_variable(bfr: BytesIO) -> str:
        size = read_uint32(bfr)
        if base_type in ['uint8', 'char']:
            return bfr.read(size)
        else:
            return bfr.read(size).decode('utf-8')

    return decode_fixed if size is not None else decode_variable


def complex(**fields):
    def decode(bfr: BytesIO):
        return
    return decode
