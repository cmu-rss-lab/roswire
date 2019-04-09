# -*- coding: utf-8 -*-
"""
TODO:
    * handle special types: Time, Duration and Header
"""
import struct


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


def simple(typ: str) -> Callable[[BytesIO], Any]:
    pattern = SIMPLE_TYPE_TO_STRUCT[typ]
    size = struct.calcsize(f'<{pattern}')
    def decode(bfr: BytesIO) -> Any:
        return struct.unpack(pattern, bfr.read(size))

    def decode_bool(bfr: BytesIO) -> bool:
        return bool(decode(bfr))

    return decode_bool if typ == 'bool' else decode


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


def message(factory: Type[Message],
            fields: Sequence[Tuple[str, Callable[[BytesIO], Any]]]
            ) -> Callable[[BytesIO], Message]:
    def decode(bfr: BytesIO):
        return factory(**{n: f(bfr) for n, f in fields})
    return decode


def build(db_typ: TypeDatabase,
          typ: Type[Message],
          fmt: MsgFormat
          ) -> Callable[[BytesIO], Message]:
    field_factories: List[Tuple[str, Callable[[BytesIO], Any]]] = []
    for field in fmt.fields:
        if field.is_array:
            if is_simple(field.base_typ):
                factory = simple_array(field.base_typ, field.length)
            else:
                factory_base = db_typ[field.base_typ].decode
                factory = complex_array(factory_base, field.length)
        elif is_simple(field.typ):
            factory = simple(field.typ)
        else:
            factory = db_typ[field.typ].decode
        field_factories[field.name] = factory
    return message(typ, field_factories)
