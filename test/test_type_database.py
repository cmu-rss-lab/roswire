import pytest
import attr

from rozzy.definitions import TypeDatabase, MsgFormat, Time

from test_file import build_file_proxy


def test_build_type():
    s = """
uint32 seq
time stamp
string frame_id
    """
    fmt = MsgFormat.from_string("PkgName", "MessageName", s)
    t = TypeDatabase.build_type(fmt)
    assert t.__name__ == fmt.name
    assert t.format == fmt

    m = t(seq=310, stamp=Time(30, 120), frame_id='foo')

    # message should be immutable
    with pytest.raises(attr.exceptions.FrozenInstanceError):
        m.seq = 500
