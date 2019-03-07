import pytest

from rozzy.definitions import (Constant, Field, MsgFormat, SrvFormat,
                               ActionFormat)
import rozzy.exceptions


def test_msg_from_string():
    s = """
    #Standard metadata for higher-level flow data types
    #sequence ID: consecutively increasing ID
    uint32 seq
    #Two-integer timestamp that is expressed as:
    # * stamp.secs: seconds (stamp_secs) since epoch
    # * stamp.nsecs: nanoseconds since stamp_secs
    # time-handling sugar is provided by the client library
    time stamp
    #Frame this data is associated with
    string frame_id

    int32 X=123
    int32 Y=-123
    string FOO=foo
    string EXAMPLE="#comments" are ignored, and leading and trailing whitespace removed
    """
    fmt = MsgFormat.from_string("PkgName", "MessageName", s)
    assert fmt.name == "MessageName"
    assert fmt.package == "PkgName"

    assert len(fmt.constants) == 4
    assert Constant('int32', 'X', '123') in fmt.constants
    assert Constant('int32', 'Y', '-123') in fmt.constants
    assert Constant('string', 'FOO', 'foo') in fmt.constants
    assert Constant('string', 'EXAMPLE', '"#comments" are ignored, and leading and trailing whitespace removed') in fmt.constants  # noqa: pycodestyle

    assert len(fmt.fields) == 3
    assert Field('uint32', 'seq') in fmt.fields
    assert Field('time', 'stamp') in fmt.fields
    assert Field('string', 'frame_id') in fmt.fields
