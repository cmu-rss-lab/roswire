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


def test_srv_from_string():
    s = """
#request constants
int8 FOO=1
int8 BAR=2
#request fields
int8 foobar
another_pkg/AnotherMessage msg
---
#response constants
uint32 SECRET=123456
#response fields
another_pkg/YetAnotherMessage val
CustomMessageDefinedInThisPackage value
uint32 an_integer
    """

    fmt = SrvFormat.from_string("PkgName", "MessageName", s)
    assert fmt.name == "MessageName"
    assert fmt.package == "PkgName"

    # check request
    req = fmt.request
    assert req is not None

    assert len(req.constants) == 2
    assert Constant('int8', 'FOO', '1') in req.constants
    assert Constant('int8', 'BAR', '2') in req.constants

    assert len(req.fields) == 2
    assert Field('int8', 'foobar') in req.fields
    assert Field('another_pkg/AnotherMessage', 'msg') in req.fields

    # check response
    res = fmt.response
    assert res is not None

    assert len(res.constants) == 1
    assert Constant('uint32', 'SECRET', '123456') in res.constants

    assert len(res.fields) == 3
    assert Field('another_pkg/YetAnotherMessage', 'val') in res.fields
    assert Field('CustomMessageDefinedInThisPackage', 'value') in res.fields
    assert Field('uint32', 'an_integer') in res.fields
