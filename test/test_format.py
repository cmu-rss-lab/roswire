from typing import List

import pytest

from roswire.proxy import FileProxy
from roswire.definitions import (Constant, Field, MsgFormat, SrvFormat,
                                 ActionFormat, Time, Package, PackageDatabase,
                                 FormatDatabase)
import roswire.exceptions

from test_file import build_file_proxy


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


def test_action_from_string():
    s = """
# Define the goal
uint32 dishwasher_id  # Specify which dishwasher we want to use
---
# Define the result
uint32 total_dishes_cleaned
---
# Define a feedback message
float32 percent_complete
"""

    fmt = ActionFormat.from_string("PkgName", "MessageName", s)
    assert fmt.name == "MessageName"
    assert fmt.package == "PkgName"

    # check goal
    goal = fmt.goal
    assert goal is not None
    assert not goal.constants
    assert len(goal.fields) == 1
    assert Field('uint32', 'dishwasher_id') in goal.fields

    # check result
    res = fmt.result
    assert res is not None
    assert not res.constants
    assert len(res.fields) == 1
    assert Field('uint32', 'total_dishes_cleaned') in res.fields

    # check feedback
    feedback = fmt.feedback
    assert feedback is not None
    assert not feedback.constants
    assert len(feedback.fields) == 1
    assert Field('float32', 'percent_complete') in feedback.fields


def test_field_to_and_from_dict():
    d = {'type': 'uint32', 'name': 'foo'}
    f = Field('uint32', 'foo')
    assert Field.from_dict(d) == Field('uint32', 'foo')
    assert Field.from_dict(f.to_dict()) == f


def test_constant_to_and_from_dict():
    d = {'type': 'uint32', 'name': 'foo', 'value': '100'}
    c = Constant('uint32', 'foo', '100')
    assert Constant.from_dict(d) == c
    assert Constant.from_dict(c.to_dict()) == c


def test_msg_format_to_and_from_dict():
    d = {'package': 'tf',
         'name': 'tfMessage',
         'fields': [
             {'type': 'geometry_msgs/TransformStamped[]',
              'name': 'transforms'}]}
    f = MsgFormat(package='tf',
                  name='tfMessage',
                  constants=[],
                  fields=[Field('geometry_msgs/TransformStamped[]', 'transforms')])
    assert MsgFormat.from_dict(d) == f
    assert MsgFormat.from_dict(f.to_dict()) == f


def test_srv_format_to_and_from_dict():
    pkg = 'nav_msgs'
    name = 'SetMap'
    name_request = 'SetMapRequest'
    name_response = 'SetMapResponse'
    d = {'package': pkg,
         'name': name,
         'request': {
            'fields': [
                {'type': 'nav_msgs/OccupancyGrid',
                 'name': 'map'},
                {'type': 'geometry_msgs/PoseWithCovarianceStamped',
                 'name': 'initial_pose'}]
         },
         'response': {
            'fields': [{'type': 'bool', 'name': 'success'}]
         }}
    f = SrvFormat(
            package=pkg,
            name=name,
            request=MsgFormat(
                package=pkg,
                name=name_request,
                constants=[],
                fields=[Field('nav_msgs/OccupancyGrid', 'map'),
                        Field('geometry_msgs/PoseWithCovarianceStamped',
                              'initial_pose')]),
            response=MsgFormat(
                package=pkg,
                name=name_response,
                constants=[],
                fields=[Field('bool', 'success')]))
    assert SrvFormat.from_dict(d) == f
    assert SrvFormat.from_dict(f.to_dict()) == f


def test_action_format_to_and_from_dict():
    pkg = 'actionlib'
    name = 'TwoInts'
    name_goal = 'TwoIntsGoal'
    name_result = 'TwoIntsResult'
    d = {'package': pkg,
         'name': name,
         'goal': {
            'fields': [{'type': 'int64', 'name': 'a'},
                       {'type': 'int64', 'name': 'b'}]
         },
         'result': {
            'fields': [{'type': 'int64', 'name': 'sum'}]
         }}
    f = ActionFormat(
            package=pkg,
            name=name,
            goal=MsgFormat(
                package=pkg,
                name=name_goal,
                constants=[],
                fields=[Field('int64', 'a'),
                        Field('int64', 'b')]),
            feedback=None,
            result=MsgFormat(
                package=pkg,
                name=name_result,
                constants=[],
                fields=[Field('int64', 'sum')]))
    assert ActionFormat.from_dict(d) == f
    assert ActionFormat.from_dict(f.to_dict()) == f


def test_action_from_file():
    with build_file_proxy() as files:
        # read .action file
        pkg = 'tf2_msgs'
        fn = '/ros_ws/src/geometry2/tf2_msgs/action/LookupTransform.action'
        fmt = ActionFormat.from_file(pkg, fn, files)
        assert fmt.package == pkg
        assert fmt.name == 'LookupTransform'
        assert fmt.fullname == 'tf2_msgs/LookupTransform'

        goal: MsgFormat = fmt.goal
        assert not goal.constants
        assert len(goal.fields) == 7
        assert Field('string', 'target_frame') in goal.fields
        assert Field('string', 'source_frame') in goal.fields
        assert Field('time', 'source_time') in goal.fields
        assert Field('duration', 'timeout') in goal.fields
        assert Field('time', 'target_time') in goal.fields
        assert Field('string', 'fixed_frame') in goal.fields
        assert Field('bool', 'advanced') in goal.fields

        assert fmt.result
        res: MsgFormat = fmt.result
        assert not res.constants
        assert len(res.fields) == 2
        assert Field('geometry_msgs/TransformStamped', 'transform') in res.fields
        assert Field('tf2_msgs/TF2Error', 'error') in res.fields

        assert not fmt.feedback

        # attempt to read .msg file
        fn = '/ros_ws/src/geometry2/tf2_msgs/msg/TFMessage.msg'
        with pytest.raises(AssertionError):
            ActionFormat.from_file(pkg, fn, files)

        # attempt to read non-existent file
        fn = '/ros_ws/src/geometry2/tf2_msgs/action/Spooky.action'
        with pytest.raises(FileNotFoundError):
            ActionFormat.from_file(pkg, fn, files)


def test_srv_from_file():
    with build_file_proxy() as files:
        # read .srv file
        pkg = 'nav_msgs'
        fn = '/ros_ws/src/common_msgs/nav_msgs/srv/SetMap.srv'
        fmt = SrvFormat.from_file(pkg, fn, files)
        assert fmt.package == pkg
        assert fmt.name == 'SetMap'
        assert fmt.fullname == 'nav_msgs/SetMap'

        req: MsgFormat = fmt.request
        assert not req.constants
        assert len(req.fields) == 2
        assert Field('nav_msgs/OccupancyGrid', 'map') in req.fields
        assert Field('geometry_msgs/PoseWithCovarianceStamped', 'initial_pose') in req.fields

        assert fmt.response
        res: MsgFormat = fmt.response
        assert not res.constants
        assert len(res.fields) == 1
        assert Field('bool', 'success') in res.fields

        # attempt to read .action file
        fn = '/ros_ws/src/geometry2/tf2_msgs/action/LookupTransform.action'
        with pytest.raises(AssertionError):
            SrvFormat.from_file(pkg, fn, files)

        # attempt to read non-existent file
        fn = '/ros_ws/src/common_msgs/nav_msgs/srv/Spooky.srv'
        with pytest.raises(FileNotFoundError):
            SrvFormat.from_file(pkg, fn, files)


def test_msg_from_file():
    with build_file_proxy() as files:
        # read .msg file
        pkg = 'tf2_msgs'
        fn = '/ros_ws/src/geometry2/tf2_msgs/msg/TFMessage.msg'
        fmt = MsgFormat.from_file(pkg, fn, files)
        assert fmt.package == pkg
        assert fmt.name == 'TFMessage'
        assert fmt.fullname == 'tf2_msgs/TFMessage'
        assert not fmt.constants
        assert len(fmt.fields) == 1
        assert Field('geometry_msgs/TransformStamped[]', 'transforms') in fmt.fields

        # attempt to read .action file
        fn = '/ros_ws/src/geometry2/tf2_msgs/action/LookupTransform.action'
        with pytest.raises(AssertionError):
            SrvFormat.from_file(pkg, fn, files)

        # attempt to read non-existent file
        fn = '/ros_ws/src/geometry2/tf2_msgs/msg/Spooky.msg'
        with pytest.raises(FileNotFoundError):
            MsgFormat.from_file(pkg, fn, files)


def test_build_format_database():
    with build_file_proxy() as files:
        paths = [
            '/ros_ws/src/geometry2/tf2_msgs',
            '/ros_ws/src/geometry/tf'
        ]
        db_package = PackageDatabase.from_paths(files, paths)
        db_format = FormatDatabase.build(db_package)
        name_messages: Set[str] = set(db_format.messages)
        name_services: Set[str] = set(db_format.services)
        name_actions: Set[str] = set(db_format.actions)
        assert name_messages == {
            'tf/tfMessage',
            'tf/FrameGraphResponse',
            'tf2_msgs/TFMessage',
            'tf2_msgs/TF2Error',
            'tf2_msgs/FrameGraphResponse',
            'tf2_msgs/LookupTransformGoal',
            'tf2_msgs/LookupTransformResult'
        }
        assert name_services == {
            'tf/FrameGraph',
            'tf2_msgs/FrameGraph'
        }
        assert name_actions == {'tf2_msgs/LookupTransform'}


def test_msg_toposort():
    with build_file_proxy() as files:
        paths = [
            '/ros_ws/src/geometry2/tf2_msgs',
            '/ros_ws/src/geometry/tf',
            '/ros_ws/src/common_msgs/geometry_msgs',
            '/ros_ws/src/std_msgs'
        ]
        db_package = PackageDatabase.from_paths(files, paths)
        db_format = FormatDatabase.build(db_package)

        msgs = db_format.messages.values()
        msgs = MsgFormat.toposort(msgs)


def test_msg_flatten():
    register: Dict[str, MsgFormat] = {}

    def mf(name: str, definition: str) -> MsgFormat:
        f = MsgFormat.from_string('example_pkg', name, definition)
        register[f.fullname] = f
        return f

    def names(fmt: MsgFormat) -> List[str]:
        buff = []
        for ctx, field in fmt.flatten(register):
            name = field.name
            if ctx:
                name = f"{'.'.join(ctx)}.{name}"
            buff.append(name)
        return buff

    f1 = mf("Item", """
uint32 foo
duration    lifespan
    """)
    f2 = mf("Container", """
Item        prize
Item[]      contents
time        stamp
    """)

    assert names(f1) == ['foo', 'lifespan']
    assert names(f2) == ['prize.foo', 'prize.lifespan', 'contents', 'stamp']
