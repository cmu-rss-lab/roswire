import pytest

from rozzy.proxy.rostopic import ROSTopicProxy

from test_basic import build_ardu


def test_pub():
    with build_ardu() as (sut, ros):
        desc = sut.description
        p = ROSTopicProxy(desc, sut.shell)
        t = desc.types['mavros_msgs/OverrideRCIn']
        m = t([100, 200, 300, 400, 500, 600, 700, 800])
        p.publish('/mavros/rc/override', m)
