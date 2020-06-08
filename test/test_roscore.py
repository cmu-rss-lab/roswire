# -*- coding: utf-8 -*-
import pytest

from roswire.proxy import SystemState


@pytest.mark.parametrize('sut', ['fetch'], indirect=True)
def test_state(sut):
    with sut.roscore() as ros:
        expected_pubs = {
            '/rosout_agg': ['/rosout']
        }
        expected_subs = {
            '/rosout': ['/rosout']
        }
        expected_services = {
            '/rosout/set_logger_level': ['/rosout'],
            '/rosout/get_loggers': ['/rosout']
        }
        expected = SystemState(publishers=expected_pubs,
                               subscribers=expected_subs,
                               services=expected_services)
        assert ros.state == expected
