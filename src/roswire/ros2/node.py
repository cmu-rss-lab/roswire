# -*- coding: utf-8 -*-
__all__ = ('ROS2Node',)

import typing

import attr

if typing.TYPE_CHECKING:
    from ..app import AppInstance


@attr.s(frozen=True, auto_attribs=True, slots=True)
class ROS2Node:
    """Provides an interface for interacting with a ROS2 node."""
    app_instance: 'AppInstance'
