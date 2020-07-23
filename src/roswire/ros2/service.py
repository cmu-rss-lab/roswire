# -*- coding: utf-8 -*-
__all__ = ('ROS2Service',)

import typing

import attr

if typing.TYPE_CHECKING:
    from ..app import AppInstance


@attr.s(frozen=True, auto_attribs=True, slots=True)
class ROS2Service:
    """Provides an interface for interacting with a ROS2 service.

    Attributes
    ----------
    name: str
        The name of the service.
    """
    app_instance: 'AppInstance'
