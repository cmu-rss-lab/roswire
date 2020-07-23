# -*- coding: utf-8 -*-
__all__ = ('ROS2StateProbe',)

import attr
import typing

from ..proxy import SystemState

if typing.TYPE_CHECKING:
    from ..app import AppInstance


@attr.s(frozen=True, auto_attribs=True)
class ROS2StateProbe:
    """Provides an interface for obtaining the instantaneous state of a ROS
    system in terms of its publishers, subscribers, and services."""
    @classmethod
    def for_app_instance(cls, app_instance: 'AppInstance') -> 'ROS2StateProbe':
        raise NotImplementedError

    def probe(self) -> SystemState:
        """Obtains the instantaneous state of the associated ROS system."""
        raise NotImplementedError

    __call__ = probe
