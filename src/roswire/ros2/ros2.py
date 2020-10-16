# -*- coding: utf-8 -*-
__all__ = ('ROS2',)

import typing

import attr

from .node_manager import ROS2NodeManager
from .roslaunch.launch import ROS2LaunchManager
from .service_manager import ROS2ServiceManager
from .state import ROS2StateProbe
from ..proxy import SystemState

if typing.TYPE_CHECKING:
    from ... import AppInstance


@attr.s(frozen=True, auto_attribs=True, slots=True)
class ROS2:
    """Provides an interface to ROS2."""
    app_instance: 'AppInstance'
    nodes: ROS2NodeManager = attr.ib(init=False)
    services: ROS2ServiceManager = attr.ib(init=False)
    launch: ROS2LaunchManager = attr.ib(init=False)
    _state_probe: ROS2StateProbe = attr.ib(init=False)

    def __attrs_post_init__(self) -> None:
        nodes = ROS2NodeManager.for_app_instance(self.app_instance)
        services = ROS2ServiceManager.for_app_instance(self.app_instance)
        state_probe = ROS2StateProbe.for_app_instance(self.app_instance)
        launch = ROS2LaunchManager.for_app_instance(self.app_instance)
        object.__setattr__(self, 'nodes', nodes)
        object.__setattr__(self, 'services', services)
        object.__setattr__(self, '_state_probe', state_probe)
        object.__setattr__(self, 'launch', launch)

    @classmethod
    def for_app_instance(cls, app_instance: 'AppInstance') -> 'ROS2':
        return ROS2(app_instance=app_instance)

    @property
    def state(self) -> SystemState:
        return self._state_probe.probe()
