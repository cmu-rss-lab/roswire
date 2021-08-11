# -*- coding: utf-8 -*-
__all__ = ("ROS2",)

import typing

import attr

from .launch import ROS2LaunchManager
from .node_manager import ROS2NodeManager
from .service_manager import ROS2ServiceManager
from .source import ROS2PackageSourceExtractor
from .state import ROS2StateProbe, ROS2SystemState
from ..common import NodeSourceInfo

if typing.TYPE_CHECKING:
    from .. import AppInstance


@attr.s(frozen=True, auto_attribs=True, slots=True)
class ROS2:
    """Provides an interface to ROS2."""

    app_instance: "AppInstance"
    nodes: ROS2NodeManager = attr.ib(init=False)
    services: ROS2ServiceManager = attr.ib(init=False)
    launch: ROS2LaunchManager = attr.ib(init=False)
    _state_probe: ROS2StateProbe = attr.ib(init=False)
    __package_source_extractor: ROS2PackageSourceExtractor = \
        attr.ib(init=False)

    def __attrs_post_init__(self) -> None:
        nodes = ROS2NodeManager.for_app_instance(self.app_instance)
        services = ROS2ServiceManager.for_app_instance(self.app_instance)
        state_probe = ROS2StateProbe.for_app_instance(self.app_instance)
        launch = ROS2LaunchManager.for_app_instance(self.app_instance)
        package_source_extractor = ROS2PackageSourceExtractor(
            self.app_instance.files
        )
        object.__setattr__(self, "nodes", nodes)
        object.__setattr__(self, "services", services)
        object.__setattr__(self, "_state_probe", state_probe)
        object.__setattr__(self, "launch", launch)
        object.__setattr__(self,
                           "__package_source_extractor",
                           package_source_extractor)

    @classmethod
    def for_app_instance(cls, app_instance: "AppInstance") -> "ROS2":
        return ROS2(app_instance=app_instance)

    @property
    def state(self) -> ROS2SystemState:
        return self._state_probe.probe()

    def get_node_sources_for_source_package(
        self,
        package_path: str
    ) -> typing.Mapping[str, NodeSourceInfo]:
        """
        Extracts the node -> source files mapping for the package with the
        source in ``package_path''

        Parameters
        ----------
        package_path: str
            The path on the container filesystem that contains the package
            source

        Returns
        -------
        Mapping[str, NodeSourceInfo]
            A (possibly empty) mapping between node names provided by the
            package and their source information
        """
        return self.__package_source_extractor.extract_source_for_package(
            package_path
        )
