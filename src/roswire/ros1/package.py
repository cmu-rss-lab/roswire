# -*- coding: utf-8 -*-
__all__ = ("ROS1Package", "ROS1PackageDatabase",)

import os
import typing
from typing import Any, Collection, Dict, List
from typing import Iterable  # noqa: F401 # Needed for tuple_from_iterable

import attr
import dockerblade
from loguru import logger
from roswire.common import ActionFormat, MsgFormat, SrvFormat
from roswire.util import tuple_from_iterable

from ..common import Package, PackageDatabase

if typing.TYPE_CHECKING:
    from .. import AppInstance


@attr.s(frozen=True, auto_attribs=True, slots=True)
class ROS1Package(Package[MsgFormat, SrvFormat, ActionFormat]):
    name: str
    path: str
    messages: Collection[MsgFormat] = attr.ib(converter=tuple_from_iterable)
    services: Collection[SrvFormat] = attr.ib(converter=tuple_from_iterable)
    actions: Collection[ActionFormat] = attr.ib(converter=tuple_from_iterable)

    @classmethod
    def build(cls, path: str, app_instance: "AppInstance") -> "ROS1Package":
        """Constructs a description of a package at a given path."""
        name: str = os.path.basename(path)
        messages: List[MsgFormat] = []
        services: List[SrvFormat] = []
        actions: List[ActionFormat] = []
        files = app_instance.files

        if not files.isdir(path):
            raise FileNotFoundError(f"directory does not exist: {path}")

        dir_msg = os.path.join(path, "msg")
        dir_srv = os.path.join(path, "srv")
        dir_action = os.path.join(path, "action")

        if files.isdir(dir_msg):
            messages = [
                MsgFormat.from_file(name, f, files)
                for f in files.listdir(dir_msg, absolute=True)
                if f.endswith(".msg")
            ]
        if files.isdir(dir_srv):
            services = [
                SrvFormat.from_file(name, f, files)
                for f in files.listdir(dir_srv, absolute=True)
                if f.endswith(".srv")
            ]
        if files.isdir(dir_action):
            actions = [
                ActionFormat.from_file(name, f, files)
                for f in files.listdir(dir_action, absolute=True)
                if f.endswith(".action")
            ]

        return ROS1Package(name, path, messages, services, actions)

    @classmethod
    def from_dict(cls, d: Dict[str, Any]) -> "ROS1Package":
        name: str = d["name"]
        messages: List[MsgFormat] = [
            MsgFormat.from_dict(dd, package=name)
            for dd in d.get("messages", [])
        ]
        services: List[SrvFormat] = [
            SrvFormat.from_dict(dd, package=name)
            for dd in d.get("services", [])
        ]
        actions: List[ActionFormat] = [
            ActionFormat.from_dict(dd, package=name)
            for dd in d.get("actions", [])
        ]
        return ROS1Package(d["name"], d["path"], messages, services, actions)

    def to_dict(self) -> Dict[str, Any]:
        d = {
            "name": self.name,
            "path": self.path,
            "messages": [m.to_dict() for m in self.messages],
            "services": [s.to_dict() for s in self.services],
            "actions": [a.to_dict() for a in self.actions],
        }
        return d


class ROS1PackageDatabase(PackageDatabase[ROS1Package]):

    @classmethod
    def from_dict(cls,
                  d: List[Dict[str, Any]]
                  ) -> "PackageDatabase[ROS1Package]":
        return cls.from_packages(ROS1Package.from_dict(dd) for dd in d)

    @classmethod
    def _build_package(cls,
                       app_instance: "AppInstance",
                       path: str
                       ) -> ROS1Package:
        return ROS1Package.build(path, app_instance)

    @classmethod
    def _determine_paths(cls, app_instance: "AppInstance") -> List[str]:
        """Parses :code:`ROS_PACKAGE_PATH` for a given shell."""
        paths: List[str] = []
        shell = app_instance.shell
        files = app_instance.files
        path_str = shell.environ("ROS_PACKAGE_PATH")
        package_paths: List[str] = path_str.strip().split(":")
        for path in package_paths:
            try:
                all_packages = files.find(path, "package.xml")
            except dockerblade.exceptions.DockerBladeException:
                logger.warning(
                    "unable to find directory in ROS_PACKAGE_PATH:" f" {path}"
                )
                continue
            paths.extend(os.path.dirname(p) for p in all_packages)
        return paths
