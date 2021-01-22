# -*- coding: utf-8 -*-
__all__ = ("ROS2PackageDatabase",)

import json
import os
import typing
from typing import (
    Any,
    Collection,
    Dict,
    Iterable,
    List,
    Mapping,
    Optional,
)

import attr
import dockerblade
from loguru import logger
from typing_extensions import Final

from ..common import (ActionFormat,
                      MsgFormat,
                      Package,
                      PackageDatabase,
                      SrvFormat)
from ..util import tuple_from_iterable

if typing.TYPE_CHECKING:
    from .. import AppInstance

_COMMAND_ROS2_PKG_PREFIXES: Final[str] = (
    "python3 -c '"
    "import json; "
    "import ament_index_python; "
    "print(json.dumps(ament_index_python.get_packages_with_prefixes()))"
    "'"
)


@attr.s(frozen=True, auto_attribs=True, slots=True)
class ROS2Package(Package[MsgFormat, SrvFormat, ActionFormat]):
    name: str
    path: str
    messages: Collection[MsgFormat] = attr.ib(converter=tuple_from_iterable)
    services: Collection[SrvFormat] = attr.ib(converter=tuple_from_iterable)
    actions: Collection[ActionFormat] = attr.ib(converter=tuple_from_iterable)

    @classmethod
    def build(cls, path: str, app_instance: "AppInstance") -> "ROS2Package":
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

        return ROS2Package(name, path, messages, services, actions)

    @classmethod
    def from_dict(cls, d: Dict[str, Any]) -> "ROS2Package":
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
        return ROS2Package(d["name"], d["path"], messages, services, actions)


class ROS2PackageDatabase(PackageDatabase[ROS2Package]):

    @classmethod
    def from_dict(cls, d: List[Dict[str, Any]]) -> "PackageDatabase[ROS2Package]":
        return cls.from_packages(ROS2Package.from_dict(dd) for dd in d)

    @classmethod
    def _build_package(cls, app_instance: "AppInstance", path: str) -> ROS2Package:
        return ROS2Package.build(path, app_instance)

    @classmethod
    def _determine_paths(cls, app_instance: "AppInstance") -> List[str]:
        """Returns a list of paths for all ROS2 packages in an application."""
        try:
            shell = app_instance.shell
            jsn = shell.check_output(_COMMAND_ROS2_PKG_PREFIXES, text=True)
        except dockerblade.exceptions.CalledProcessError:
            logger.error("failed to obtain ROS2 package prefixes")
            raise
        package_to_prefix: Mapping[str, str] = json.loads(jsn)
        paths: List[str] = [
            os.path.join(prefix, f"share/{package}")
            for (package, prefix) in package_to_prefix.items()
        ]
        return paths


