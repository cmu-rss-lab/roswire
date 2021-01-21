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
    Iterator,
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


class ROS2PackageDatabase(PackageDatabase):

    __contents: Dict[str, Any]

    @classmethod
    def build(
            cls, app_instance: "AppInstance", paths: Optional[List[str]] = None
    ) -> "ROS2PackageDatabase":
        if paths is None:
            paths = cls._determine_paths(app_instance)
        db_package = cls._from_paths(app_instance, paths)
        return db_package

    @classmethod
    def _from_paths(
            cls,
            app_instance: "AppInstance",
            paths: List[str],
            ignore_bad_paths: bool = True,
    ) -> "ROS2PackageDatabase":
        """
        Constructs a package database from a list of the paths of the packages
        belonging to the database.

        Parameters
        ----------
        app_instance: AppInstance
            an instance of an application from which to get
            the package database
        paths: List[str]
            a list of the absolute paths of the packages.
        ignore_bad_paths: bool
            If :code:`True`, non-existent paths will be ignored.
            If :code:`False`, a :exc:`FileNotFoundError` will be raised.

        Raises
        ------
        FileNotFoundError
            if no package is found at a given path.
        """
        packages: List[ROS2Package] = []
        for p in paths:
            try:
                package = ROS2Package.build(p, app_instance)
            except FileNotFoundError:
                logger.exception(f"unable to build package: {p}")
                if not ignore_bad_paths:
                    raise
            else:
                packages.append(package)
        return cls._from_packages_and_paths(packages, paths)

    @classmethod
    def _from_packages_and_paths(
            cls, packages: Iterable[ROS2Package], paths: Iterable[str]
    ) -> "ROS2PackageDatabase":
        return ROS2PackageDatabase(packages, paths)

    @classmethod
    def from_dict(cls, d: List[Dict[str, Any]]) -> "ROS2PackageDatabase":
        return ROS2PackageDatabase([ROS2Package.from_dict(dd) for dd in d], [])

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

    def __init__(
            self, packages: Iterable[ROS2Package], paths: Iterable[str]
    ) -> None:
        self.__contents = {p.name: p for p in packages}
        self._paths_in_package = list(paths)

    def __len__(self) -> int:
        """Returns the number of packages within this database."""
        return len(self.__contents)

    def __getitem__(self, name: str) -> Package:
        """Fetches the description for a given package.

        Raises
        ------
        KeyError
            if no package exists with the given name.
        """
        return self.__contents[name]

    def __iter__(self) -> Iterator[str]:
        """
        Returns an iterator over the names of the packages contained within
        this database.
        """
        yield from self.__contents
