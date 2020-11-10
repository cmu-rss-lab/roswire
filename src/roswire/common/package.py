# -*- coding: utf-8 -*-
__all__ = ("Package", "PackageDatabase")

import os
import typing
from abc import ABC, abstractmethod
from typing import (Any, Dict, Iterable, Iterator, List,
                    Mapping, Optional, Tuple)

import attr
from loguru import logger

from .action import ActionFormat
from .msg import MsgFormat
from .srv import SrvFormat
from ..util import tuple_from_iterable

if typing.TYPE_CHECKING:
    from .. import AppInstance


@attr.s(frozen=True, auto_attribs=True, slots=True)
class Package:
    name: str
    path: str
    messages: Tuple[MsgFormat, ...] = attr.ib(converter=tuple_from_iterable)
    services: Tuple[SrvFormat, ...] = attr.ib(converter=tuple_from_iterable)
    actions: Tuple[ActionFormat, ...] = attr.ib(converter=tuple_from_iterable)

    @staticmethod
    def build(path: str, app_instance: "AppInstance") -> "Package":
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

        return Package(name, path, messages, services, actions)

    @staticmethod
    def from_dict(d: Dict[str, Any]) -> "Package":
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
        return Package(d["name"], d["path"], messages, services, actions)

    def to_dict(self) -> Dict[str, Any]:
        d = {
            "name": self.name,
            "path": self.path,
            "messages": [m.to_dict() for m in self.messages],
            "services": [s.to_dict() for s in self.services],
            "actions": [a.to_dict() for a in self.actions],
        }
        return d


class PackageDatabase(ABC, Mapping[str, Package]):
    """
    An immutable database of packages, represented as :class:`Package`
    instances, indexed by their names, given as :class:`str`.

    Note
    ----
        Implements most :class:`dict` operations via
        :class:`abc.collections.Mapping`,
        including :code:`db['name']`, :code:`len(db)`), :code:`db.keys()`,
        :code:`db.values()`, and :code:`iter(db)`.
        As instances of this class are immutable, no destructive
        :class:`dict` operations are provided (e.g., :code:`del db['foo'])`
        and `db['foo'] = bar`).
    """

    @classmethod
    def build(
        cls, app_instance: "AppInstance", paths: Optional[List[str]] = None
    ) -> "PackageDatabase":
        if paths is None:
            paths = cls._determine_paths(app_instance)
        db_package = cls._from_paths(app_instance, paths)
        return db_package

    @classmethod
    @abstractmethod
    def _determine_paths(cls, app_instance: "AppInstance") -> List[str]:
        """Parses the package paths for a given shell.

        Parameters
        ----------
        app_instance: AppInstance
            An instance of the application for which the
            list of paths should be obtained
        """
        ...

    @classmethod
    @abstractmethod
    def _from_packages_and_paths(
        cls, packages: Iterable[Package], paths: Iterable[str]
    ) -> "PackageDatabase":
        """
        Constructs a package database from a packages
        and paths in the container

        Parameters
        ----------
        packages: Iterable[Package]
            A collection of the packages to be included in the databse
        paths: Iterable[str]
            A collection of paths
        """
        ...

    @classmethod
    def _from_paths(
        cls,
        app_instance: "AppInstance",
        paths: List[str],
        ignore_bad_paths: bool = True,
    ) -> "PackageDatabase":
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
        packages: List[Package] = []
        for p in paths:
            try:
                package = Package.build(p, app_instance)
            except FileNotFoundError:
                logger.exception(f"unable to build package: {p}")
                if not ignore_bad_paths:
                    raise
            else:
                packages.append(package)
        return cls._from_packages_and_paths(packages, paths)

    def __init__(
        self, packages: Iterable[Package], paths: Iterable[str]
    ) -> None:
        self.__contents = {p.name: p for p in packages}
        self._paths_in_package = list(paths)

    @property
    def paths(self) -> List[str]:
        return self._paths_in_package

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

    @classmethod
    @abstractmethod
    def from_dict(cls, d: List[Dict[str, Any]]) -> "PackageDatabase":
        ...

    def to_dict(self) -> List[Dict[str, Any]]:
        return [p.to_dict() for p in self.values()]
