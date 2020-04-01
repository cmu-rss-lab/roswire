# -*- coding: utf-8 -*-
__all__ = ('Package', 'PackageDatabase')

from typing import (Tuple, List, Dict, Union, Any, Iterator, Collection,
                    Mapping, Callable, Iterable)
import os

from loguru import logger
import attr
import dockerblade
import shlex

from .msg import MsgFormat
from .srv import SrvFormat
from .action import ActionFormat
from ..util import tuple_from_iterable


@attr.s(frozen=True, auto_attribs=True, slots=True)
class Package:
    name: str
    path: str
    messages: Tuple[MsgFormat, ...] = attr.ib(converter=tuple_from_iterable)
    services: Tuple[SrvFormat, ...] = attr.ib(converter=tuple_from_iterable)
    actions: Tuple[ActionFormat, ...] = attr.ib(converter=tuple_from_iterable)

    @staticmethod
    def build(path: str, files: dockerblade.FileSystem) -> 'Package':
        """Constructs a description of a package at a given path."""
        name: str = os.path.basename(path)
        messages: List[MsgFormat] = []
        services: List[SrvFormat] = []
        actions: List[ActionFormat] = []

        if not files.isdir(path):
            raise FileNotFoundError(f"directory does not exist: {path}")

        dir_msg = os.path.join(path, 'msg')
        dir_srv = os.path.join(path, 'srv')
        dir_action = os.path.join(path, 'action')

        if files.isdir(dir_msg):
            messages = [MsgFormat.from_file(name, f, files)
                        for f in files.listdir(dir_msg, absolute=True)
                        if f.endswith('.msg')]
        if files.isdir(dir_srv):
            services = [SrvFormat.from_file(name, f, files)
                        for f in files.listdir(dir_srv, absolute=True)
                        if f.endswith('.srv')]
        if files.isdir(dir_action):
            actions = [ActionFormat.from_file(name, f, files)
                       for f in files.listdir(dir_action, absolute=True)
                       if f.endswith('.action')]

        return Package(name,  # type: ignore
                       path,
                       messages,
                       services,
                       actions)

    @staticmethod
    def from_dict(d: Dict[str, Any]) -> 'Package':
        name: str = d['name']
        messages: List[MsgFormat] = [MsgFormat.from_dict(dd, package=name)
                                     for dd in d.get('messages', [])]
        services: List[SrvFormat] = [SrvFormat.from_dict(dd, package=name)
                                     for dd in d.get('services', [])]
        actions: List[ActionFormat] = [ActionFormat.from_dict(dd, package=name)
                                       for dd in d.get('actions', [])]
        return Package(d['name'],  # type: ignore
                       d['path'],
                       messages,
                       services,
                       actions)

    def to_dict(self) -> Dict[str, Any]:
        d = {'name': self.name,
             'path': self.path,
             'messages': [m.to_dict() for m in self.messages],
             'services': [s.to_dict() for s in self.services],
             'actions': [a.to_dict() for a in self.actions]}
        return d


class PackageDatabase(Mapping[str, Package]):
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
    @staticmethod
    def paths(shell: dockerblade.Shell,
              files: dockerblade.FileSystem
              ) -> List[str]:
        """Parses :code:`ROS_PACKAGE_PATH` for a given shell."""
        path_str = shell.environ('ROS_PACKAGE_PATH')
        package_paths: List[str] = path_str.strip().split(':')
        paths: List[str] = []
        for path in package_paths:
            try:
                all_packages = files.find(path, 'package.xml')
            except dockerblade.exceptions.DockerBladeException:
                logger.warning('unable to find directory in ROS_PACKAGE_PATH:'
                               f' {path}')
                continue
            package_dirs = [os.path.dirname(p) for p in all_packages]
            paths.extend(package_dirs)
        return paths

    @staticmethod
    def from_paths(files: dockerblade.FileSystem,
                   paths: List[str],
                   ignore_bad_paths: bool = True
                   ) -> 'PackageDatabase':
        """
        Constructs a package database from a list of the paths of the packages
        belonging to the database.

        Parameters
        ----------
        files: dockerblade.FileSystem
            access to the filesystem.
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
                package = Package.build(p, files)
            except FileNotFoundError:
                if not ignore_bad_paths:
                    raise
            packages.append(package)
        return PackageDatabase(packages)

    def __init__(self, packages: Collection[Package]) -> None:
        self.__contents = {p.name: p for p in packages}

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

    @staticmethod
    def from_dict(d: List[Dict[str, Any]]) -> 'PackageDatabase':
        return PackageDatabase([Package.from_dict(dd) for dd in d])

    def to_dict(self) -> List[Dict[str, Any]]:
        return [p.to_dict() for p in self.values()]
