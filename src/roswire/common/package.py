# -*- coding: utf-8 -*-
__all__ = ('Package', 'PackageDatabase')

import json
import os
import typing
from typing import (Any, Collection, Dict, Iterator, List,
                    Mapping, Optional, Tuple)

import attr
import dockerblade
from loguru import logger
from typing_extensions import Final

from .action import ActionFormat
from .msg import MsgFormat
from .srv import SrvFormat
from ..util import tuple_from_iterable

if typing.TYPE_CHECKING:
    from .. import AppInstance, ROSVersion

_COMMAND_ROS2_PKG_PREFIXES: Final[str] = (
    "python -c '"
    "import json; "
    "import ament_index_python; "
    "print(json.dumps(ament_index_python.get_packages_with_prefixes()))"
    "'")


@attr.s(frozen=True, auto_attribs=True, slots=True)
class Package:
    name: str
    path: str
    messages: Tuple[MsgFormat, ...] = attr.ib(converter=tuple_from_iterable)
    services: Tuple[SrvFormat, ...] = attr.ib(converter=tuple_from_iterable)
    actions: Tuple[ActionFormat, ...] = attr.ib(converter=tuple_from_iterable)

    @staticmethod
    def build(path: str, app_instance: 'AppInstance') -> 'Package':
        """Constructs a description of a package at a given path."""
        name: str = os.path.basename(path)
        messages: List[MsgFormat] = []
        services: List[SrvFormat] = []
        actions: List[ActionFormat] = []
        files = app_instance.files

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

        return Package(name,
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
        return Package(d['name'],
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
    @classmethod
    def build(cls,
              app_instance: 'AppInstance',
              paths: Optional[List[str]] = None
              ) -> 'PackageDatabase':
        if paths is None:
            paths = cls._determine_paths(app_instance)
        db_package = cls._from_paths(app_instance, paths)
        return db_package

    @classmethod
    def _paths_ros1(cls, app_instance: 'AppInstance') -> List[str]:
        """Parses :code:`ROS_PACKAGE_PATH` for a given shell."""
        paths: List[str] = []
        shell = app_instance.shell
        files = app_instance.files
        path_str = shell.environ('ROS_PACKAGE_PATH')
        package_paths: List[str] = path_str.strip().split(':')
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

    @classmethod
    def _paths_ros2(cls, app_instance: 'AppInstance') -> List[str]:
        """Returns a list of paths for all ROS2 packages in an application."""
        try:
            shell = app_instance.shell
            jsn = shell.check_output(_COMMAND_ROS2_PKG_PREFIXES, text=True)
        except dockerblade.exceptions.CalledProcessError:
            logger.error('failed to obtain ROS2 package prefixes')
            raise
        package_to_prefix: Mapping[str, str] = json.loads(jsn)
        paths: List[str] = [os.path.join(prefix, f'share/{package}')
                            for (package, prefix) in package_to_prefix.items()]
        return paths

    @classmethod
    def _determine_paths(cls, app_instance: 'AppInstance') -> List[str]:
        """Parses :code:`ROS_PACKAGE_PATH` for a given shell.

        Parameters
        ----------
        app_instance: AppInstance
            An instanceof of the application for which the
            list of paths should be obtained
        """
        if app_instance.description.distribution.ros == ROSVersion.ROS2:
            return cls._paths_ros2(app_instance)
        return cls._paths_ros1(app_instance)

    @classmethod
    def _from_paths(cls,
                    app_instance: 'AppInstance',
                    paths: List[str],
                    ignore_bad_paths: bool = True
                    ) -> 'PackageDatabase':
        """
        Constructs a package database from a list of the paths of the packages
        belonging to the database.

        Parameters
        ----------
        app_instance: AppInstance
            an instance fo an application from which to get
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
        return PackageDatabase(packages, paths)

    def __init__(self,
                 packages: Collection[Package],
                 paths: List[str]) -> None:
        self.__contents = {p.name: p for p in packages}
        self._paths_in_package = paths

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

    @staticmethod
    def from_dict(d: List[Dict[str, Any]]) -> 'PackageDatabase':
        return PackageDatabase([Package.from_dict(dd) for dd in d], [])

    def to_dict(self) -> List[Dict[str, Any]]:
        return [p.to_dict() for p in self.values()]
