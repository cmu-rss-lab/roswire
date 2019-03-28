__all__ = ('Package', 'PackageDatabase')

from typing import (Tuple, List, Dict, Union, Any, Iterator, Collection,
                    Mapping)
import os

import attr

from .msg import MsgFormat
from .srv import SrvFormat
from .action import ActionFormat
from ..proxy import FileProxy, ShellProxy


@attr.s(frozen=True)
class Package:
    name: str = attr.ib()
    path: str = attr.ib()
    messages: Tuple[MsgFormat, ...] = attr.ib(converter=tuple)
    services: Tuple[SrvFormat, ...] = attr.ib(converter=tuple)
    actions: Tuple[ActionFormat, ...] = attr.ib(converter=tuple)

    @staticmethod
    def build(path: str, files: FileProxy) -> 'Package':
        """
        Constructs a description of a package located at a given path.
        """
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
    @staticmethod
    def paths(shell: ShellProxy) -> List[str]:
        """
        Parses the contents of the ROS_PACKAGE_PATH environment variable for a
        given shell.
        """
        code, path_str, duration = shell.execute('echo "${ROS_PACKAGE_PATH}"')
        if code != 0:
            raise Exception("unexpected error when fetching ROS_PACKAGE_PATH")
        paths: List[str] = path_str.strip().split(':')
        return paths

    @staticmethod
    def from_paths(files: FileProxy,
                   paths: List[str],
                   ignore_bad_paths: bool = True
                   ) -> 'PackageDatabase':
        """
        Constructs a package database from a list of the paths of the packages
        belonging to the database.

        Parameters:
            files: access to the filesystem.
            paths: a list of the absolute paths of the packages.
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
        """
        Returns a count of the number of packages within this database.
        """
        return len(self.__contents)

    def __getitem__(self, name: str) -> Package:
        """
        Fetches the description for a given package.

        Raises:
            KeyError: if no package exists with the given name.
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
