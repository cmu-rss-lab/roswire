# -*- coding: utf-8 -*-
__all__ = ("Package", "PackageDatabase",)

import os
import typing
import typing as t
from abc import ABC, abstractmethod

import attr
from loguru import logger

from .action import ActionFormat
from .msg import MsgFormat
from .package_xml.package import PackageDefinition, parse_package_string
from .srv import SrvFormat

if t.TYPE_CHECKING:
    from .. import AppInstance


MF = typing.TypeVar("MF", bound=MsgFormat)
SF = typing.TypeVar("SF", bound=SrvFormat)
AF = typing.TypeVar("AF", bound=ActionFormat)


class Package(t.Generic[MF, SF, AF], ABC):
    name: str
    path: str
    messages: t.Collection[MF]
    services: t.Collection[SF]
    actions: t.Collection[AF]

    @classmethod
    @abstractmethod
    def from_dict(cls, dict: t.Dict[str, t.Any]) -> "Package":
        ...

    def to_dict(self) -> t.Dict[str, t.Any]:
        d = {
            "name": self.name,
            "path": self.path,
            "messages": [m.to_dict() for m in self.messages],
            "services": [s.to_dict() for s in self.services],
            "actions": [a.to_dict() for a in self.actions],
        }
        return d


PT = typing.TypeVar("PT", bound=Package)


@attr.s(frozen=True)
class PackageDatabase(t.Generic[PT], ABC, t.Mapping[str, PT]):
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

    _contents: t.Mapping[str, PT] = attr.ib()
    _definitions: t.Dict[str, PackageDefinition] = attr.ib(factory=dict)

    @classmethod
    def from_packages(cls,
                      packages: typing.Iterable[PT]
                      ) -> "PackageDatabase[PT]":
        return cls({p.name: p for p in packages})

    @classmethod
    def from_paths(cls,
                   app_instance: "AppInstance",
                   paths: t.List[str],
                   ignore_bad_paths: bool = True,
                   ) -> "PackageDatabase[PT]":
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
        packages: t.Dict[str, PT] = {}
        for p in paths:
            try:
                package = cls._build_package(app_instance, p)
            except FileNotFoundError:
                logger.exception(f"unable to build package: {p}")
                if not ignore_bad_paths:
                    raise
            else:
                if package.name not in packages:
                    packages[package.name] = package
        return cls.from_packages(packages.values())

    @classmethod
    @abstractmethod
    def _build_package(cls, app_instance: "AppInstance", path: str) -> PT:
        ...

    @classmethod
    def build(cls,
              app_instance: "AppInstance",
              paths: t.Optional[t.List[str]] = None
              ) -> "PackageDatabase[PT]":
        if paths is None:
            paths = cls._determine_paths(app_instance)
        db_package = cls.from_paths(app_instance, paths)
        return db_package

    @classmethod
    @abstractmethod
    def _determine_paths(cls, app_instance: "AppInstance") -> t.List[str]:
        ...

    @classmethod
    @abstractmethod
    def from_dict(cls, d: t.List[t.Dict[str, t.Any]]) -> "PackageDatabase[PT]":
        ...

    def to_dict(self) -> t.List[t.Dict[str, t.Any]]:
        return [p.to_dict() for p in self.values()]

    def __len__(self) -> int:
        """Returns the number of packages within this database."""
        return len(self._contents)

    def __getitem__(self, name: str) -> PT:
        """Fetches the description for a given package.

        Raises
        ------
        KeyError
            if no package exists with the given name.
        """
        return self._contents[name]

    def __iter__(self) -> t.Iterator[str]:
        """
        Returns an iterator over the names of the packages contained within
        this database.
        """
        yield from self._contents

    def get_package_definition(self, package: Package, app_instance: "AppInstance") -> PackageDefinition:
        """
        Return the contents of the package.xml file associated with the package.

        This function reads the package.xml on-demand, and then returns a cached copy thereafter.

        Parameters
        ----------
        package: Package
            The package to get the definition of
        app_instance: AppInstance
            The AppInstance that contains the defintion

        Returns
        -------
        PackageDefinition
            The definition of the pacakge as defined in package.xml
        """
        if package.name in self._definitions:
            return self._definitions[package.name]

        package_xml = os.path.join(package.path, "package.xml")
        if not app_instance.files.isfile(package_xml):
            raise ValueError(f'No package.xml for package "{package.name}" in "{package.path}')

        contents = app_instance.files.read(package_xml)
        defn = parse_package_string(contents, filename=package_xml)
        self._definitions[package.name] = defn
        return defn
