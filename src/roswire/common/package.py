# -*- coding: utf-8 -*-
__all__ = ("NodeSourceInfo", "Package", "PackageDatabase", "SourceLanguage")

import enum
import re
import typing
from abc import ABC, abstractmethod
from pathlib import Path
from typing import (
    Any,
    Collection,
    Dict,
    Generic,
    Iterator,
    List,
    Mapping,
    Optional,
    Set,
)

import attr
from loguru import logger

from .cmake import ParserContext as CMakeParserContext, argparse as cmake_argparse
from .action import ActionFormat
from .msg import MsgFormat
from .srv import SrvFormat
from ..util import tuple_from_iterable


if typing.TYPE_CHECKING:
    from .. import AppInstance


class SourceLanguage(enum.Enum):
    CXX = "cxx"
    PYTHON = "python"


@attr.s(frozen=True, auto_attribs=True, slots=True)
class NodeSourceInfo:
    node_name: Optional[str]
    language: SourceLanguage
    sources: Collection[str] = attr.ib(converter=tuple_from_iterable)

    def to_dict(self) -> Dict[str, Any]:
        return {"name" : self.node_name,
                "language": self.language.value,
                "sources" : list(self.sources),
                }

    @classmethod
    def from_dict(cls, info: Dict[str, Any]) -> "NodeSourceInfo":
        return NodeSourceInfo(info["name"], SourceLanguage(info["language"]), info["sources"])

    @classmethod
    def from_cmake(cls, file_contents: str) -> Collection['NodeSourceInfo']:
        source_infos: Set['NodeSourceInfo'] = set()
        cmake_env: Dict[str, str] = {}

        for cmd, args, arg_tokens, (fname, line, column) in CMakeParserContext().parse(file_contents):
            if cmd == "set":
                opts, args = cmake_argparse(args, {"PARENT_SCOPE": "-", "FORCE": "-", "CACHE" : "*"})
                cmake_env[args[0]] = ";".join(args[1:])
            elif cmd == "unset":
                opts, args = cmake_argparse(args, {"CACHE": "-"})
                del cmake_env[args[0]]
            elif cmd == "add_executable":
                name = args[0]
                sources: List[str] = []
                for token_type, token_val in arg_tokens[1:]:
                    if not token_val.startswith("$"):
                        sources.append(token_val)
                    else:
                        matches = re.match(r'\${(.*)}', token_val)
                        if matches and matches.group(1) in cmake_env:
                            sources.extend(cmake_env[matches.group(1)].split(";"))
                source_infos.add(NodeSourceInfo(node_name=name, language=SourceLanguage.CXX, sources=sources))
            elif cmd == "catkin_install_python":
                opts, args = cmake_argparse(args, {"PROGRAMS": "*", "DESTINATION": "*"})
                if 'PROGRAMS' in opts:
                    program_opts = opts['PROGRAMS']
                    for i in range(len(program_opts)):
                        # http://docs.ros.org/en/jade/api/catkin/html/howto/format2/installing_python.html
                        # Convention is that ros python nodes are in nodes/ directory. All others are in
                        # scripts/. So just include python installs that are in nodes/
                        program = program_opts[i]
                        if program.startswith("nodes/"):
                            name = Path(program[0]).stem
                            sources = [program]
                            source_infos.add(NodeSourceInfo(name, SourceLanguage.PYTHON, sources))
                else:
                    raise ValueError("PROGRAMS not specified in catkin_install_python")
        return source_infos


MF = typing.TypeVar("MF", bound=MsgFormat)
SF = typing.TypeVar("SF", bound=SrvFormat)
AF = typing.TypeVar("AF", bound=ActionFormat)


class Package(Generic[MF, SF, AF], ABC):
    name: str
    path: str
    messages: Collection[MF]
    services: Collection[SF]
    actions: Collection[AF]
    nodes: Mapping[str, NodeSourceInfo]

    @classmethod
    @abstractmethod
    def from_dict(cls, dict: Dict[str, Any]) -> "Package":
        ...

    def to_dict(self) -> Dict[str, Any]:
        d = {
            "name": self.name,
            "path": self.path,
            "messages": [m.to_dict() for m in self.messages],
            "services": [s.to_dict() for s in self.services],
            "actions": [a.to_dict() for a in self.actions],
            "node_sources": {n.node_name : n.to_dict() for n in self.nodes}
        }
        return d


PT = typing.TypeVar("PT", bound=Package)


@attr.s(frozen=True)
class PackageDatabase(Generic[PT], ABC, Mapping[str, PT]):
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

    _contents: Mapping[str, PT] = attr.ib()

    @classmethod
    def from_packages(cls,
                      packages: typing.Iterable[PT]
                      ) -> "PackageDatabase[PT]":
        return cls({p.name: p for p in packages})

    @classmethod
    def from_paths(cls,
                   app_instance: "AppInstance",
                   paths: List[str],
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
        packages: List[PT] = []
        for p in paths:
            try:
                package = cls._build_package(app_instance, p)
            except FileNotFoundError:
                logger.exception(f"unable to build package: {p}")
                if not ignore_bad_paths:
                    raise
            else:
                packages.append(package)
        return cls.from_packages(packages)

    @classmethod
    @abstractmethod
    def _build_package(cls, app_instance: "AppInstance", path: str) -> PT:
        ...

    @classmethod
    def build(cls,
              app_instance: "AppInstance",
              paths: Optional[List[str]] = None
              ) -> "PackageDatabase[PT]":
        if paths is None:
            paths = cls._determine_paths(app_instance)
        db_package = cls.from_paths(app_instance, paths)
        return db_package

    @classmethod
    @abstractmethod
    def _determine_paths(cls, app_instance: "AppInstance") -> List[str]:
        ...

    @classmethod
    @abstractmethod
    def from_dict(cls, d: List[Dict[str, Any]]) -> "PackageDatabase[PT]":
        ...

    def to_dict(self) -> List[Dict[str, Any]]:
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

    def __iter__(self) -> Iterator[str]:
        """
        Returns an iterator over the names of the packages contained within
        this database.
        """
        yield from self._contents
