# -*- coding: utf-8 -*-
__all__ = (
    "extract_sources_from_cmake",
    "NodeSourceInfo",
    "PackageSourceExtractor",
    "SourceLanguage",)

import abc
import enum
import re
import typing as t
from pathlib import Path
from typing import Any, Iterable  # noqa: F401, #501 # Needed for tuple_from_iterable and argparse

import attr

from .cmake import (
    argparse as cmake_argparse,
    ParserContext as CMakeParserContext,
)
from ..util import tuple_from_iterable

if t.TYPE_CHECKING:
    from .. import AppInstance


class SourceLanguage(enum.Enum):
    CXX = "cxx"
    PYTHON = "python"


@attr.s(frozen=True, auto_attribs=True, slots=True)
class NodeSourceInfo:
    node_name: str
    language: SourceLanguage
    sources: t.Collection[str] = attr.ib(converter=tuple_from_iterable)

    def to_dict(self) -> t.Dict[str, t.Any]:
        return {"name": self.node_name,
                "language": self.language.value,
                "sources": list(self.sources),
                }

    @classmethod
    def from_dict(cls, info: t.Dict[str, t.Any]) -> "NodeSourceInfo":
        return NodeSourceInfo(
            info["name"],
            SourceLanguage(info["language"]),
            info["sources"]
        )


def extract_sources_from_cmake(
        file_contents: str
) -> t.Collection['NodeSourceInfo']:
    """
    Extracts NodeSource information about nodes in a CMakefilesList.txt.

    Note, ROS2 uses both CMakeLists.txt and setup.py for package information,
    so we factor this method into common.

    Parameters
    ----------
    file_contents: str
        The contents of the CMakeLists.txt file

    Returns
    -------
    Collection[NodeSourceInfo]
        A collection of NodeSourceInfo, one for each node defined in the file
    """
    src_info: t.Set['NodeSourceInfo'] = set()
    cmake_env: t.Dict[str, str] = {}

    for cmd, args, arg_tokens, (_fname, _line, _column) \
            in CMakeParserContext().parse(file_contents):
        if cmd == "set":
            opts, args = cmake_argparse(
                args,
                {"PARENT_SCOPE": "-", "FORCE": "-", "CACHE": "*"}
            )
            cmake_env[args[0]] = ";".join(args[1:])
        elif cmd == "unset":
            opts, args = cmake_argparse(args, {"CACHE": "-"})
            del cmake_env[args[0]]
        elif cmd == "add_executable":
            name = args[0]
            sources: t.List[str] = []
            for _token_type, token_val in arg_tokens[1:]:
                if not token_val.startswith("$"):
                    sources.append(token_val)
                else:
                    matches = re.match(r'\${(.*)}', token_val)
                    if matches and matches.group(1) in cmake_env:
                        sources.extend(cmake_env[matches.group(1)].split(";"))
            src_info.add(NodeSourceInfo(name, SourceLanguage.CXX, sources))
        elif cmd == "catkin_install_python":
            opts, args = cmake_argparse(
                args,
                {"PROGRAMS": "*", "DESTINATION": "*"}
            )
            if 'PROGRAMS' in opts:
                program_opts = opts['PROGRAMS']
                for i in range(len(program_opts)):
                    # http://docs.ros.org/en/jade/api/catkin/html/howto/format2/installing_python.html  # noqa: E501
                    # Convention is that ros python nodes are in nodes/
                    # directory. All others are in scripts/. So just
                    # include python installs that are in nodes/
                    program = program_opts[i]
                    if program.startswith("nodes/"):
                        name = Path(program[0]).stem
                        sources = [program]
                        src_info.add(
                            NodeSourceInfo(
                                name,
                                SourceLanguage.PYTHON,
                                sources))
            else:
                raise ValueError(
                    "PROGRAMS not specified in catkin_install_python"
                )
    return src_info


class PackageSourceExtractor(abc.ABC):
    @classmethod
    @abc.abstractmethod
    def for_app_instance(
        cls,
        app_instance: "AppInstance"
    ) -> "PackageSourceExtractor":
        ...

    @abc.abstractmethod
    def extract_source_for_package(
        self,
        path_to_package: str
    ) -> t.Mapping[str, NodeSourceInfo]:
        ...
