# -*- coding: utf-8 -*-
__all__ = (
    "CMakeInfo",
    "ExecutableInfo",
    "ExecutableKind",
    "PackageSourceExtractor",
    "SourceLanguage",)

import abc
import enum
import os
import typing as t
from pathlib import Path
from typing import Any, Iterable  # noqa: F401, E501 # Needed for tuple_from_iterable and argparse

import attr
import dockerblade
from loguru import logger

from . import Package
from .cmake import (
    argparse as cmake_argparse,
    ParserContext,
)

if t.TYPE_CHECKING:
    from .. import AppInstance


class SourceLanguage(enum.Enum):
    CXX = "cxx"
    PYTHON = "python"


class ExecutableKind(enum.Enum):
    NODE = "node"
    LIBRARY = "library"


@attr.s(auto_attribs=True, slots=True)
class ExecutableInfo:
    name: t.Optional[str]
    language: SourceLanguage
    kind: ExecutableKind
    sources: t.Set[str]
    restrict_to_paths: t.Collection[str]

    def to_dict(self) -> t.Dict[str, t.Any]:
        return {"name": self.name,
                "language": self.language.value,
                "kind": self.kind.value,
                "sources": list(self.sources),
                "path_restrictions": list(self.restrict_to_paths)}

    @classmethod
    def from_dict(cls, info: t.Dict[str, t.Any]) -> "ExecutableInfo":
        return ExecutableInfo(info["name"],
                              SourceLanguage(info["language"]),
                              ExecutableKind(info["kind"]),
                              set(info["sources"]),
                              set(info["path_restrictions"]))

    @property
    def entrypoint(self) -> t.Optional[str]:
        if self.language == SourceLanguage.CXX:
            return "main"
        else:
            return None


@attr.s(auto_attribs=True, slots=True)
class NodeletExecutableInfo(ExecutableInfo):
    entrypoint: str

    def to_dict(self) -> t.Dict[str, t.Any]:
        d = super().to_dict()
        d['entrypoint'] = self.entrypoint
        return d

    @classmethod
    def from_dict(cls, info: t.Dict[str, t.Any]) -> 'NodeletExecutableInfo':
        return NodeletExecutableInfo(info["name"],
                                     SourceLanguage(info["language"]),
                                     ExecutableKind(info["kind"]),
                                     set(info["sources"]),
                                     set(info["path_restrictions"]),
                                     info['entrypoint'])


@attr.s(auto_attribs=True, slots=True, frozen=True)
class CMakeInfo:
    executables: t.Mapping[str, ExecutableInfo]


@attr.s(auto_attribs=True)
class PackageSourceExtractor(abc.ABC):
    _files: dockerblade.FileSystem

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
        package: Package
    ) -> CMakeInfo:
        ...

    @abc.abstractmethod
    def package_paths(self, package: Package) -> t.Collection[str]:
        ...

    def process_cmake_contents(
        self,
        file_contents: str,
        package: Package,
        cmake_env: t.Dict[str, str],
    ) -> CMakeInfo:
        """
        Processes the contents of a CMakeLists.txt file for information about
        executables. Recursively includes other CMakeLists.txt files that may
        be included.

        Parameters
        ----------
        file_contents: str
            The contents of the CMakeLists.txt file
        package: Package
            The package where the CMakeLists.txt file is defined
        cmake_env: t.Dict[str, str]
            Any context variables for processing the contents
        source_extractor

        Returns
        -------
        t.Mapping[str, ExecutableInfo]
            A mapping from executable names to information about the executable
        """
        executables: t.Dict[str, ExecutableInfo] = {}
        context = ParserContext().parse(file_contents, skip_callable=False)
        for cmd, args, _arg_tokens, (_fname, _line, _column) in context:
            if cmd == "set":
                opts, args = cmake_argparse(
                    args,
                    {"PARENT_SCOPE": "-", "FORCE": "-", "CACHE": "*"}
                )
                cmake_env[args[0]] = ";".join(args[1:])
            if cmd == "unset":
                opts, args = cmake_argparse(args, {"CACHE": "-"})
                cmake_env[args[0]] = ""
            if cmd == "add_executable":
                self.__process_add_executable(
                    args,
                    cmake_env,
                    executables,
                    package)
            if cmd == "catkin_install_python":
                self.__process_python_executables(
                    args,
                    cmake_env,
                    executables)
            if cmd == 'add_library':
                self.__process_add_library(
                    args,
                    cmake_env,
                    executables,
                    package)
            if cmd == "add_subdirectory":
                executables = self.__process_add_subdirectory(
                    args,
                    cmake_env,
                    executables,
                    package)
        return CMakeInfo(executables)

    def __process_add_subdirectory(
        self,
        args: t.List[str],
        cmake_env: t.Dict[str, str],
        executables: t.Dict[str, ExecutableInfo],
        package: Package
    ) -> t.Dict[str, ExecutableInfo]:
        new_env = cmake_env.copy()
        new_env['cwd'] = os.path.join(cmake_env.get('cwd', '.'), args[0])
        join = os.path.join(package.path, new_env['cwd'])
        cmakelists_path = os.path.join(join, 'CMakeLists.txt')
        logger.debug(f"Processing {cmakelists_path}")
        included_package_info = self.process_cmake_contents(
            self._files.read(cmakelists_path),
            package,
            new_env,
        )
        executables = {
            **executables,
            **{s: included_package_info.executables[s]
               for s in included_package_info.executables}
        }
        return executables

    def __process_add_executable(
        self,
        args: t.List[str],
        cmake_env: t.Dict[str, str],
        executables: t.Dict[str, ExecutableInfo],
        package: Package
    ) -> None:
        name = args[0]
        sources: t.Set[str] = set()
        for source in args[1:]:
            if 'cwd' in cmake_env:
                sources.add(os.path.join(cmake_env['cwd'], source))
            else:
                sources.add(source)
        logger.debug(f"Adding C++ sources for {name}")
        executables[name] = ExecutableInfo(
            name=name,
            language=SourceLanguage.CXX,
            kind=ExecutableKind.NODE,
            sources=sources,
            restrict_to_paths=self.package_paths(package))

    def __process_add_library(
        self,
        args: t.List[str],
        cmake_env: t.Dict[str, str],
        executables: t.Dict[str, ExecutableInfo],
        package: Package
    ) -> None:
        name = args[0]
        if 'cwd' in cmake_env:
            sources = {os.path.join(cmake_env['cwd'], s) for s in args[1:]}
        else:
            sources = set(args[1:])
        logger.debug(f"Adding C++ library {name}")
        executables[name] = ExecutableInfo(
            name,
            SourceLanguage.CXX,
            ExecutableKind.LIBRARY,
            sources,
            self.package_paths(package))

    def __process_python_executables(
        self,
        args: t.List[str],
        cmake_env: t.Dict[str, str],
        executables: t.Dict[str, ExecutableInfo]
    ) -> None:
        opts, args = cmake_argparse(
            args,
            {"PROGRAMS": "*", "DESTINATION": "*"}
        )
        if 'PROGRAMS' in opts:
            for i in range(len(opts['PROGRAMS'])):
                # http://docs.ros.org/en/jade/api/catkin/html/howto/format2/installing_python.html  # noqa: F401, E501
                # Convention is that ros python nodes are in nodes/ directory.
                # All others are in scripts/. So just include python installs
                # that are in nodes/
                program = opts['PROGRAMS'][i]
                if program.startswith("nodes/"):
                    name = Path(program[0]).stem
                    sources = set(program)
                    if 'cwd' in cmake_env:
                        sources = set(os.path.join(cmake_env['cwd'], program))
                    logger.debug(f"Adding Python sources for {name}")
                    executables[name] = ExecutableInfo(name,
                                                       SourceLanguage.PYTHON,
                                                       ExecutableKind.NODE,
                                                       sources,
                                                       set())
        else:
            raise ValueError('PROGRAMS not specified in catin_install_python')
