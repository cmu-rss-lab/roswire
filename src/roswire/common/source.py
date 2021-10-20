# -*- coding: utf-8 -*-
__all__ = (
    "CMakeInfo",
    "CMakeTarget",
    "CMakeExtractor",
    "PackageCMakeTargets",
    "SourceLanguage",
)

import abc
import enum
import os
import re
import typing as t
from pathlib import Path
from typing import Any, Iterable  # noqa: F401, E501 # Needed for tuple_from_iterable and argparse

import attr
from loguru import logger

from . import Package
from .cmake import (
    argparse as cmake_argparse,
    ParserContext,
)
from .nodelet_xml import NodeletLibrary, NodeletsInfo
from .package_xml.package import InvalidPackage
from ..util import key_val_list_to_dict

if t.TYPE_CHECKING:
    from .. import AppInstance


class SourceLanguage(enum.Enum):
    CXX = "cxx"
    PYTHON = "python"


DUMMY_VALUE = "__dummy_property_value__"  # A dummy value used as a stand in for properties we don't need


@attr.s(auto_attribs=True)
class CMakeTarget:
    name: str
    language: SourceLanguage
    sources: t.Set[str]
    restrict_to_paths: t.Set[str]

    def to_dict(self) -> t.Dict[str, t.Any]:
        return {"name": self.name,
                "language": self.language.value,
                "sources": list(self.sources),
                "path_restrictions": list(self.restrict_to_paths)}

    @classmethod
    def from_dict(cls, info: t.Dict[str, t.Any]) -> "CMakeTarget":
        return CMakeTarget(info["name"],
                           SourceLanguage(info["language"]),
                           set(info["sources"]),
                           set(info["path_restrictions"]))


@attr.s(auto_attribs=True)
class PackageCMakeTargets:
    """Describes the CMake build targets for a given package

    Attributes
    ----------
    package: Package
        the package to which these sources belong
    targets: t.Collection[CMakeTarget]
        the build targets that are contained within this package
    """

    package: Package
    targets: t.Collection[CMakeTarget]


@attr.s(auto_attribs=True)
class CMakeBinaryTarget(CMakeTarget):
    @property
    def entrypoint(self) -> t.Optional[str]:
        if self.language == SourceLanguage.CXX:
            return "main"
        else:
            return None


@attr.s(auto_attribs=True)
class CMakeLibraryTarget(CMakeBinaryTarget):
    _entrypoint: t.Optional[str] = attr.ib(default=None)

    @property
    def entrypoint(self) -> t.Optional[str]:
        return self._entrypoint

    @entrypoint.setter
    def entrypoint(self, entrypoint: str) -> None:
        self._entrypoint = entrypoint

    def to_dict(self) -> t.Dict[str, t.Any]:
        d = super().to_dict()
        if self.entrypoint:
            d['entrypoint'] = self.entrypoint
        return d

    @classmethod
    def from_dict(cls, info: t.Dict[str, t.Any]) -> 'CMakeLibraryTarget':
        target = CMakeLibraryTarget(info["name"],
                                    SourceLanguage(info["language"]),
                                    set(info["sources"]),
                                    set(info["path_restrictions"]),
                                    )
        if 'entrypoint' in info:
            target.entrypoint = info['entrypoint']
        return target


@attr.s(auto_attribs=True, slots=True, frozen=True)
class CMakeInfo:
    targets: t.Dict[str, CMakeTarget]


@attr.s(auto_attribs=True)
class CMakeExtractor(abc.ABC):
    _app_instance: "AppInstance"

    @classmethod
    @abc.abstractmethod
    def for_app_instance(
        cls,
        app_instance: "AppInstance"
    ) -> "CMakeExtractor":
        ...

    @abc.abstractmethod
    def get_cmake_info(
        self,
        package: Package
    ) -> CMakeInfo:
        ...

    @abc.abstractmethod
    def package_paths(self, package: Package) -> t.Set[str]:
        ...

    @abc.abstractmethod
    def _get_global_cmake_variables(self, package: Package) -> t.Dict[str, str]:
        ...

    def get_nodelet_entrypoints(self, package: Package) -> t.Mapping[str, NodeletLibrary]:
        """
        Returns the potential nodelet entrypoints and classname for the package.

        Parameters
        ----------
        package: Package
            The package to get nodelet info from

        Returns
        -------
        Mapping[str, NodeletLibrary]
            A mapping of nodelet names to NodeletInfo
        """
        workspace = package.path
        nodelets_xml_path = os.path.join(workspace, 'nodelet_plugins.xml')
        if not self._app_instance.files.isfile(nodelets_xml_path):
            # Read from the package database
            logger.info("Is nodelet plugin defined in package.xml?")
            try:
                defn = self._app_instance.description.packages.get_package_definition(package, self._app_instance)
                for export in defn.exports:
                    logger.debug("Looking in export of package.xml")
                    if export.tagname == 'nodelet' and 'plugin' in export.attributes:
                        logger.debug("Found nodelet tag and plugin attribute")
                        plugin = export.attributes['plugin']
                        plugin = plugin.replace('${prefix}/', '')
                        nodelets_xml_path = os.path.join(package.path, plugin)
            except InvalidPackage:
                logger.warning("Failed to parse package.xml")

        logger.debug(f"Looking for nodelet plugin file: {nodelets_xml_path}")
        if self._app_instance.files.exists(nodelets_xml_path):
            logger.debug(f"Reading plugin information from {nodelets_xml_path}")
            contents = self._app_instance.files.read(nodelets_xml_path)
            logger.debug(f"Contents of that file: {contents}")
            nodelet_info = NodeletsInfo.from_nodelet_xml(contents)
            return {info.name.split('/')[1] : info for info in nodelet_info.libraries}
        logger.warning(f"The specified '{nodelets_xml_path}' does not exist.")
        return {}

    def _info_from_cmakelists(self, cmakelists_path: str, package: Package) -> CMakeInfo:
        contents = self._app_instance.files.read(cmakelists_path)
        env = self._get_global_cmake_variables(package)
        info = self._process_cmake_contents(contents, package, env)
        nodelet_libraries = self.get_nodelet_entrypoints(package)
        # Add in classname as a name that can be referenced in loading nodelets
        for nodelet, library in nodelet_libraries.items():
            if nodelet in info.targets:
                info.targets[library.name] = info.targets[nodelet]
        for nodelet, library in nodelet_libraries.items():
            if nodelet not in info.targets:
                logger.warning(f"info.targets={info.targets}")
                logger.warning(f"Package {package.name}: '{nodelet}' "
                               f"is referenced in nodelet_plugins.xml but not in "
                               f"CMakeLists.txt.")
            else:
                target = info.targets[nodelet]
                assert isinstance(target, CMakeLibraryTarget)
                target.entrypoint = library.entrypoint
        return info

    def _process_cmake_contents(
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

        Returns
        -------
        CMakeInfo:
            Information about the targets in CMakeLists.txt
        """
        executables: t.Dict[str, CMakeTarget] = {}
        context = ParserContext().parse(file_contents, skip_callable=False, var=cmake_env)
        for cmd, args, _arg_tokens, (_fname, _line, _column) in context:
            cmd = cmd.lower()
            if cmd == "project":
                opts, args = cmake_argparse(args, {})
                cmake_env["PROJECT_NAME"] = args[0]
            if cmd == "set_target_properties":
                opts, args = cmake_argparse(args, {"PROPERTIES": "*"})
                properties = key_val_list_to_dict(opts.get("PROPERTIES", []))
                if 'OUTPUT_NAME' in properties:
                    var_pattern = re.compile(r"([^$]*)\${([^}]*)}(.*)")
                    var_match = var_pattern.match(args[0])
                    if var_match:
                        args[0] = var_match.group(1) + cmake_env[var_match.group(2)] + var_match.group(3)
                    if args[0] in executables:
                        executables[properties['OUTPUT_NAME']] = executables[args[0]]
                        del executables[args[0]]
                    else:
                        logger.error(f"{args[0]} is not in the list of targets")
            if cmd == "set":
                opts, args = cmake_argparse(
                    args,
                    {"PARENT_SCOPE": "-", "FORCE": "-", "CACHE": "*"}
                )
                cmake_env[args[0]] = ";".join(args[1:])
            if cmd == "unset":
                opts, args = cmake_argparse(args, {"CACHE": "-"})
                cmake_env[args[0]] = ""
            if cmd == "file":
                logger.warning(f'Processing file directive: {args}')
                opts, args = cmake_argparse(args, {'FOLLOW_SYMLINKS': '-',
                                                   'LIST_DIRECTORIES': '?',
                                                   'RELATIVE': '?',
                                                   'GLOB_RECURSE': '-',
                                                   'GLOB': '-',
                                                   })
                self.__process_file_directive(args, cmake_env, opts, package)
            if cmd == "add_executable" or cmd == 'cuda_add_executable':
                opts, args = cmake_argparse(
                    args,
                    {"EXCLUDE_FROM_ALL": "-"}
                )
                if not opts['EXCLUDE_FROM_ALL']:
                    self.__process_add_executable(
                        args,
                        cmake_env,
                        executables,
                        package)
            if cmd == "catkin_install_python":
                self.__process_python_executables(
                    args,
                    cmake_env,
                    executables,
                    package,
                )
            if cmd == 'add_library' or cmd == 'cuda_add_library':
                opts, args = cmake_argparse(
                    args,
                    {"SHARED": "-"}
                )
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

    def __process_file_directive(
        self,
        args: t.List[str],
        cmake_env: t.Dict[str, t.Any],
        opts: t.Dict[str, t.Any],
        package: Package,
    ) -> None:
        if not opts['GLOB_RECURSE'] and not opts['GLOB']:
            logger.warning(f"Cannot process file({args[0]} ...")
        else:
            path = package.path
            if opts['RELATIVE']:
                path = os.path.join(package.path, opts['RELATIVE'])
            # remove . and .. from the path by resolving them
            path = os.path.normpath(path)
            logger.debug(f"Finding files matching {args[1:]} in {path}")
            matches = []
            for arg in args[1:]:
                finds = self._app_instance.files.find(path, arg)
                logger.debug(f"Found the following matches to {arg} in {path}: {finds}")
                matches.extend(finds)
            if opts['RELATIVE']:
                # convert path to be relative
                matches = [os.path.relpath(m, path) for m in matches]

            cmake_env[args[0]] = ';'.join(matches)
            logger.debug(f"Set {args[0]} to {cmake_env[args[0]]}")

    def __process_add_subdirectory(
        self,
        args: t.List[str],
        cmake_env: t.Dict[str, str],
        executables: t.Dict[str, CMakeTarget],
        package: Package,
    ) -> t.Dict[str, CMakeTarget]:
        new_env = cmake_env.copy()
        new_env['CMAKE_SOURCE_DIR'] = os.path.join(cmake_env.get('CMAKE_SOURCE_DIR', '.'), args[0])
        new_env['CMAKE_CURRENT_SOURCE_DIR'] = new_env['CMAKE_SOURCE_DIR']
        new_env['CMAKE_CURRENT_BINARY_DIR'] = new_env['CMAKE_SOURCE_DIR']
        new_env['CMAKE_BINARY_DIR'] = new_env['CMAKE_SOURCE_DIR']
        new_env['cwd'] = os.path.join(cmake_env.get('cwd', '.'), args[0])
        join = os.path.join(package.path, new_env['cwd'])
        cmakelists_path = os.path.join(join, 'CMakeLists.txt')
        logger.debug(f"Processing {cmakelists_path}")
        included_package_info = self._process_cmake_contents(
            self._app_instance.files.read(cmakelists_path),
            package,
            new_env,
        )
        executables = {
            **executables,
            **{s: included_package_info.targets[s]
               for s in included_package_info.targets}
        }
        return executables

    def __process_add_executable(
        self,
        args: t.List[str],
        cmake_env: t.Dict[str, str],
        executables: t.Dict[str, CMakeTarget],
        package: Package,
    ) -> None:
        name = args[0]
        sources: t.Set[str] = set()
        for source in args[1:]:
            real_src = self._resolve_to_real_file(source, package, cmake_env)
            sources.add(real_src)
        logger.debug(f"Adding C++ sources for {name}")
        executables[name] = CMakeBinaryTarget(
            name=name,
            language=SourceLanguage.CXX,
            sources=sources,
            restrict_to_paths=self.package_paths(package))

    def _resolve_to_real_file(
        self,
        filename: str,
        package: Package,
        cmake_env: t.Dict[str, str],
    ) -> str:
        real_filename = filename
        if 'cwd' in cmake_env:
            real_filename = os.path.join(cmake_env['cwd'], filename)
        if not self._app_instance.files.isfile(os.path.join(package.path, real_filename)):
            path = Path(real_filename)
            parent = str(path.parent)
            try:
                all_files = self._app_instance.files.listdir(os.path.join(package.path, parent))
                matching_files = [f for f in all_files if f.startswith(path.name)]
                if len(matching_files) != 1:
                    raise ValueError(f"Only one file should match '{real_filename}'. "
                                     f"Currently {len(matching_files)} files do: {matching_files}")
                real_filename = os.path.join(parent, matching_files[0])
            except Exception:
                logger.error('Error finding real file')
                logger.error(cmake_env)
                raise
        return real_filename

    def __process_add_library(
        self,
        args: t.List[str],
        cmake_env: t.Dict[str, str],
        executables: t.Dict[str, CMakeTarget],
        package: Package,
    ) -> None:
        name = args[0]
        sources: t.Set[str] = set()
        throw = False
        for source in args[1:]:
            real_src = self._resolve_to_real_file(source, package, cmake_env)
            sources.add(real_src)
        if throw:
            logger.error(sources)
            raise NotImplementedError()
        logger.debug(f"Adding C++ library {name}")
        executables[name] = CMakeLibraryTarget(
            name,
            SourceLanguage.CXX,
            sources,
            self.package_paths(package))

    def __process_python_executables(
        self,
        args: t.List[str],
        cmake_env: t.Dict[str, str],
        executables: t.Dict[str, CMakeTarget],
        package: Package,
    ) -> None:
        opts, args = cmake_argparse(
            args,
            {"PROGRAMS": "*", "DESTINATION": "*"}
        )
        if 'PROGRAMS' not in opts:
            raise ValueError('PROGRAMS not specified in catin_install_python')

        for program in opts['PROGRAMS']:
            # http://docs.ros.org/en/jade/api/catkin/html/howto/format2/installing_python.html  # noqa: F401, E501
            # Convention is that ros python nodes are in nodes/ directory.
            # All others are in scripts/. So just include python installs
            # that are in nodes/
            if program.startswith("nodes/"):
                name = Path(program[0]).stem
                sources: t.Set[str] = set()
                source = self._resolve_to_real_file(program, package, cmake_env)
                sources.add(source)
                logger.debug(f"Adding Python sources for {name}")
                executables[name] = CMakeTarget(name,
                                                SourceLanguage.PYTHON,
                                                sources,
                                                set())
