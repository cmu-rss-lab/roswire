# -*- coding: utf-8 -*-
"""
This module provides handling of XML substitution arguments, which are used
by XML launch and xacro files.

Reference
---------
https://github.com/ros/ros_comm/tree/kinetic-devel/tools/roslaunch/src/roslaunch/substitution_args.py
"""
__all__ = ('ArgumentResolver',)

from typing import Any, Dict, Match, Sequence
import os
import re
import shlex
import subprocess

from loguru import logger
import attr
import dockerblade

from ...exceptions import SubstitutionError

R_ARG = re.compile(r'\$\(.+?\)')
R_FIND_ARG = re.compile(r'\$\(find .+?\)([^\s]+)')


@attr.s(auto_attribs=True)
class ArgumentResolver:
    shell: dockerblade.Shell
    files: dockerblade.FileSystem
    workspaces: Sequence[str]
    context: Dict[str, Any] = attr.ib(default=None)

    def _resolve_arg(self, s: str) -> str:
        """
        Raises
        ------
        EnvNotFoundError
            if a given environment variable is not found.
        """
        shell = self.shell
        context = self.context
        logger.debug(f"resolving substitution argument: {s}")
        s = s[2:-1]
        logger.debug(f"stripped delimiters: {s}")
        kind, *params = s.split(' ')
        logger.debug(f"argument kind: {kind}")

        # we deal with find in a later stage
        if kind == 'find':
            return f'$({s})'
        if kind == 'env':
            return shell.environ(params[0])
        if kind == 'optenv':
            try:
                return shell.environ(params[0])
            except dockerblade.exceptions.EnvNotFoundError:
                return ' '.join(params[1:])
        if kind == 'dirname':
            try:
                dirname = os.path.dirname(context['filename'])
            except KeyError:
                m = 'filename is not provided by the launch context'
                raise SubstitutionError(m)
            dirname = os.path.normpath(dirname)
            return dirname
        if kind == 'arg':
            arg_name = params[0]
            if 'arg' not in context or arg_name not in context['arg']:
                m = f'arg not supplied to launch context [{arg_name}]'
                raise SubstitutionError(m)
            return context['arg'][arg_name]

        # TODO $(anon name)
        return s

    def _find_package_path(self, package: str) -> str:
        cmd = f'rospack find {shlex.quote(package)}'
        try:
            location = self.shell.check_output(cmd)
        except subprocess.CalledProcessError as err:
            raise SubstitutionError(f'failed to locate package: {package}') from err  # noqa
        return location.strip()

    def _find_executable(self, package: str, path: str) -> str:
        logger.debug(f'$(find-executable [package={package}] [path={path}]')
        path_original = path

        # look for executable in lib/ directory of workspaces
        for path_workspace in self.workspaces:
            path_in_ws = os.path.join(path_workspace, 'lib', package, path)
            if self.files.access(path_in_ws, os.X_OK):
                return path_in_ws

        # look for executable in source directory of package
        path_package = self._find_package_path(package)
        path_in_package = os.path.join(path_package, path)
        if not self.files.access(path_in_package, os.X_OK):
            m = ("$(find-executable pkg path) failed "
                 f"[package={package}; path={path_original}]")
            raise SubstitutionError(m)
        return path_in_package

    def _find_resource(self, package: str, path: str) -> str:
        for path_workspace in self.workspaces:
            path_in_ws = os.path.join(path_workspace, 'share', package, path)
            if self.files.isfile(path_in_ws):
                return path_in_ws
        path_package = self._find_package_path(package)
        path_in_package = os.path.join(path_package, path)
        if not self.files.isfile(path_in_package):
            m = ("$(find-resource pkg path) failed "
                 f"[package={package}; path={path}]")
            raise SubstitutionError(m)
        return path_in_package

    def _resolve_find(self, package: str, path: str) -> str:
        logger.debug(f'resolving find: {package}')
        path_original = path

        # if there is a path following the $(find ...) argument, then we first
        # attempt to treat the argument as part of an executable or resource
        # path
        if path:
            path = path.replace('\\', '/')
            if path.startswith('/'):
                path = path[1:]

            try:
                resolved_path = self._find_executable(package, path)
                logger.debug(f'resolved executable path: {resolved_path}')
                return resolved_path
            except SubstitutionError:
                pass
            try:
                resolved_path = self._find_resource(package, path)
                logger.debug(f'resolved resource path: {resolved_path}')
                return resolved_path
            except SubstitutionError:
                pass

        resolved_path = self._find_package_path(package) + path_original
        return resolved_path

    def resolve(self, s: str) -> str:
        """Resolves a given argument string."""
        # TODO $(eval ...)
        if s.startswith('$(eval ') and s[-1] == ')':
            raise NotImplementedError
        s = R_ARG.sub(lambda m: self._resolve_arg(m.group(0)), s)

        def process_find_arg(match: Match[str]) -> str:
            # split tag and optional trailing path
            tag: str
            path: str
            tag, path = match.group(0).split(')', 1)
            tag += ')'
            args = tag[2:-1].split(' ')
            assert len(args) == 2
            tag_name, package = args
            assert tag_name == 'find'
            return self._resolve_find(package, path)

        # resolve find arguments
        s = R_FIND_ARG.sub(process_find_arg, s)
        return s
