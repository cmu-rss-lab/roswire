# -*- coding: utf-8 -*-
"""
This module provides handling of XML substitution arguments, which are used
by XML launch and xacro files.

Reference
---------
https://github.com/ros/ros_comm/tree/kinetic-devel/tools/roslaunch/src/roslaunch/substitution_args.py
"""
__all__ = ('ArgumentResolver',)

from typing import Any, Dict, Match
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
    context: Dict[str, Any] = attr.ib(default=None)

    def _resolve_substitution_arg(self, s: str) -> str:
        """
        Raises
        ------
        EnvNotFoundError
            if a given environment variable is not found.
        """
        logger.debug(f"resolving substitution argument: {s}")
        s = s[2:-1]
        logger.debug(f"stripped delimiters: {s}")
        kind, *params = s.split(' ')
        logger.debug(f"argument kind: {kind}")

        # we deal with find in a later stage
        if kind == 'find':
            return f'$({s})'
        elif kind == 'env':
            var = params[0]
            return self._resolve_env(var)
        elif kind == 'optenv':
            var = params[0]
            default = ' '.join(params[1:])
            return self._resolve_optenv(var, default)
        elif kind == 'dirname':
            return self._resolve_dirname()
        elif kind == 'arg':
            arg_name = params[0]
            return self._resolve_arg(arg_name)
        elif kind == 'anon':
            return self._resolve_anon(params[0])
        return s

    def _resolve_dirname(self) -> str:
        try:
            dirname = os.path.dirname(self.context['filename'])
        except KeyError:
            m = 'filename is not provided by the launch context'
            raise SubstitutionError(m)
        return os.path.normpath(dirname)

    def _resolve_anon(self, name: str) -> str:
        raise NotImplementedError

    def _resolve_env(self, var: str) -> str:
        return self.shell.environ(var)

    def _resolve_optenv(self, var: str, default: str) -> str:
        try:
            return self.shell.environ(var)
        except dockerblade.exceptions.EnvNotFoundError:
            return default

    def _resolve_arg(self, arg_name: str) -> str:
        context = self.context
        if 'arg' not in context or arg_name not in context['arg']:
            m = f'arg not supplied to launch context [{arg_name}]'
            raise SubstitutionError(m)
        return context['arg'][arg_name]

    def _find_package_path(self, package: str) -> str:
        cmd = f'rospack find {shlex.quote(package)}'
        try:
            location = self.shell.check_output(cmd, text=True)
        except subprocess.CalledProcessError as err:
            raise SubstitutionError(f'failed to locate package: {package}') from err  # noqa
        return location.strip()

    def _find_executable(self, package: str, path: str) -> str:
        logger.debug(f'$(find-executable [package={package}] [path={path}]')
        path_original = path

        # look for executable in lib/ directory of workspaces
        catkin_find_command = ("catkin_find --first-only --libexec "
                               f"{shlex.quote(package)} {shlex.quote(path)}")
        try:
            path_in_ws = self.shell.check_output(catkin_find_command,
                                                 text=True)
            path_in_ws = path_in_ws.strip()
            return path_in_ws
        except dockerblade.CalledProcessError:
            pass

        # look for executable in source directory of package
        path_package = self._find_package_path(package)
        path_in_package = os.path.join(path_package, path)
        if not self.files.access(path_in_package, os.X_OK):
            m = ("$(find-executable pkg path) failed "
                 f"[package={package}; path={path_original}]")
            raise SubstitutionError(m)
        return path_in_package

    def _find_resource(self, package: str, path: str) -> str:
        catkin_find_command = ("catkin_find --first-only --share "
                               f"{shlex.quote(package)} {shlex.quote(path)}")
        try:
            path_in_ws = self.shell.check_output(catkin_find_command,
                                                 text=True)
            path_in_ws = path_in_ws.strip()
            return path_in_ws
        except dockerblade.CalledProcessError:
            pass

        path_package = self._find_package_path(package)
        path_in_package = os.path.join(path_package, path)
        if not self.files.isfile(path_in_package):
            m = ("$(find-resource pkg path) failed "
                 f"[package={package}; path={path}]")
            raise SubstitutionError(m)
        return path_in_package

    def _resolve_find(self, package: str, path: Optional[str] = None) -> str:
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

    def _resolve_eval(self, attribute_string: str) -> str:
        logger.debug(f'resolving eval: {attribute_string}')
        assert attribute_string.startswith('$(eval ')
        assert attribute_string[-1] == ')'
        eval_string = attribute_string[7:-1]

        if '__' in attribute_string:
            m = ("$(eval ...): refusing to evaluate potentially dangerous "
                 "expression -- must not contain double underscores")
            raise SubstitutionError(m)

        _builtins = {x: __builtins__[x]  # type: ignore
                     for x in ('dict', 'float', 'int', 'list', 'map')}
        _locals = {
            'true': True,
            'True': True,
            'false': False,
            'False': False,
            '__builtins__': _builtins,
            'arg': self._resolve_arg,
            'anon': self._resolve_anon,
            'dirname': self._resolve_dirname,
            'env': self._resolve_env,
            'find': self._resolve_find,
            'optenv': self._resolve_optenv
        }

        result = str(eval(eval_string, {}, _locals))
        logger.debug(f'resolved eval [{attribute_string}]: {result}')
        return result

    def resolve(self, s: str) -> str:
        """Resolves a given argument string."""
        if s.startswith('$(eval ') and s[-1] == ')':
            return self._resolve_eval(s)
        s = R_ARG.sub(lambda m: self._resolve_substitution_arg(m.group(0)), s)

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
