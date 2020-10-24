# -*- coding: utf-8 -*-
__all__ = ('ROS1PackageDatabase',)

import os
import typing
from typing import Any, Dict, List

import dockerblade
from loguru import logger

from ..common import PackageDatabase

if typing.TYPE_CHECKING:
    from .. import AppInstance


class ROS1PackageDatabase(PackageDatabase):

    @classmethod
    def from_dict(cls, d: List[Dict[str, Any]]) -> 'PackageDatabase':
        return cls._from_dict_internal(d)

    @classmethod
    def _determine_paths(cls, app_instance: 'AppInstance') -> List[str]:

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
