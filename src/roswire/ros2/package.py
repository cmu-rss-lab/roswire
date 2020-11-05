# -*- coding: utf-8 -*-
__all__ = ('ROS2PackageDatabase',)

import json
import os
import typing
from typing import Any, Dict, Iterable, List, Mapping

import dockerblade
from loguru import logger
from typing_extensions import Final

from ..common import Package, PackageDatabase

if typing.TYPE_CHECKING:
    from .. import AppInstance

_COMMAND_ROS2_PKG_PREFIXES: Final[str] = (
    "python -c '"
    "import json; "
    "import ament_index_python; "
    "print(json.dumps(ament_index_python.get_packages_with_prefixes()))"
    "'")


class ROS2PackageDatabase(PackageDatabase):
    @classmethod
    def _from_pacakages_and_paths(cls, packages: Iterable[Package],
                                  paths: Iterable[str]) -> 'PackageDatabase':
        return ROS2PackageDatabase(packages, paths)

    @classmethod
    def from_dict(cls, d: List[Dict[str, Any]]) -> 'PackageDatabase':
        return ROS2PackageDatabase([Package.from_dict(dd) for dd in d], [])

    @classmethod
    def _determine_paths(cls, app_instance: 'AppInstance') -> List[str]:
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
