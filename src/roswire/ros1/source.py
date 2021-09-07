# -*- coding: utf-8 -*-
__all__ = ("ROS1PackageSourceExtractor",)

import os.path
import typing as t

import attr
from loguru import logger

from ..common import Package
from ..common.source import (CMakeInfo, PackageSourceExtractor)

if t.TYPE_CHECKING:
    from ..app.instance import AppInstance


@attr.s(slots=True)
class ROS1PackageSourceExtractor(PackageSourceExtractor):

    @classmethod
    def for_app_instance(
        cls,
        app_instance: "AppInstance",
    ) -> "ROS1PackageSourceExtractor":
        return ROS1PackageSourceExtractor(files=app_instance.files)

    def extract_source_for_package(
        self,
        package: Package
    ) -> CMakeInfo:
        path_to_package = package.path
        cmakelists_path = os.path.join(path_to_package, "CMakeLists.txt")
        if not self._files.isfile(cmakelists_path):
            logger.warning(f"No `CMakeLists.txt' in {path_to_package}")
            return {}

        contents = self._files.read(cmakelists_path)
        return self.process_cmake_contents(contents, package, {})

    def package_paths(self, package: Package) -> t.Collection[str]:
        # TODO Do this properly
        include: str = os.path.normpath(
            os.path.join(package.path, f'../../include/{package.name}')
        )
        return {package.path, include}

