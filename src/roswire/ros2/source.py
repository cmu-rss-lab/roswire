# -*- coding: utf-8 -*-
__all__ = ("ROS2PackageSourceExtractor",)

import os.path
import typing as t

import attr
import dockerblade
from loguru import logger

from ..common import Package
from ..common.source import (
    CMakeInfo,
    CMakeExtractor,
)

if t.TYPE_CHECKING:
    from .. import AppInstance


@attr.s(auto_attribs=True)
class ROS2PackageSourceExtractor(CMakeExtractor):
    _files: dockerblade.FileSystem

    @classmethod
    def for_app_instance(
        cls,
        app_instance: "AppInstance",
    ) -> "ROS2PackageSourceExtractor":
        return ROS2PackageSourceExtractor(files=app_instance.files)

    def get_cmake_info(
        self,
        package: Package,
    ) -> CMakeInfo:
        path_to_package = package.path
        cmakelists_path = os.path.join(path_to_package, "CMakeLists.txt")

        if self._files.isfile(cmakelists_path):
            contents = self._files.read(cmakelists_path)
            return self._process_cmake_contents(contents, package, {})

        setuppy_path = os.path.join(path_to_package, "setup.py")
        if self._files.isfile(setuppy_path):
            logger.error(
                "Do not know how to process ROS2 packages with setup.py yet."
            )
            raise NotImplementedError("Do not know how to process ROS2 "
                                      "packages with setup.py yet.")

        logger.error(f"There is no package information inside "
                     f"{path_to_package}. Is it a package soure directory?")
        raise ValueError(f"No pacakge information for {path_to_package}.")

    def package_paths(self, package: Package) -> t.Collection[str]:
        # TODO Do this properly
        include: str = os.path.normpath(
            os.path.join(package.path, f'../../include/{package.name}')
        )
        return {package.path, include}
