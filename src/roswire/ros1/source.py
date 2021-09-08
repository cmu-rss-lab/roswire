# -*- coding: utf-8 -*-
__all__ = ("ROS1PackageSourceExtractor",)

import os.path
import typing as t

import attr
import dockerblade
from loguru import logger

from ..common import Package
from ..common.source import CMakeExtractor, CMakeInfo, CMakeLibraryTarget

if t.TYPE_CHECKING:
    from ..app.instance import AppInstance


@attr.s(slots=True)
class ROS1PackageSourceExtractor(CMakeExtractor):

    @classmethod
    def for_app_instance(
        cls,
        app_instance: "AppInstance",
    ) -> "ROS1PackageSourceExtractor":
        return ROS1PackageSourceExtractor(files=app_instance.files)

    @classmethod
    def for_filesystem(
        cls,
        files: dockerblade.FileSystem
    ) -> "ROS1PackageSourceExtractor":
        return ROS1PackageSourceExtractor(files)

    def get_cmake_info(
        self,
        package: Package
    ) -> CMakeInfo:
        path_to_package = package.path
        cmakelists_path = os.path.join(path_to_package, "CMakeLists.txt")
        if not self._files.isfile(cmakelists_path):
            logger.warning(f"No `CMakeLists.txt' in {path_to_package}")
            raise ValueError(f"No `CMakeLists.txt' in {path_to_package}")

        contents = self._files.read(cmakelists_path)
        info = self._process_cmake_contents(contents, package, {})
        nodelets = self.get_nodelet_entrypoints(package)
        for nodelet, entrypoint in nodelets.items():
            if nodelet not in info.targets:
                logger.error(f"'{nodelet}' is referenced in "
                             f"nodelet_plugins.xml but not in "
                             f"CMakeLists.txt")
            else:
                target = info.targets[nodelet]
                assert isinstance(target, CMakeLibraryTarget)
                target.entrypoint = entrypoint
        return info

    def package_paths(self, package: Package) -> t.Set[str]:
        # TODO Do this properly
        include: str = os.path.normpath(
            os.path.join(package.path, f'../../include/{package.name}')
        )
        return {package.path, include}
