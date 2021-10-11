# -*- coding: utf-8 -*-
__all__ = ("ROS2PackageSourceExtractor",)

import os.path
import typing as t

import attr
import dockerblade
from loguru import logger

from ..common import Package
from ..common.source import CMakeExtractor, CMakeInfo, CMakeLibraryTarget

if t.TYPE_CHECKING:
    from .. import AppInstance


@attr.s(auto_attribs=True)
class ROS2PackageSourceExtractor(CMakeExtractor):

    @classmethod
    def for_app_instance(
        cls,
        app_instance: "AppInstance",
    ) -> "ROS2PackageSourceExtractor":
        return ROS2PackageSourceExtractor(app_instance=app_instance)

    def get_cmake_info(
        self,
        package: Package,
    ) -> CMakeInfo:
        path_to_package = package.path
        cmakelists_path = os.path.join(path_to_package, "CMakeLists.txt")

        if self._app_instance.files.isfile(cmakelists_path):
            contents = self._app_instance.files.read(cmakelists_path)
            info = self._process_cmake_contents(contents, package, {})
            nodelets, alternative_names = self.get_nodelet_entrypoints(package)
            for nodelet, altname in alternative_names.items():
                if nodelet in info.targets:
                    info.targets[altname] = info.targets[nodelet]

            for nodelet, entrypoint in nodelets.items():
                if nodelet not in info.targets and nodelet not in alternative_names.values():
                    logger.warning(f"info.targets={info.targets}")
                    logger.warning(f"Package {package.name}: '{nodelet}' "
                                   f"is referenced in nodelet_plugins.xml but not in "
                                   f"CMakeLists.txt.")
                else:
                    target = info.targets[nodelet]
                    assert isinstance(target, CMakeLibraryTarget)
                    target.entrypoint = entrypoint

            return info
        setuppy_path = os.path.join(path_to_package, "setup.py")
        if self._app_instance.files.isfile(setuppy_path):
            logger.error(
                "Do not know how to process ROS2 packages with setup.py yet."
            )
            raise NotImplementedError("Do not know how to process ROS2 "
                                      "packages with setup.py yet.")

        logger.error(f"There is no package information inside "
                     f"{path_to_package}. Is it a package soure directory?")
        raise ValueError(f"No pacakge information for {path_to_package}.")

    def package_paths(self, package: Package) -> t.Set[str]:
        # TODO Do this properly
        include: str = os.path.normpath(
            os.path.join(package.path, f'../../include/{package.name}')
        )
        return {package.path, include}
