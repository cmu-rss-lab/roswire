# -*- coding: utf-8 -*-
__all__ = ("ROS1PackageSourceExtractor",)

import os.path
import typing as t

import attr
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
        return ROS1PackageSourceExtractor(app_instance=app_instance)

    def get_cmake_info(
        self,
        package: Package
    ) -> CMakeInfo:
        path_to_package = package.path
        cmakelists_path = os.path.join(path_to_package, "CMakeLists.txt")
        if not self._app_instance.files.isfile(cmakelists_path):
            logger.warning(f"No `CMakeLists.txt' in {path_to_package}")
            raise ValueError(f"No `CMakeLists.txt' in {path_to_package}")

        contents = self._app_instance.files.read(cmakelists_path)
        info = self._process_cmake_contents(contents, package, {})
        nodelets, alternative_names = self.get_nodelet_entrypoints(package)
        # Add in classname as alternative name that is referenced in loading nodelets
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

    def _find_package_workspace(self, package: Package) -> str:
        """Determines the absolute path of the workspace to which a given
        package belongs.

        Raises
        ------
        ValueError
            if the workspace for the given package could not be determined
        """
        workspace_path = os.path.dirname(package.path)
        while workspace_path != "/":

            catkin_marker_path = \
                os.path.join(workspace_path, ".catkin_workspace")
            logger.debug(f"looking for workspace marker: {catkin_marker_path}")
            if self._app_instance.files.exists(catkin_marker_path):
                return workspace_path

            catkin_tools_dir = os.path.join(workspace_path, ".catkin_tools")
            logger.debug(f"looking for workspace marker: {catkin_tools_dir}")
            if self._app_instance.files.exists(catkin_tools_dir):
                return workspace_path

            workspace_path = os.path.dirname(workspace_path)

        raise ValueError(f"unable to determine workspace for package: "
                         f"{package}")

    def package_paths(self, package: Package) -> t.Set[str]:
        paths = {package.path}
        workspace = self._find_package_workspace(package)

        for contender in ('devel/include',
                          'devel_isolated/include',
                          'install/include',
                          ):
            workspace_contender = \
                os.path.join(workspace, contender, package.name)
            if self._app_instance.files.exists(workspace_contender):
                paths.add(workspace_contender)

        return paths
