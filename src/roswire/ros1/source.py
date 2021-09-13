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
                logger.error(f"Package {package.name}: '{nodelet}' "
                             f"is referenced in "
                             f"nodelet_plugins.xml but not in "
                             f"CMakeLists.txt")
            else:
                target = info.targets[nodelet]
                assert isinstance(target, CMakeLibraryTarget)
                target.entrypoint = entrypoint
        return info

    def _find_package_devel_workspaces(self, package: Package) -> t.Set[str]:
        """Determines the absolute path of the workspace to which a given package belongs.
        Raises
        ------
        ValueError
            if the workspace for the given package could not be determined
        """

        workspace_path = os.path.dirname(package.path)
        result = None
        while workspace_path != "/" and result is None:

            catkin_marker_path = os.path.join(workspace_path, ".catkin_workspace")
            logger.debug(f"looking for workspace marker: {catkin_marker_path}")
            if self._files.exists(catkin_marker_path):
                result = workspace_path

            catkin_tools_dir = os.path.join(workspace_path, ".catkin_tools")
            logger.debug(f"looking for workspace marker: {catkin_tools_dir}")
            if self._files.exists(catkin_tools_dir):
                result = workspace_path

            workspace_path = os.path.dirname(workspace_path)

        if result is None:
            raise ValueError(f"unable to determine workspace for package: {package}")

        return {os.path.join(result, 'devel/include'),
                os.path.join(result, 'install/include'),
                os.path.join(result, 'devel_isolated/include')}

    def package_paths(self, package: Package) -> t.Set[str]:
        paths = {package.path}
        paths.update(self._find_package_devel_workspaces(package))
        return paths
