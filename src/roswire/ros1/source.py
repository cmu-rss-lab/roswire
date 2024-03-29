# -*- coding: utf-8 -*-
__all__ = ("ROS1PackageSourceExtractor",)

import os.path
import typing as t

import attr
from loguru import logger

from ..common import Package
from ..common.source import CMakeExtractor, CMakeInfo, DUMMY_VALUE

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

        return self._info_from_cmakelists(cmakelists_path, package)

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

    def _get_global_cmake_variables(self, package: Package) -> t.Dict[str, str]:
        dict_ = {
            'CMAKE_SOURCE_DIR': '',
            'PROJECT_SOURCE_DIR': '',
            'CMAKE_CURRENT_SOURCE_DIR': '',
            'PROJECT_VERSION': DUMMY_VALUE,
            'CATKIN_GLOBAL_INCLUDE_DESTINATION': "/include",
            'PYTHON_EXT_SUFFIX': '""',
        }
        workspace = self._find_package_workspace(package)
        paths = set()
        for contender in ('devel',
                          'devel_isolated',
                          'install',
                          ):
            workspace_contender = \
                os.path.join(workspace, contender)
            if self._app_instance.files.exists(workspace_contender):
                paths.add(workspace_contender)
        assert len(paths) == 1
        dict_['CATKIN_DEVEL_PREFIX'] = os.path.join(paths.pop(), package.name)
        dict_['CMAKE_BINARY_DIR'] = os.path.join(workspace, 'build')
        dict_['CMAKE_CURRENT_BINARY_DIR'] = os.path.join(dict_['CMAKE_BINARY_DIR'], package.name)
        dict_['PROJECT_NAME'] = package.name  # Put package name as default project, overwritten by project(...)
        return dict_
