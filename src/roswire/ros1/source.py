# -*- coding: utf-8 -*-
__all__ = ("ROS1PackageSourceExtractor",)

import os.path
import typing as t

import attr
import dockerblade
from loguru import logger

from ..common import Package
from ..common.source import (ExecutableInfo, PackageSourceExtractor, process_cmake_contents)

if t.TYPE_CHECKING:
    from ..app.instance import AppInstance


@attr.s(auto_attribs=True)
class ROS1PackageSourceExtractor(PackageSourceExtractor):
    _files: dockerblade.FileSystem

    @classmethod
    def for_app_instance(
        cls,
        app_instance: "AppInstance",
    ) -> "ROS1PackageSourceExtractor":
        return ROS1PackageSourceExtractor(files=app_instance.files)

    def extract_source_for_package(
        self,
        package: Package
    ) -> t.Mapping[str, ExecutableInfo]:
        path_to_package = package.path
        cmakelists_path = os.path.join(path_to_package, "CMakeLists.txt")
        if not self._files.isfile(cmakelists_path):
            logger.warning(f"No `CMakeLists.txt' in {path_to_package}")
            return {}

        contents = self._files.read(cmakelists_path)
        return process_cmake_contents(contents, self._files, package, {}, self)
