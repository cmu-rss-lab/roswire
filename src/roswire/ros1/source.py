# -*- coding: utf-8 -*-
__all__ = ("ROS1PackageSourceExtractor",)

import os.path
import typing as t

import attr
import dockerblade
from loguru import logger

from ..common.source import (
    extract_sources_from_cmake,
    NodeSourceInfo,
    PackageSourceExtractor,
)

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
        path_to_package: str
    ) -> t.Mapping[str, NodeSourceInfo]:
        cmakelists_path = os.path.join(path_to_package, "CMakeLists.txt")
        if not self._files.isfile(cmakelists_path):
            logger.warning(f"No `CMakeLists.txt' in {path_to_package}")
            return {}

        contents = self._files.read(cmakelists_path)
        source_infos = extract_sources_from_cmake(contents)
        return {n.node_name: n for n in source_infos}
