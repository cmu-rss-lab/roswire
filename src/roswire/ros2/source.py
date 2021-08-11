# -*- coding: utf-8 -*-
__all__ = ("ROS2PackageSourceExtractor",)

import os.path
import typing as t

import attr
import dockerblade
from loguru import logger

from .. import AppInstance
from ..common.source import (
    extract_sources_from_cmake,
    NodeSourceInfo,
    PackageSourceExtractor
)


@attr.s(auto_attribs=True)
class ROS2PackageSourceExtractor(PackageSourceExtractor):
    _files: dockerblade.FileSystem

    @classmethod
    def for_app_instance(
        cls,
        app_instance: AppInstance
    ) -> "ROS2PackageSourceExtractor":
        return ROS2PackageSourceExtractor(files=app_instance.files)

    def extract_source_for_package(
        self,
        path_to_package: str
    ) -> t.Mapping[str, NodeSourceInfo]:
        cmakelists_path = os.path.join(path_to_package, "CMakeLists.txt")

        if self._files.isfile(cmakelists_path):
            contents = self._files.read(cmakelists_path)
            source_infos = extract_sources_from_cmake(contents)
            return {n.name: n for n in source_infos}

        setuppy_path = os.path.join(path_to_package, "setup.py")
        if self._files.isfile(setuppy_path):
            logger.error(
                "Do not know how to process ROS2 packages with setup.py yet."
            )
            return {}

        logger.error(f"There is no package information inside "
                     f"{path_to_package}. Is it a package soure directory?")
        return {}
