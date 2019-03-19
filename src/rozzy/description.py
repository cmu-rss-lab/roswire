__all__ = ('SystemDescription', 'SystemDescriptionManager')

from typing import Union, Dict, Any
from uuid import UUID
import contextlib
import logging
import os

import yaml
import attr
from docker.models.images import Image as DockerImage

from .definitions import TypeDatabase, FormatDatabase, PackageDatabase
from .proxy import ContainerProxyManager

logger: logging.Logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)


@attr.s(slots=True)
class SystemDescription:
    sha256: str = attr.ib()
    types: TypeDatabase = attr.ib()
    formats: FormatDatabase = attr.ib()
    packages: PackageDatabase = attr.ib()

    @staticmethod
    def from_file(fn: str) -> 'SystemDescription':
        with open(fn, 'r') as f:
            d = yaml.load(f)
        return SystemDescription.from_dict(d)

    @staticmethod
    def from_dict(d: Dict[str, Any]) -> 'SystemDescription':
        sha256: str = d['sha256']
        packages = PackageDatabase.from_dict(d['packages'])
        formats = FormatDatabase.build(packages)
        types = TypeDatabase.build(formats)
        return SystemDescription(sha256=sha256,
                                 packages=packages,
                                 formats=formats,
                                 types=types)

    def to_dict(self) -> Dict[str, Any]:
        return {'sha256': self.sha256,
                'packages': self.packages.to_dict()}


class SystemDescriptionManager:
    def __init__(self,
                 containers: ContainerProxyManager,
                 dir_cache: str
                 ) -> None:
        self.__containers = containers

        # ensure that the cache directory exists
        self.__dir_cache = dir_cache
        os.makedirs(dir_cache, exist_ok=True)

    def load(self,
             image_or_tag: Union[str, DockerImage]
             ) -> SystemDescription:
        sha256 = self.__containers.image_sha256(image_or_tag)
        fn = os.path.join(self.__dir_cache, sha256)
        try:
            return SystemDescription.from_file(fn)
        except FileNotFoundError:
            logger.exception("failed to load description for image: %s",
                             image_or_tag)
            raise

    def saved(self, image_or_tag: Union[str, DockerImage]) -> bool:
        """
        Determines whether a description for a given image has been saved.
        """
        sha256 = self.__containers.image_sha256(image_or_tag)
        fn = os.path.join(self.__dir_cache, sha256)
        return os.path.exists(fn)

    def save(self, description: SystemDescription) -> None:
        fn = os.path.join(self.__dir_cache, description.sha256)
        yml = description.to_dict()
        with open(fn, 'w') as f:
            yaml.dump(yml, f, default_flow_style=False)

    def build(self,
              image_or_tag: Union[str, DockerImage],
              save: bool = True
              ) -> SystemDescription:
        sha256 = self.__containers.image_sha256(image_or_tag)
        with self.__containers.launch(image_or_tag) as container:
            paths = PackageDatabase.paths(container.shell)
            db_package = PackageDatabase.from_paths(container.files, paths)
        db_format = FormatDatabase.build(db_package)
        db_type = TypeDatabase.build(db_format)

        description = SystemDescription(sha256=sha256,
                                        packages=db_package,
                                        formats=db_format,
                                        types=db_type)
        if save:
            self.save(description)
        return description

    def load_or_build(self,
                      image_or_tag: Union[str, DockerImage],
                      save: bool = True
                      ) -> SystemDescription:
        if self.saved(image_or_tag):
            return self.load(image_or_tag)
        return self.build(image_or_tag, save)
