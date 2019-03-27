# -*- coding: utf-8 -*-
"""
This module is used to build descriptions of containerised ROS applications,
and provides a manager for persisting those descriptions to disk.
"""
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
    """
    A description of the packages, types, and specifications within
    a containerised ROS application.

    Attributes
    ----------
        sha256: str
            The ID of the Docker image for the application.
        types: TypeDatabase
            A database of types for the application.
        formats: FormatDatabase
            A database of message, service and action specifications.
        packages:
            A database of the packages contained within the application.
    """
    sha256: str = attr.ib()
    types: TypeDatabase = attr.ib()
    formats: FormatDatabase = attr.ib()
    packages: PackageDatabase = attr.ib()

    @staticmethod
    def from_file(fn: str) -> 'SystemDescription':
        """Loads a description from a given file."""
        with open(fn, 'r') as f:
            d = yaml.load(f)
        return SystemDescription.from_dict(d)

    @staticmethod
    def from_dict(d: Dict[str, Any]) -> 'SystemDescription':
        """Constructs a description from a JSON dictionary."""
        sha256: str = d['sha256']
        packages = PackageDatabase.from_dict(d['packages'])
        formats = FormatDatabase.build(packages)
        types = TypeDatabase.build(formats)
        return SystemDescription(sha256=sha256,
                                 packages=packages,
                                 formats=formats,
                                 types=types)

    def to_dict(self) -> Dict[str, Any]:
        """Produces a JSON dictionary for this description."""
        return {'sha256': self.sha256,
                'packages': self.packages.to_dict()}


class SystemDescriptionManager:
    """
    Builds and stores static descriptions of containerised ROS applications.

    Attributes
    ----------
    dir: str
        The absolute path of the directory where descriptions are stored.
    """
    def __init__(self,
                 containers: ContainerProxyManager,
                 dir_cache: str
                 ) -> None:
        self.__containers = containers

        # ensure that the cache directory exists
        self.__dir_cache = dir_cache
        os.makedirs(dir_cache, exist_ok=True)

    @property
    def dir(self) -> str:
        return self.__dir_cache

    def load(self,
             image_or_tag: Union[str, DockerImage]
             ) -> SystemDescription:
        """
        Attempts to load the description for a given Docker image from disk.

        Parameters
        ----------
        image_or_tag: Union[str, DockerImage]
            the name or object for the Docker image.

        Returns
        -------
        SystemDescription
            A description of the application contained within the given image.

        Raises
        ------
        FileNotFoundError:
            if no description for the given image is stored on disk.
        """
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
        """Saves a given description to disk."""
        fn = os.path.join(self.__dir_cache, description.sha256)
        yml = description.to_dict()
        with open(fn, 'w') as f:
            yaml.dump(yml, f, default_flow_style=False)

    def build(self,
              image_or_tag: Union[str, DockerImage],
              save: bool = True
              ) -> SystemDescription:
        """
        Builds a description of the ROS application contained within a given
        image and optionally saves that description to disk.


        Returns
        -------
        SystemDescription
            A description of the application contained within the given image.
        """
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
        """
        Attempts to load a description for a given image from disk, and if
        none can be found, a description will be built from scratch and
        optionally saved.

        Parameters
        ----------
        image_or_tag: Union[str, DockerImage]
            the name or object for the Docker image.
        save: bool
            if :code:`True`, the description will be saved to disk in the
            event that it needs to be built from scratch.

        Returns
        -------
        SystemDescription
            A description of the application contained within the given image.
        """
        if self.saved(image_or_tag):
            return self.load(image_or_tag)
        return self.build(image_or_tag, save)
