# -*- coding: utf-8 -*-
"""
This module is used to build descriptions of containerised ROS applications,
and provides a manager for persisting those descriptions to disk.
"""
__all__ = ('SystemDescription', 'SystemDescriptionManager')

from typing import Any, Dict, Sequence, Union
from uuid import UUID
import base64
import contextlib
import logging
import os

from docker.models.images import Image as DockerImage
from loguru import logger
import attr
import yaml

from .definitions import TypeDatabase, FormatDatabase, PackageDatabase
from .proxy import ContainerProxyManager


@attr.s(slots=True, frozen=True, auto_attribs=True)
class SystemDescription:
    """
    An immutable description of all of the packages, types, and
    specifications (i.e., :code:`.msg`, :code:`.srv`, and :code:`.action`
    files) within a containerised ROS application.

    Attributes
    ----------
    sha256: str
        The ID of the Docker image for the application.
    sources: Sequence[str]
        The sequence of setup files that should be used to load the ROS
        workspace for the application.
    types: TypeDatabase
        A database of types for the application.
    formats: FormatDatabase
        A database of message, service and action specifications.
    packages: PackageDatabase
        A database of the packages contained within the application.
    """
    sha256: str
    sources: Sequence[str]
    types: TypeDatabase
    formats: FormatDatabase
    packages: PackageDatabase

    @staticmethod
    def from_file(fn: str) -> 'SystemDescription':
        with open(fn, 'r') as f:
            d = yaml.safe_load(f)
        return SystemDescription.from_dict(d)

    @staticmethod
    def from_dict(d: Dict[str, Any]) -> 'SystemDescription':
        sha256: str = d['sha256']
        sources: Sequence[str] = tuple(d['sources'])
        packages = PackageDatabase.from_dict(d['packages'])
        formats = FormatDatabase.build(packages)
        types = TypeDatabase.build(formats)
        return SystemDescription(sha256=sha256,
                                 sources=sources,
                                 packages=packages,
                                 formats=formats,
                                 types=types)

    def to_dict(self) -> Dict[str, Any]:
        return {'sha256': self.sha256,
                'sources': list(self.sources),
                'packages': self.packages.to_dict()}


class SystemDescriptionManager:
    """Builds and stores descriptions of containerised ROS applications.

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

    def _path_for_image_with_sources(self,
                                     image_or_tag: Union[str, DockerImage],
                                     sources: Sequence[str]
                                     ) -> str:
        sha256 = self.__containers.image_sha256(image_or_tag)
        return self._path_for_sha256_with_sources(sha256, sources)

    def _path_for_sha256_with_sources(self,
                                      sha256: str,
                                      sources: Sequence[str]
                                      ) -> str:
        unencoded_path = ':'.join([sha256] + list(sources))
        encoded_path = \
            base64.b64encode(unencoded_path.encode('utf-8')).decode('utf-8')
        return os.path.join(self.__dir_cache, encoded_path)

    def load(self,
             image_or_tag: Union[str, DockerImage],
             sources: Sequence[str]
             ) -> SystemDescription:
        """Attempts to load the description for a given Docker image from disk.

        Parameters
        ----------
        image_or_tag: Union[str, DockerImage]
            the name or object for the Docker image.
        sources: Sequence[str]
            The sequence of setup files that should be used to load the ROS
            workspace.

        Returns
        -------
        SystemDescription
            A description of the application contained within the given image.

        Raises
        ------
        FileNotFoundError
            if no description for the given image is stored on disk.
        """
        filename = self._path_for_image_with_sources(image_or_tag, sources)
        try:
            return SystemDescription.from_file(filename)
        except FileNotFoundError:
            logger.exception('failed to load description for image '
                             f'[{image_or_tag}]: file [{filename}] not found.')
            raise

    def saved(self,
              image_or_tag: Union[str, DockerImage],
              sources: Sequence[str]
              ) -> bool:
        """Determines whether an application description has been saved."""
        filename = self._path_for_image_with_sources(image_or_tag, sources)
        return os.path.exists(filename)

    def save(self, description: SystemDescription) -> None:
        """Saves a given description to disk."""
        filename = self._path_for_sha256_with_sources(description.sha256,
                                                      description.sources)
        yml = description.to_dict()
        with open(filename, 'w') as f:
            yaml.dump(yml, f, default_flow_style=False)

    def build(self,
              image_or_tag: Union[str, DockerImage],
              sources: Sequence[str],
              save: bool = True
              ) -> SystemDescription:
        """
        Builds a description of the ROS application contained within a given
        image and optionally saves that description to disk.

        Parameters
        ----------
        image_or_tag: Union[str, DockerImage]
            The name or object for the Docker image.
        sources: Sequence[str]
            The sequence of setup files that should be used to load the ROS
            workspace.
        save: bool
            If :code:`True`, the description will be saved to disk.

        Returns
        -------
        SystemDescription
            A description of the application contained within the given image.
        """
        with self.__containers.launch(image_or_tag, sources=sources) as container:  # noqa
            paths = PackageDatabase.paths(container.shell, container.files)
            db_package = PackageDatabase.from_paths(container.files, paths)
        db_format = FormatDatabase.build(db_package)
        db_type = TypeDatabase.build(db_format)

        sha256 = self.__containers.image_sha256(image_or_tag)
        description = SystemDescription(sha256=sha256,
                                        sources=sources,
                                        packages=db_package,
                                        formats=db_format,
                                        types=db_type)
        if save:
            self.save(description)
        return description

    def load_or_build(self,
                      image_or_tag: Union[str, DockerImage],
                      sources: Sequence[str],
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
        sources: Sequence[str]
            The sequence of setup files that should be used to load the ROS
            workspace.
        save: bool
            if :code:`True`, the description will be saved to disk in the
            event that it needs to be built from scratch.

        Returns
        -------
        SystemDescription
            A description of the application contained within the given image.
        """
        if self.saved(image_or_tag, sources):
            return self.load(image_or_tag, sources)
        return self.build(image_or_tag, sources, save)
