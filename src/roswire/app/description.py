# -*- coding: utf-8 -*-
"""
This module is used to build and manage descriptions of ROS applications.
"""
__all__ = ('AppDescription',)

import base64
import os
import typing
from typing import Any, Dict, Optional

import attr
import yaml
from loguru import logger

from ..common import FormatDatabase, PackageDatabase, TypeDatabase
from ..distribution import ROSDistribution, ROSVersion
from ..ros1 import ROS1PackageDatabase
from ..ros2 import ROS2PackageDatabase

if typing.TYPE_CHECKING:
    from .app import App


@attr.s(slots=True, frozen=True, auto_attribs=True)
class AppDescription:
    """
    An immutable description of all of the packages, types, and
    specifications (i.e., :code:`.msg`, :code:`.srv`, and :code:`.action`
    files) within a containerised ROS application.

    Attributes
    ----------
    app: App
        The associated application.
    distribution: ROSDistribution
        The ROS distribution used by this application.
    types: TypeDatabase
        A database of types for the application.
    formats: FormatDatabase
        A database of message, service and action specifications.
    packages: PackageDatabase
        A database of the packages contained within the application.
    """
    app: 'App'
    distribution: ROSDistribution
    types: TypeDatabase
    formats: FormatDatabase
    packages: PackageDatabase

    @classmethod
    def _from_dict_for_app(cls,
                           d: Dict[str, Any],
                           app: 'App'
                           ) -> 'AppDescription':
        packages = PackageDatabase.from_dict(d['packages'])
        formats = FormatDatabase.build(packages)
        types = TypeDatabase.build(formats)
        distribution = ROSDistribution.with_name(d['distribution'])
        return AppDescription(app=app,
                              distribution=distribution,
                              packages=packages,
                              formats=formats,
                              types=types)

    def _to_dict(self) -> Dict[str, Any]:
        return {'sha256': self.app.sha256,
                'distribution': self.distribution.name,
                'sources': list(self.app.sources),
                'packages': self.packages.to_dict()}

    @classmethod
    def load(cls,
             app: 'App',
             filename: Optional[str] = None
             ) -> 'AppDescription':
        if not filename:
            filename = cls.path(app)

        try:
            with open(filename, 'r') as f:
                dict_ = yaml.safe_load(f)
        except FileNotFoundError:
            logger.exception('failed to load description for app '
                             f'[{app}]: file [{filename}] not found.')
            raise

        return cls._from_dict_for_app(dict_, app)

    @classmethod
    def saved(cls, app: 'App') -> bool:
        """Determines whether a given application already has a description
        saved to the filesystem."""
        filename = cls.path(app)
        return os.path.exists(filename)

    @classmethod
    def path(cls, app: 'App') -> str:
        """Determines the filesystem location where the description for a given
        application will be saved."""
        dir_roswire = app._roswire.workspace
        dir_app_descriptions = os.path.join(dir_roswire, 'descriptions')
        os.makedirs(dir_app_descriptions, exist_ok=True)

        unencoded_path = ':'.join([app.sha256] + list(app.sources))
        encoded_path = \
            base64.b64encode(unencoded_path.encode('utf-8')).decode('utf-8')
        return os.path.join(dir_app_descriptions, encoded_path)

    @classmethod
    def for_app(cls, app: 'App') -> 'AppDescription':
        """Produces a description for a given application."""
        with app.launch(require_description=False) as app_instance:
            distribution_name = app_instance.shell.environ('ROS_DISTRO')
            distribution = ROSDistribution.with_name(distribution_name)
            if distribution.ros == ROSVersion.ROS1:
                db_package = ROS1PackageDatabase.build(app_instance)
            else:
                db_package = ROS2PackageDatabase.build(app_instance)
        db_format = FormatDatabase.build(db_package)
        db_type = TypeDatabase.build(db_format)
        return AppDescription(app=app,
                              distribution=distribution,
                              packages=db_package,
                              formats=db_format,
                              types=db_type)

    def save(self, filename: Optional[str] = None) -> None:
        """Saves this description to disk."""
        if not filename:
            filename = self.path(self.app)
        dict_ = self._to_dict()
        with open(filename, 'w') as f:
            yaml.dump(dict_, f, default_flow_style=False)
