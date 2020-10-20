# -*- coding: utf-8 -*-
__all__ = ('ROS2ServiceManager',)

import typing
from typing import Iterator, Mapping

import attr

from .service import ROS2Service

if typing.TYPE_CHECKING:
    from .. import AppInstance


@attr.s(frozen=True, auto_attribs=True, slots=True)
class ROS2ServiceManager(Mapping[str, ROS2Service]):
    """Provides an interface for interacting with ROS2 services."""
    app_instance: 'AppInstance'

    @classmethod
    def for_app_instance(cls,
                         app_instance: 'AppInstance'
                         ) -> 'ROS2ServiceManager':
        return ROS2ServiceManager(app_instance=app_instance)

    def __getitem__(self, name: str) -> ROS2Service:
        """Attempts to access a given service.

        Parameters
        ----------
        name: str
            The name of the service.

        Returns
        -------
        ROS2Service
            An interface to the given service.

        Raises
        ------
        ServiceNotFound
            If no service is found with the given name.
        """
        raise NotImplementedError

    def __len__(self) -> int:
        """Returns a count of the number of advertised ROS services."""
        raise NotImplementedError

    def __iter__(self) -> Iterator[str]:
        """Returns an iterator over the names of all advertised ROS services."""  # noqa
        raise NotImplementedError
