# -*- coding: utf-8 -*-
__all__ = ('ServiceManager',)

import typing
import xmlrpc.client
from typing import AbstractSet, Iterator, Mapping
from urllib.parse import urlparse

import dockerblade

from .service import Service
from ..state import SystemStateProbe
from ... import exceptions

if typing.TYPE_CHECKING:
    from ...app import AppDescription


class ServiceManager(Mapping[str, Service]):
    """Provides access to the registered services on a ROS graph."""
    def __init__(self,
                 description: 'AppDescription',
                 host_ip_master: str,
                 api: xmlrpc.client.Server,
                 shell: dockerblade.Shell
                 ) -> None:
        self.__description = description
        self.__host_ip_master = host_ip_master
        self.__api = api
        self.__shell = shell
        self.__state_probe: SystemStateProbe = \
            SystemStateProbe.via_xmlrpc_connection(self.__api)

    def __get_service_names(self) -> AbstractSet[str]:
        return set(self.__state_probe().services.keys())

    def __len__(self) -> int:
        """The number of advertised services on this ROS graph."""
        return len(self.__get_service_names())

    def __iter__(self) -> Iterator[str]:
        """Returns an iterator over the names of all registered services."""
        yield from self.__get_service_names()

    def __getitem__(self, name: str) -> Service:
        """Fetches a proxy for a service with a given name.

        Parameters
        ----------
        name: str
            The name of the service.

        Returns
        -------
        Service
            A proxy to the given service.

        Raises
        ------
        ServiceNotFound
            If no service is found with the given name.
        """
        code: int
        msg: str
        url_container: str
        code, msg, url_container = \
            self.__api.lookupService('/.roswire', name)  # type: ignore

        if code == -1:
            raise exceptions.ServiceNotFoundError(name)
        if code != 1:
            m = "an unexpected error occurred when retrieving services"
            m = f"{m}: {msg} (code: {code})"
            raise exceptions.ROSWireException(m)

        # convert URL to host network
        parsed = urlparse(url_container)
        url_host = f"{parsed.scheme}://{self.__host_ip_master}:{parsed.port}"

        # find the format for the service
        command = f'rosservice type {name}'
        try:
            name_fmt = self.__shell.check_output(command, text=True)
        except dockerblade.exceptions.CalledProcessError as error:
            m = f"unable to determine type for service [{name}]"
            raise exceptions.ROSWireException(m) from error
        fmt = self.__description.formats.services[name_fmt]
        return Service(name=name,
                       url=url_host,
                       format=fmt,
                       description=self.__description,
                       shell=self.__shell)
