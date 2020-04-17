# -*- coding: utf-8 -*-
__all__ = ('ServiceProxy', 'ServiceManagerProxy')

from typing import Iterator, Set, Mapping, Optional
from urllib.parse import urlparse
import xmlrpc.client

import attr
import dockerblade
import yaml

from .. import exceptions
from ..definitions import Message, SrvFormat, MsgFormat
from ..description import SystemDescription


@attr.s(slots=True, auto_attribs=True)
class ServiceProxy:
    """Provides access to a ROS service.

    Attributes
    ----------
    name: str
        The fully qualified name of this service.
    url: str
        The URL of this service.
    format: SrvFormat
        The :code:`.srv` definition for this service.
    """
    name: str
    url: str
    format: SrvFormat
    _description: SystemDescription
    _shell: dockerblade.Shell

    def call(self, message: Optional[Message] = None) -> Optional[Message]:
        """Calls this service.

        Parameters
        ----------
        message: Message, optional
            The message, if any, that should be sent to the service.

        Returns
        -------
        Optional[Message]
            The reply produced by the service, if any.
        """
        if not message:
            yml = '{}'
        else:
            yml = yaml.dump(message.to_dict())
        command = f"rosservice call {self.name} '{yml}'"
        try:
            output = self._shell.check_output(command)
        except dockerblade.exceptions.CalledProcessError as error:
            if error.returncode == 2:
                raise exceptions.ROSWireException('illegal service call args') from error  # noqa
            raise exceptions.ROSWireException('unexpected error during service call') from error  # noqa

        fmt_response: Optional[MsgFormat] = self.format.response
        if not fmt_response:
            return None

        d = yaml.safe_load(output)
        db_type = self._description.types
        return db_type.from_dict(fmt_response, d)


class ServiceManagerProxy(Mapping[str, ServiceProxy]):
    """Provides access to the registered services on a ROS graph."""
    def __init__(self,
                 description: SystemDescription,
                 host_ip_master: str,
                 api: xmlrpc.client.ServerProxy,
                 shell: dockerblade.Shell
                 ) -> None:
        self.__description = description
        self.__host_ip_master = host_ip_master
        self.__api = api
        self.__shell = shell

    def __get_service_names(self) -> Set[str]:
        code, msg, state = self.__api.getSystemState('/.roswire')
        if code != 1:
            m = "an unexpected error occurred when retrieving services"
            m = f"{m}: {msg} (code: {code})"
            raise exceptions.ROSWireException(m)
        pubs, subs, services_and_providers = state
        services: Set[str] = set(s[0] for s in services_and_providers)
        return services

    def __len__(self) -> int:
        """The number of advertised services on this ROS graph."""
        return len(self.__get_service_names())

    def __iter__(self) -> Iterator[str]:
        """Returns an iterator over the names of all registered services."""
        yield from self.__get_service_names()

    def __getitem__(self, name: str) -> ServiceProxy:
        """Fetches a proxy for a service with a given name.

        Parameters
        ----------
        name: str
            The name of the service.

        Returns
        -------
        ServiceProxy
            A proxy to the given service.

        Raises
        ------
        ServiceNotFound
            If no service is found with the given name.
        """
        code, msg, url_container = self.__api.lookupService('/.roswire', name)

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
        return ServiceProxy(name,
                            url_host,
                            fmt,
                            self.__description,
                            self.__shell)
