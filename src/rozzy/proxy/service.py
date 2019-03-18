__all__ = ('ServiceProxy', 'ServiceProxyManager')

from typing import Iterator, Any, List, Set, Mapping, Optional
from urllib.parse import urlparse
import xmlrpc.client
import logging
import shlex
import json

import attr
import yaml

from .shell import ShellProxy
from .. import exceptions
from ..definitions import Message

logger: logging.Logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)


@attr.s(slots=True)
class ServiceProxy:
    name: str = attr.ib()
    url: str = attr.ib()
    _shell: ShellProxy = attr.ib()

    def call(self, message: Optional[Message] = None) -> None:
        if not message:
            yml = '{}'
        else:
            yml = yaml.dump(message.to_dict())
        cmd = f"rosservice call {self.name} '{yml}'"
        code, output, duration = self._shell.execute(cmd)

        if code == 2:
            raise exceptions.RozzyException('illegal service call args')
        if code != 0:
            raise exceptions.RozzyException('unexpected error during service call')  # noqa

        # TODO parse output
        print(f"CODE: {code}")
        print(f"OUTPUT: {output}")
        print(f"DURATION: {duration:.3f} secs.")


class ServiceManagerProxy(Mapping[str, ServiceProxy]):
    """
    Provides access to the registered services on a ROS graph.
    """
    def __init__(self,
                 host_ip_master: str,
                 api: xmlrpc.client.ServerProxy,
                 shell: ShellProxy
                 ) -> None:
        self.__host_ip_master = host_ip_master
        self.__api = api
        self.__shell = shell

    def __get_service_names(self) -> Set[str]:
        code, msg, state = self.__api.getSystemState('./rozzy')
        if code != 1:
            m = "an unexpected error occurred when retrieving services"
            m = f"{m}: {msg} (code: {code})"
            raise exceptions.RozzyException(m)
        pubs, subs, services_and_providers = state
        services: Set[str] = set(s[0] for s in services_and_providers)
        return services

    def __len__(self) -> int:
        """
        The number of advertised services on this ROS graph.
        """
        return len(self.__get_service_names())

    def __iter__(self) -> Iterator[str]:
        """
        Returns an iterator over the names of all registered services.
        """
        yield from self.__get_service_names()

    def __getitem__(self, name: str) -> ServiceProxy:
        """
        Fetches a proxy for a service with a given name.

        Raises:
            ServiceNotFound: if no service is found with the given name.
        """
        code, msg, url_container = self.__api.lookupService('./rozzy', name)

        if code == -1:
            raise exceptions.ServiceNotFoundError(name)
        if code != 1:
            m = "an unexpected error occurred when retrieving services"
            m = f"{m}: {msg} (code: {code})"
            raise exceptions.RozzyException(m)

        # convert URL to host network
        parsed = urlparse(url_container)
        url_host = f"{parsed.scheme}://{self.__host_ip_master}:{parsed.port}"
        return ServiceProxy(name, url_host, self.__shell)
