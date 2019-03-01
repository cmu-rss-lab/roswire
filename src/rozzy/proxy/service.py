__all__ = ['ServiceProxy', 'ServiceProxyManager']

from typing import Iterator, Any, List, Set, Mapping
# from collections.abc import Mapping
from urllib.parse import urlparse
import xmlrpc.client
import logging

from .shell import ShellProxy
from .. import exceptions

logger = logging.getLogger(__name__)  # type: logging.Logger
logger.setLevel(logging.DEBUG)


class ServiceProxy:
    def __init__(self,
                 name: str,
                 url: str,
                 shell: ShellProxy
                 ) -> None:
        self.__shell: ShellProxy = shell
        self.__name: str = name
        self.__url: str = url

    @property
    def name(self) -> str:
        """
        The name of this service.
        """
        return self.__name

    @property
    def url(self) -> str:
        """
        The URL of this service.
        """
        return self.__url

    def call(self) -> None:
        # NOTE using the shell is much slower than TCPROS
        # TODO escape arguments
        # https://stackoverflow.com/questions/18935754/how-to-escape-special-characters-of-a-string-with-single-backslashes
        # http://wiki.ros.org/ROS/YAMLCommandLine
        # convert to a one-line YAML string
        yml_args = "10 1 True"  # "True"
        name =  "/mavros/set_stream_rate"  # self.__name  # FIXME
        cmd = f'rosservice call {name} {yml_args}'
        code, output, duration = self.__shell.execute(cmd)

        if code == 2:
            raise exceptions.RozzyException("illegal service call arguments.")
        if code != 0:
            raise exceptions.RozzyException("unexpected error during service call.")

        # TODO parse the output

        print(f"CODE: {code}")
        print(f"OUTPUT: {output}")


class ServiceManagerProxy(Mapping[str, ServiceProxy]):
    """
    Provides access to the registered services on a ROS graph.
    """
    def __init__(self,
                 host_ip_master: str,
                 api: xmlrpc.client.ServerProxy,
                 shell: ShellProxy
                 ) -> None:
        self.__host_ip_master: str = host_ip_master
        self.__api: xmlrpc.client.ServerProxy = api
        self.__shell: ShellProxy = shell

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
