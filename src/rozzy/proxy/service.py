__all__ = ['ServiceProxy', 'ServiceProxyManager']

from typing import Iterator, Any, List
from urllib.parse import urlparse
import xmlrpc.client
import logging

import attr

from .. import exceptions

logger = logging.getLogger(__name__)  # type: logging.Logger
logger.setLevel(logging.DEBUG)


@attr.s
class ServiceProxy:
    name: str = attr.ib()
    url: str = attr.ib()


class ServiceManagerProxy:
    def __init__(self,
                 host_ip_master: str,
                 api: xmlrpc.client.ServerProxy
                 ) -> None:
        self.__host_ip_master = host_ip_master
        self.__api = api

    def __iter__(self) -> Iterator[str]:
        """
        Returns an iterator over the names of all registered services.
        """
        code, msg, state = self.__api.getSystemState('./rozzy')
        if code != 1:
            m = "an unexpected error occurred when retrieving services"
            m = f"{m}: {msg} (code: {code})"
            raise exceptions.RozzyException(m)

        pubs, subs, services_and_providers = state
        services = [s[0] for s in services_and_providers]  # type: List[str]
        yield from services

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
        return ServiceProxy(name, url_host)
