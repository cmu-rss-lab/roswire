__all__ = ['ServiceProxyManager']

from typing import Iterator, Any, List
import xmlrpc.client
import logging

from ..exceptions import RozzyException

logger = logging.getLogger(__name__)  # type: logging.Logger
logger.setLevel(logging.DEBUG)


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
            raise RozzyException(m)

        pubs, subs, services_and_providers = state
        services = [s[0] for s in services_and_providers]  # type: List[str]
        yield from services
