# -*- coding: utf-8 -*-
__all__ = ('ROS1NodeManager',)

import xmlrpc.client
from typing import AbstractSet, Iterator
from urllib.parse import urlparse

import dockerblade
from loguru import logger

from .node import ROS1Node
from .state import SystemStateProbe
from ..common import Node, NodeManager
from ..exceptions import NodeNotFoundError, ROSWireException


class ROS1NodeManager(NodeManager):
    """Provides access to all nodes on a ROS graph."""

    def __init__(self,
                 host_ip_master: str,
                 api: xmlrpc.client.ServerProxy,
                 shell: dockerblade.Shell
                 ) -> None:
        self.__host_ip_master: str = host_ip_master
        self.__api: xmlrpc.client.ServerProxy = api
        self.__shell: dockerblade.Shell = shell
        self.__state_probe: SystemStateProbe = \
            SystemStateProbe.via_xmlrpc_connection(self.__api)

    def __get_node_names(self) -> AbstractSet[str]:
        """Fetches a list of the names of all active nodes."""
        return self.__state_probe().nodes

    def __len__(self) -> int:
        """Returns a count of the number of active nodes."""
        return len(self.__get_node_names())

    def __iter__(self) -> Iterator[str]:
        """Returns an iterator over the names of all active nodes."""
        yield from self.__get_node_names()

    def __getitem__(self, name: str) -> Node:
        """Attempts to fetch a given node.

        Parameters
        ----------
        name: str
            The name of the node.

        Returns
        -------
        Node
            An interface to the given node.

        Raises
        ------
        NodeNotFoundError
            If there is no node with the given name.
        """
        code: int
        status: str
        uri_container: str
        code, status, uri_container = \
            self.__api.lookupNode('/.roswire', name)  # type: ignore
        if code == -1:
            raise NodeNotFoundError(name)
        if code != 1:
            m = f"unexpected error when attempting to find node [{name}]: {status} (code: {code})"   # noqa: pycodestyle
            raise ROSWireException(m)

        # convert URI to host network
        port = urlparse(uri_container).port
        uri_host = f"http://{self.__host_ip_master}:{port}"
        return ROS1Node(name, uri_host, self.__shell)

    def __delitem__(self, name: str) -> None:
        """Shutdown and deregister a given node.

        Parameters
        ----------
        name: str
            The name of the node.

        Raises
        ------
        NodeNotFoundError
            no node found with given name.
        """
        try:
            node = self[name]
        except NodeNotFoundError:
            logger.exception(f"failed to delete node [{name}]: not found.")
            raise
        node.shutdown()
