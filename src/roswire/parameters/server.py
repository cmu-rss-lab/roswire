# -*- coding: utf-8 -*-
__all__ = ('ParameterServer',)

from typing import Any, Iterator, Mapping, Sequence
import xmlrpc.client

from .. import exceptions


class ParameterServer(Mapping[str, Any]):
    """Provides access to the ROS parameter server.
    See: http://wiki.ros.org/ROS/Parameter%20Server%20API
    """
    def __init__(self, connection: xmlrpc.client.ServerProxy) -> None:
        """
        Constructs a new parameter server proxy using an XML-RPC server proxy
        for a given ROS master.
        """
        self.__caller_id = '/.roswire'
        self.__connection = connection

    def __len__(self) -> int:
        """
        Returns a count of the number of registered parameters.
        """
        return len(list(self))

    def __contains__(self, key: object) -> bool:
        """
        Determines whether the parameter server contains a given parameter
        or tree with a given key.
        """
        assert isinstance(key, str)
        conn = self.__connection
        code, msg, result = \
            conn.hasParam(self.__caller_id, key)  # type: ignore
        if code != 1:
            raise exceptions.ROSWireException("bad API call!")
        assert isinstance(result, bool)
        return result

    def __iter__(self) -> Iterator[str]:
        """
        Returns an iterator over the names of the parameters stored on the
        server.
        """
        code: int
        msg: str
        result: Sequence[str]
        code, msg, result = \
            self.__connection.getParamNames(self.__caller_id)  # type: ignore
        if code != 1:
            raise exceptions.ROSWireException("bad API call!")
        yield from result

    def __getitem__(self, key: str) -> Any:
        """
        Fetches the value of a given parameter from the server.
        If the provided key is a namespace, then the contents of that
        namespace will be returned as a dictionary.

        Parameters:
            key: the name of the parameter (or namespace).

        Returns:
            The value of the parameter or the contents of the given namespace.

        Raises:
            ParameterNotFoundError: if no parameter with the given key is found
                on the parameter server.
        """
        code, msg, result = \
            self.__connection.getParam(self.__caller_id, key)  # type: ignore
        if code == -1:
            raise exceptions.ParameterNotFoundError(key)
        if code != 1:
            raise exceptions.ROSWireException("bad API call")
        return result

    def __setitem__(self, key: str, value: Any) -> None:
        """
        Sets the value of a parameter on the server. If the value is a
        dictionary, it will be treated as a parameter tree.
        """
        code, msg, result = \
            self.__connection.setParam(self.__caller_id, key, value)  # type: ignore  # noqa
        if code != 1:
            raise exceptions.ROSWireException("bad API call!")

    def __delitem__(self, key: str) -> None:
        """
        Deletes a given parameter (or parameter tree) from the server.

        Parameters:
            key: the key for the parameter or parameter tree.

        Raises:
            KeyError: if no parameter or parameter tree is found with the
                given key on the server.
        """
        code, msg, result = \
            self.__connection.deleteParam(self.__caller_id, key)  # type: ignore  # noqa
        if code == -1:
            raise exceptions.ParameterNotFoundError(key)
        if code != 1:
            raise exceptions.ROSWireException("bad API call!")
