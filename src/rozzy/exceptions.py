class RozzyException(Exception):
    """
    Base class used by all Rozzy exceptions.
    """


class NodeNotFoundError(KeyError, RozzyException):
    """
    No node was found with the given name.
    """
    def __init__(self, name: str) -> None:
        super().__init__(f"node not found: {name}")


class ServiceNotFoundError(KeyError, RozzyException):
    """
    No service was found with the given name.
    """
    def __init__(self, name: str) -> None:
        super().__init__(f"service not found: {name}")


class ParameterNotFoundError(KeyError, RozzyException):
    """
    No parameter was found with the given name.
    """
    def __init__(self, name: str) -> None:
        super().__init__(f"parameter not found: {name}")
