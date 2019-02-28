class RozzyException(Exception):
    """
    Base class used by all Rozzy exceptions.
    """


class NodeNotFoundError(RozzyException):
    """
    No node was found with the given name.
    """
    def __init__(self, name: str) -> None:
        super().__init__(f"node not found: {name}")


class ServiceNotFoundError(RozzyException):
    """
    No service was found with the given name.
    """
    def __init__(self, name: str) -> None:
        super().__init__(f"service not found: {name}")
