class RozzyException(Exception):
    """
    Base class used by all Rozzy exceptions.
    """


class NodeNotFound(RozzyException):
    """
    No node was found with the given name.
    """
    def __init__(self, name: str) -> None:
        super().__init__(f"node not found: {name}")
