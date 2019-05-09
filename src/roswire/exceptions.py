# -*- coding: utf-8 -*-
class ROSWireException(Exception):
    """Base class used by all ROSWire exceptions."""


class CatkinException(ROSWireException):
    """Base class used by all Catkin-related exceptions."""


class CatkinBuildFailed(CatkinException):
    """The attempt to build via catkin failed."""
    def __init__(self, retcode: int, reason: str) -> None:
        msg = f"catkin build failed [retcode: {retcode}]: {reason}"
        super().__init__(msg)


class PlayerNotStarted(ROSWireException):
    """Playback has not begun."""


class PlayerAlreadyStarted(ROSWireException):
    """Playback has already started."""


class PlayerAlreadyStopped(ROSWireException):
    """Playback has already stopped."""


class PlayerFailure(ROSWireException):
    """An unexpected error occurred during playback."""
    def __init__(self, retcode: int, stdout: str) -> None:
        self.retcode = retcode
        self.stdout = stdout
        msg = "unexpected failure during bag playback [return code: {}]:\n{}"
        msg = msg.format(retcode, stdout)
        super().__init__(msg)


class PlayerTimeout(ROSWireException):
    """Playback did not complete within the specified timeout."""


class RecorderAlreadyStarted(ROSWireException):
    """Recording has already started."""


class RecorderNotStarted(ROSWireException):
    """Recording has not begun."""


class RecorderAlreadyStopped(ROSWireException):
    """Recording has already stopped."""


class ParsingError(ROSWireException):
    """ROSWire failed to parse a given file/string."""


class NodeNotFoundError(KeyError, ROSWireException):
    """No node was found with the given name."""
    def __init__(self, name: str) -> None:
        super().__init__(f"node not found: {name}")


class ServiceNotFoundError(KeyError, ROSWireException):
    """No service was found with the given name."""
    def __init__(self, name: str) -> None:
        super().__init__(f"service not found: {name}")


class ParameterNotFoundError(KeyError, ROSWireException):
    """No parameter was found with the given name."""
    def __init__(self, name: str) -> None:
        super().__init__(f"parameter not found: {name}")
