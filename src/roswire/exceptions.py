# -*- coding: utf-8 -*-
import attr as _attr


class ROSWireException(Exception):
    """Base class used by all ROSWire exceptions."""


@_attr.s(frozen=True, auto_exc=True, auto_attribs=True)
class UnexpectedServiceCallError(ROSWireException):
    """An unexpected error occurred during a service call.

    Attributes
    ----------
    service_name: str
        The name of the service that was called.
    retcode: int
        The return code that was produced by rosservice.
    output: str
        The output that was produced by rosservice
    """
    service_name: str
    retcode: int
    output: str

    def __str__(self) -> str:
        m = (f"Unexpected exit code [{self.retcode}] occurred "
             f" during call to service [{self.service_name}]. "
             f"Produced output: \"{self.output}\"")
        return m


class FailedToParseLaunchFile(ROSWireException):
    """An attempt to parse a launch file failed."""


class EnvNotFoundError(ROSWireException):
    """A given environment variable could not be found."""
    def __init__(self, var: str) -> None:
        m = f"could not find enviroment variable: {var}"
        super().__init__(m)


class SubstitutionError(ROSWireException):
    """An error occurred during substitution argument handling."""


class CatkinException(ROSWireException):
    """Base class used by all Catkin-related exceptions."""


class CatkinBuildFailed(CatkinException):
    """The attempt to build via catkin failed."""
    def __init__(self, retcode: int, reason: str) -> None:
        msg = f"catkin build failed [retcode: {retcode}]: {reason}"
        super().__init__(msg)


class CatkinCleanFailed(CatkinException):
    """An attempt to clean the catkin workspace failed."""
    def __init__(self, retcode: int, reason: str) -> None:
        msg = f"catkin clean failed [retcode: {retcode}]: {reason}"
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


@_attr.s(frozen=True, auto_exc=True, auto_attribs=True)
class PatchFailedError(ROSWireException):
    """An error occurred during the application of a patch."""
    retcode: int
    output: str
