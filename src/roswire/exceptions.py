# -*- coding: utf-8 -*-
import typing as _typing

import attr as _attr

if _typing.TYPE_CHECKING:
    from . import App


class ROSWireException(Exception):
    """Base class used by all ROSWire exceptions."""


@_attr.s(frozen=True, auto_exc=True, auto_attribs=True, str=False)
class PackageNotFound(ValueError, ROSWireException):
    """No package was found with a given name."""
    package: str

    def __str__(self) -> str:
        return f"Could not find package with name: {self.package}"


@_attr.s(frozen=True, auto_exc=True, auto_attribs=True, str=False)
class ImageNotFound(ValueError, ROSWireException):
    """No Docker image was found with a given name."""
    image: str

    def __str__(self) -> str:
        return f"Could not find Docker image with name: {self.image}"


@_attr.s(frozen=True, auto_exc=True, auto_attribs=True, str=False)
class LaunchFileNotFound(ValueError, ROSWireException):
    """No launch file was found at the given path."""
    path: str

    def __str__(self) -> str:
        return f"Could not find launch file at path: {self.path}"


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


@_attr.s(frozen=True, auto_exc=True, auto_attribs=True, str=False)
class NoDescriptionError(RuntimeError, ROSWireException):
    """No description has been generated for an application."""
    app: 'App'

    def __str__(self) -> str:
        return f"no description for application: {self.app}"


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


@_attr.s(frozen=True, auto_exc=True, auto_attribs=True, str=False)
class SourceNotFoundError(ValueError, ROSWireException):
    """A given source could not be found inside the container."""
    source: str

    def __str__(self) -> str:
        return f"source not found in container: {self.source}"


class NodeShutdownError(ROSWireException):
    """ROS2 node could not be killed"""
    def __init__(self, node: str) -> None:
        m = f"could not shut down node: {node}"
        super().__init__(m)
