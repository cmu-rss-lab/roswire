# -*- coding: utf-8 -*-
"""
This file implements a proxy for parsing the contents of launch files.
"""
__all__ = ('LaunchFileReader',)

import abc
import typing
from typing import (Optional, Sequence)


from .config import LaunchConfig

if typing.TYPE_CHECKING:
    from ... import AppInstance


class LaunchFileReader(abc.ABC):

    @classmethod
    @abc.abstractmethod
    def for_app_instance(cls, app_instance: 'AppInstance') -> 'LaunchFileReader':
        ...

    @abc.abstractmethod
    def read(self,
             fn: str,
             argv: Optional[Sequence[str]] = None
             ) -> LaunchConfig:
        ...

    @abc.abstractmethod
    def locate_node_binary(self,
                           package: str,
                           node_type: str) -> str:
        ...



