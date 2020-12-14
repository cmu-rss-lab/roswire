# -*- coding: utf-8 -*-
__all__ = ("SystemState",)

from abc import ABC, abstractmethod
from typing import AbstractSet, Collection, Mapping


class SystemState(ABC):

    @property
    @abstractmethod
    def publishers(self) -> Mapping[str, Collection[str]]:
        ...

    @property
    @abstractmethod
    def subscribers(self) -> Mapping[str, Collection[str]]:
        ...

    @property
    @abstractmethod
    def services(self) -> Mapping[str, Collection[str]]:
        ...

    @property
    @abstractmethod
    def nodes(self) -> AbstractSet[str]:
        ...

    @property
    @abstractmethod
    def topics(self) -> AbstractSet[str]:
        ...
