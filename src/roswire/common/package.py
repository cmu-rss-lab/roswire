# -*- coding: utf-8 -*-
__all__ = ("Package", "PackageDatabase")

import typing
from abc import ABC, abstractmethod
from typing import (
    Any,
    Collection,
    Dict,
    Generic,
    List,
    Mapping,
    Optional,
    Sequence
)

from .action import ActionFormat
from .msg import MsgFormat
from .srv import SrvFormat

if typing.TYPE_CHECKING:
    from .. import AppInstance

MF = typing.TypeVar("MF", bound=MsgFormat)
SF = typing.TypeVar("SF", bound=SrvFormat)
AF = typing.TypeVar("AF", bound=ActionFormat)


class Package(Generic[MF, SF, AF], ABC):
    name: str
    path: str
    messages: Collection[MF]
    services: Collection[SF]
    actions: Collection[AF]

    @classmethod
    @abstractmethod
    def build(cls, path: str, app_instance: "AppInstance") -> "Package":
        ...

    @classmethod
    @abstractmethod
    def from_dict(cls, dict: Dict[str, Any]) -> "Package":
        ...

    def to_dict(self) -> Dict[str, Any]:
        d = {
            "name": self.name,
            "path": self.path,
            "messages": [m.to_dict() for m in self.messages],
            "services": [s.to_dict() for s in self.services],
            "actions": [a.to_dict() for a in self.actions],
        }
        return d


class PackageDatabase(ABC, Mapping[str, Package]):
    """
    An immutable database of packages, represented as :class:`Package`
    instances, indexed by their names, given as :class:`str`.

    Note
    ----
        Implements most :class:`dict` operations via
        :class:`abc.collections.Mapping`,
        including :code:`db['name']`, :code:`len(db)`), :code:`db.keys()`,
        :code:`db.values()`, and :code:`iter(db)`.
        As instances of this class are immutable, no destructive
        :class:`dict` operations are provided (e.g., :code:`del db['foo'])`
        and `db['foo'] = bar`).
    """

    _paths_in_package: Sequence[str]

    @classmethod
    @abstractmethod
    def build(cls,
              app_instance: "AppInstance",
              paths: Optional[List[str]] = None
              ) -> "PackageDatabase":
        ...

    @property
    def paths(self) -> Sequence[str]:
        return self._paths_in_package

    @classmethod
    @abstractmethod
    def from_dict(cls, d: List[Dict[str, Any]]) -> "PackageDatabase":
        ...

    def to_dict(self) -> List[Dict[str, Any]]:
        return [p.to_dict() for p in self.values()]
