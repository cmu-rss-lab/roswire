__all__ = ['Package']

from typing import Tuple, List
import os

import attr

from .msg import MsgFormat
from .srv import SrvFormat
from .action import ActionFormat
from ..proxy import FileProxy


@attr.s(frozen=True)
class Package:
    name: str = attr.ib()
    path: str = attr.ib()
    messages: Tuple[MsgFormat, ...] = attr.ib(converter=tuple)
    services: Tuple[SrvFormat, ...]  = attr.ib(converter=tuple)
    actions: Tuple[ActionFormat, ...] = attr.ib(converter=tuple)

    @staticmethod
    def build(path: str, files: FileProxy) -> 'Package':
        """
        Constructs a description of a package located at a given path.
        """
        name: str = os.path.basename(path)
        messages: List[MsgFormat] = []
        services: List[SrvFormat] = []
        actions: List[ActionFormat] = []

        if not files.isdir(path):
            raise FileNotFoundError(f"directory does not exist: {path}")

        dir_msg = os.path.join(path, 'msg')
        if files.isdir(dir_msg):
            messages = [MsgFormat.from_file(f, files)
                        for f in files.listdir(dir_msg) if f.endswith('.msg')]

        return Package(name,  # type: ignore
                       path,
                       messages,
                       services,
                       actions)
