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
        dir_srv = os.path.join(path, 'srv')
        dir_action = os.path.join(path, 'action')

        if files.isdir(dir_msg):
            messages = [MsgFormat.from_file(name, f, files)
                        for f in files.listdir(dir_msg) if f.endswith('.msg')]
        if files.isdir(dir_srv):
            services = [SrvFormat.from_file(name, f, files)
                        for f in files.listdir(dir_srv) if f.endswith('.srv')]
        if files.isdir(dir_action):
            actions = [ActionFormat.from_file(name, f, files)
                       for f in files.listdir(dir_action)
                       if f.endswith('.action')]

        return Package(name,  # type: ignore
                       path,
                       messages,
                       services,
                       actions)
