__all__ = ['Package']

from typing import Tuple, List, Dict, Union, Any
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
    services: Tuple[SrvFormat, ...] = attr.ib(converter=tuple)
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
                        for f in files.listdir(dir_msg, absolute=True)
                        if f.endswith('.msg')]
        if files.isdir(dir_srv):
            services = [SrvFormat.from_file(name, f, files)
                        for f in files.listdir(dir_srv, absolute=True)
                        if f.endswith('.srv')]
        if files.isdir(dir_action):
            actions = [ActionFormat.from_file(name, f, files)
                       for f in files.listdir(dir_action, absolute=True)
                       if f.endswith('.action')]

        return Package(name,  # type: ignore
                       path,
                       messages,
                       services,
                       actions)

    @staticmethod
    def from_dict(d: Dict[str, Any]) -> 'Package':
        name: str = d['name']
        messages: List[MsgFormat] = [MsgFormat.from_dict(dd, package=name)
                                     for dd in d.get('messages', [])]
        services: List[SrvFormat] = [SrvFormat.from_dict(dd, package=name)
                                     for dd in d.get('services', [])]
        actions: List[ActionFormat] = [ActionFormat.from_dict(dd, package=name)
                                       for dd in d.get('actions', [])]
        return Package(d['name'],  # type: ignore
                       d['path'],
                       messages,
                       services,
                       actions)

    def to_dict(self) -> Dict[str, Any]:
        d = {'name': self.name,
             'path': self.path,
             'messages': [m.to_dict() for m in self.messages],
             'services': [s.to_dict() for s in self.services],
             'actions': [a.to_dict() for a in self.actions]}
        return d
