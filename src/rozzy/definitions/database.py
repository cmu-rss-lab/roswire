__all__ = ['FormatDatabase']

import os

import attr

from .msg import MsgFormat
from .srv import SrvFormat
from .action import ActionFormat
from ..system import System


@attr.s(frozen=True)
class FormatDatabase:
    @staticmethod
    def for_container(container: Container) -> 'FormatDatabase':
        """
        Constructs a format database for a given container.
        """
        files = container.files
        for pkg_name, pkg_path in container.find_packages():

            dir_msg = os.path.join(pkg_path, 'msg')
            if not files.exists(dir_msg):
                files_msg = []
            else:
                files_msg = files.ls(dir_msg, lambda fn: fn.endswith('.msg'))

            # TODO find all srv files
            dir_srv = os.path.join(pkg_path, 'srv')

            # TODO find all action files
            dir_action = os.path.join(pkg_path, 'action')

            # TODO MsgFormat.from_file(FileProxy, str) -> MsgFormat

    def __init__(self) -> None:
        self.__messages: Dict[str, MsgFormat] = {}
        self.__services: Dict[str, SrvFormat] = {}
        self.__actions: Dict[str, ActionFormat] = {}
