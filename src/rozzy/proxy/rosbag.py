__all__ = ['ROSBagProxy']

from typing import Iterator
import contextlib

from .shell import ShellProxy


class ROSBagProxy:
    def __init__(self, shell: ShellProxy) -> None:
        self.__shell = shell
        self.__dir_ctr_bag = None  # FIXME

    @contextlib.contextmanager
    def record(self) -> Iterator['ROSBagRecorderProxy']:
        # create a file for the bag
        cmd = "rosbag record -q -a -O {}"
        cmd = cmd.format(self.__fn_container)
        yield
