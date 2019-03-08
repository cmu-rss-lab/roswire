__all__ = ['FileProxy']

from typing import List
import os
import shlex

from bugzoo import BugZoo as BugZooDaemon
from bugzoo import Container as BugZooContainer

from .shell import ShellProxy
from ..exceptions import RozzyException


class FileProxy:
    def __init__(self,
                 daemon_bugzoo: BugZooDaemon,
                 container_bugzoo: BugZooContainer,
                 ws_host: str,
                 shell: ShellProxy
                 ) -> None:
        self.__daemon_bugzoo: BugZooDaemon = daemon_bugzoo
        self.__container_bugzoo: BugZooContainer = container_bugzoo
        self.__shell: ShellProxy = shell

        self.__dir_ws_host: str = ws_host
        self.__dir_ws_container: str = '/.rozzy'

    def exists(self, path: str) -> bool:
        """
        Determines whether a file or directory exists at the given path.
        """
        cmd = f'test -e "{shlex.quote(path)}"'
        code, output, duration = self.__shell.execute(cmd)
        return code == 0

    def isfile(self, path: str) -> bool:
        """
        Determines whether a regular file exists at a given path.
        """
        cmd = f'test -f "{shlex.quote(path)}"'
        code, output, duration = self.__shell.execute(cmd)
        return code == 0

    def isdir(self, path: str) -> bool:
        """
        Determines whether a directory exists at a given path.
        """
        cmd = f'test -d "{shlex.quote(path)}"'
        code, output, duration = self.__shell.execute(cmd)
        return code == 0

    def islink(self, path: str) -> bool:
        """
        Determines whether a symbolic link exists at a given path.
        """
        cmd = f'test -h "{shlex.quote(path)}"'
        code, output, duration = self.__shell.execute(cmd)
        return code == 0

    def listdir(self, d: str) -> List[str]:
        """
        Returns a list of the files belonging to a given directory.

        Raises:
            OSError: if the given path isn't a directory.
            OSError: if the given path is not a file or directory.
        """
        cmd = f'ls -A -1 "{shlex.quote(d)}"'
        code, output, duration = self.__shell.execute(cmd)
        if code == 2:
            raise OSError(f"no such file or directory: {d}")
        if not self.isdir(d):
            raise OSError(f"Not a directory: {d}")
        return output.replace('\r', '').split('\n')

    def mkdir(self, d: str) -> None:
        """
        Creates a directory at a given path.

        Raises:
            NotADirectoryError: if the parent directory isn't a directory.
            FileNotFoundError: if the parent directory doesn't exist.
            FileExistsError: if a file or directory already exist at the given
                path.
            RozzyException: if an unexpected error occurred.
        """
        cmd = f'mkdir "{shlex.quote(d)}"'
        code, output, duration = self.__shell.execute(cmd)
        if code == 0:
            return

        if self.exists(d):
            raise FileExistsError(d)

        d_parent = os.path.dirname(d)
        if not self.exists(d_parent):
            raise FileNotFoundError(d_parent)
        elif not self.isdir(d_parent):
            raise NotADirectoryError(d_parent)
        else:
            raise RozzyException("unexpected mkdir failure")

    def makedirs(self, d: str, exist_ok: bool = False) -> None:
        """
        Recursively creates a directory at a given path, creating any missing
        intermediate directories along the way.

        Parameters:
            d: the path to the directory.
            exist_ok: specifies whether or not an exception should be raised
                if the given directory already exists.

        Raises:
            FileExistsError: if either (a) `exist_ok=False` and a directory
                already exists at the given path, or (b) a file already exists
                at the given path.
            NotADirectoryError: if the parent directory isn't a directory.
            RozzyException: if an unexpected error occurred.
        """
        d_parent = os.path.dirname(d)
        if self.isdir(d) and not exist_ok:
            m = f"directory already exists: {d}"
            raise FileExistsError(m)
        if self.isfile(d):
            m = f"file already exists at given path: {d}"
            raise FileExistsError(m)
        if self.exists(d_parent) and self.isfile(d_parent):
            m = f"parent directory is actually a file: {d_parent}"
            raise NotADirectoryError(m)

        cmd = f'mkdir -p "{shlex.quote(d)}"'
        code, output, duration = self.__shell.execute(cmd)
        if code != 0:
            raise RozzyException("unexpected makedirs failure")

    def remove(self, fn: str) -> None:
        """
        Removes a given file.

        Note:
            Does not handle permissions errors.

        Raises:
            FileNotFoundError: if the given file does not exist.
            IsADirectoryError: if the given path is a directory.
        """
        cmd = f'rm "{shlex.quote(fn)}"'
        code, output, duration = self.__shell.execute(cmd)
        if code == 0:
            return

        if not self.exists(fn):
            raise FileNotFoundError(f"file not found: {fn}")
        elif self.isdir(fn):
            raise IsADirectoryError(f"cannot remove directory: {fn}")
        else:
            raise RozzyException("unexpected makedirs failure")
