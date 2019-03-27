# -*- coding: utf-8 -*-
"""
This file implements a proxy for accessing container file systems.
"""
__all__ = ('FileProxy',)

from typing import List, Union, Iterator, Optional, overload
from typing_extensions import Literal
import os
import contextlib
import shlex
import logging
import tempfile
import subprocess

from docker.models.containers import Container as DockerContainer

from .shell import ShellProxy
from ..exceptions import ROSWireException

logger: logging.Logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)


class FileProxy:
    def __init__(self,
                 container_docker: DockerContainer,
                 shell: ShellProxy
                 ) -> None:
        self.__docker_id = container_docker.id
        self.__container_docker = container_docker
        self.__shell = shell

    def copy_from_host(self, path_host: str, path_container: str) -> None:
        """
        Copies a given file or directory tree from the host to the container.

        Parameters:
            fn_host: the file or directory tree that should be copied from
                the host.
            fn_container: the destination path on the container.

        Raises:
            FileNotFoundError: if no file or directory exists at the given path
                on the host.
            FileNotFoundError: if the parent directory of the container
                filepath does not exist.
            OSError: if the copy operation failed.
        """
        id_container: str = self.__docker_id
        if not os.path.exists(path_host):
            m = f"file [{path_host}] does not exist on host"
            raise FileNotFoundError(m)

        path_container_parent: str = os.path.dirname(path_container)
        if not os.path.isdir(path_container_parent):
            m = (f"directory [{path_container_parent}] "
                 "does not exist on container")
            raise FileNotFoundError(m)

        cmd: str = (f"docker cp {shlex.quote(path_host)} "
                    f"{id_container}:{shlex.quote(path_container)}")
        try:
            subprocess.check_call(cmd, shell=True)
        except subprocess.CalledProcessError:
            m = (f"failed to copy file [{path_host}] "
                 f"from host to container [{id_container}]: {path_container}")
            raise OSError(m)

    def copy_to_host(self, path_container: str, path_host: str) -> None:
        """
        Copies a given file or directory tree from the container to the host.

        Parameters:
            fn_container: the file that should be copied from the container.
            fn_host: the destination filepath on the host.

        Raises:
            FileNotFoundError: if no file or directory exists at the given path
                inside the container.
            FileNotFoundError: if the parent directory of the host filepath
                does not exist.
            OSError: if the copy operation failed.
        """
        id_container: str = self.__docker_id
        if not self.exists(path_container):
            m = (f"file [{path_container}] does not exist in container "
                 f"[{id_container}]")
            raise FileNotFoundError(m)

        path_host_parent: str = os.path.dirname(path_host)
        if not os.path.isdir(path_host_parent):
            m = (f"directory [{path_host_parent}] "
                 "does not exist on host machine")
            raise FileNotFoundError(m)

        cmd: str = (f"docker cp {id_container}:{shlex.quote(path_container)} "
                    f"{shlex.quote(path_host)}")
        try:
            subprocess.check_call(cmd, shell=True)
        except subprocess.CalledProcessError:
            m = (f"failed to copy file [{path_container}] "
                 f"from container [{id_container}] to host: {path_host}")
            raise OSError(m)

    def write(self, fn: str, contents: Union[str, bytes]) -> None:
        mode = 'wb' if isinstance(contents, bytes) else 'w'
        dir_parent = os.path.dirname(fn)
        if not self.isdir(dir_parent):
            m = f"parent directory does not exist: {dir_parent}"
            raise FileNotFoundError(m)

        # write to a temporary file on the host and copy to container
        _, fn_host = tempfile.mkstemp()
        try:
            with open(fn_host, mode) as f:
                f.write(contents)
            self.copy_from_host(fn_host, fn)
        finally:
            os.remove(fn_host)

    @overload
    def read(self, fn: str) -> str:
        ...

    @overload
    def read(self, fn: str, binary: Literal[True]) -> bytes:
        ...

    @overload
    def read(self, fn: str, binary: Literal[False]) -> str:
        ...

    def read(self, fn: str, binary: bool = False) -> Union[str, bytes]:
        """
        Reads the contents of a given file.

        Parameters:
            fn: path to the file.
            binary: if true, read the binary contents of the file;
                otherwise, treat the file as a text file.
        """
        mode = 'rb' if binary else 'r'
        if not self.exists(fn):
            raise FileNotFoundError(f"file not found: {fn}")
        if self.isdir(fn):
            raise IsADirectoryError(f"cannot read directory: {fn}")

        _, fn_host_temp = tempfile.mkstemp(suffix='.roswire')
        try:
            self.copy_to_host(fn, fn_host_temp)
            with open(fn_host_temp, mode) as f:
                return f.read()
        finally:
            os.remove(fn_host_temp)

    def exists(self, path: str) -> bool:
        """
        Determines whether a file or directory exists at the given path.
        """
        cmd = f'test -e {shlex.quote(path)}'
        code, output, duration = self.__shell.execute(cmd)
        return code == 0

    def isfile(self, path: str) -> bool:
        """
        Determines whether a regular file exists at a given path.
        """
        cmd = f'test -f {shlex.quote(path)}'
        code, output, duration = self.__shell.execute(cmd)
        return code == 0

    def isdir(self, path: str) -> bool:
        """
        Determines whether a directory exists at a given path.
        """
        cmd = f'test -d {shlex.quote(path)}'
        code, output, duration = self.__shell.execute(cmd)
        return code == 0

    def islink(self, path: str) -> bool:
        """
        Determines whether a symbolic link exists at a given path.
        """
        cmd = f'test -h {shlex.quote(path)}'
        code, output, duration = self.__shell.execute(cmd)
        return code == 0

    def listdir(self,
                d: str,
                *,
                absolute: bool = False
                ) -> List[str]:
        """
        Returns a list of the files belonging to a given directory.

        Parameters:
            d: absolute path to the directory.
            absolute: if True, returns a list of absolute paths; if False,
                returns a list of relative paths.

        Raises:
            OSError: if the given path isn't a directory.
            OSError: if the given path is not a file or directory.
        """
        cmd = f'ls -A -1 {shlex.quote(d)}'
        code, output, duration = self.__shell.execute(cmd)
        if code == 2:
            raise OSError(f"no such file or directory: {d}")
        if not self.isdir(d):
            raise OSError(f"Not a directory: {d}")
        paths: List[str] = output.replace('\r', '').split('\n')
        if absolute:
            paths = [os.path.join(d, p) for p in paths]
        return paths

    def mkdir(self, d: str) -> None:
        """
        Creates a directory at a given path.

        Raises:
            NotADirectoryError: if the parent directory isn't a directory.
            FileNotFoundError: if the parent directory doesn't exist.
            FileExistsError: if a file or directory already exist at the given
                path.
            ROSWireException: if an unexpected error occurred.
        """
        cmd = f"mkdir {shlex.quote(d)}"
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
            raise ROSWireException("unexpected mkdir failure")

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
            ROSWireException: if an unexpected error occurred.
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

        cmd = f'mkdir -p {shlex.quote(d)}'
        code, output, duration = self.__shell.execute(cmd)
        if code != 0:
            raise ROSWireException("unexpected makedirs failure")

    def remove(self, fn: str) -> None:
        """
        Removes a given file.

        Note:
            Does not handle permissions errors.

        Raises:
            FileNotFoundError: if the given file does not exist.
            IsADirectoryError: if the given path is a directory.
            ROSWireException: an unexpected failure occurred.
        """
        cmd = f'rm {shlex.quote(fn)}'
        code, output, duration = self.__shell.execute(cmd)
        if code == 0:
            return

        if not self.exists(fn):
            raise FileNotFoundError(f"file not found: {fn}")
        elif self.isdir(fn):
            raise IsADirectoryError(f"cannot remove directory: {fn}")

        raise ROSWireException("unexpected remove failure")

    def rmdir(self, d: str) -> None:
        """
        Removes a given directory.

        Note:
            Does not handle permissions errors.

        Raises:
            FileNotFoundError: if the given directory does not exist.
            NotADirectoryError: if the given path is not a directory.
            OSError: if the given directory is not empty.
            ROSWireException: an unexpected failure occurred.
        """
        if self.isfile(d):
            raise NotADirectoryError(f"cannot remove file: {d}")
        if not self.isdir(d):
            raise FileNotFoundError(f"directory does not exist: {d}")

        cmd = f'rmdir {shlex.quote(d)}'
        code, output, duration = self.__shell.execute(cmd)
        if code == 0:
            return
        if code == 1 and 'Directory not empty' in output:
            raise OSError(f"directory not empty: {d}")

        raise ROSWireException("unexpected rmdir failure")

    def rmtree(self, d: str) -> None:
        """
        Removes a given directory tree.

        Note:
            Does not handle permissions errors.

        Raises:
            FileNotFoundError: if the given directory does not exist.
            NotADirectoryError: if the given path is not a directory.
            OSError: if the directory tree removal failed.
        """
        if self.isfile(d):
            raise NotADirectoryError(f"cannot remove file: {d}")
        if not self.isdir(d):
            raise FileNotFoundError(f"directory does not exist: {d}")

        cmd = f'rm -rf {shlex.quote(d)}'
        code, output, duration = self.__shell.execute(cmd)
        if code != 0:
            raise OSError(f"failed to remove directory tree: {d}")

    def mktemp(self,
               suffix: Optional[str] = None,
               prefix: Optional[str] = None,
               dirname: Optional[str] = None
               ) -> str:
        """Creates a temporary file.

        Parameters
        ----------
        suffix: str, optional
            an optional suffix for the filename.
        prefix: str, optional
            an optional prefix for the filename.
        dirname: str, optional
            if specified, the temporary file will be created in the given
            directory.

        Raises
        ------
        FileNotFoundError:
            if specified directory does not exist.
        OSError:
            if the temporary file could not be constructed.

        Returns
        -------
        str
            The absolute path of the temporary file.
        """
        template = shlex.quote(f"{prefix if prefix else 'tmp'}.XXXXXXXXXX")
        cmd_parts = ['mktemp', template]
        if suffix:
            cmd_parts += ['--suffix', shlex.quote(suffix)]
        if dirname:
            cmd_parts += ['-p', shlex.quote(dirname)]
            if not self.isdir(dirname):
                m = f'directory does not exist: {dirname}'
                raise FileNotFoundError(m)
        cmd = ' '.join(cmd_parts)

        code, output, duration = self.__shell.execute(cmd)
        # TODO capture context
        if code != 0:
            raise OSError(f"failed to create temporary directory")

        return output

    @contextlib.contextmanager
    def tempfile(self,
                 suffix: Optional[str] = None,
                 prefix: Optional[str] = None,
                 dirname: Optional[str] = None
                 ) -> Iterator[str]:
        """Creates a temporary file within a context.

        Upon exiting the context, the temporary file will be destroyed.

        See Also
        --------
        mktemp: Uses the same arguments to create a temporary file.

        Yields
        ------
        str
            The absolute path of the temporary file.
        """
        fn = self.mktemp(suffix=suffix, prefix=prefix, dirname=dirname)
        logger.debug("created temporary file: %s", fn)
        yield fn
        logger.debug("destroying temporary file: %s", fn)
        try:
            self.remove(fn)
        except FileNotFoundError:
            logger.debug("temporary file already destroyed: %s", fn)
