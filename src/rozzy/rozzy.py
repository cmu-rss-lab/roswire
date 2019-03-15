__all__ = ['Rozzy']

from typing import Optional, Dict, Iterator
from uuid import uuid4
import os
import pathlib
import logging
import contextlib
import shutil

from bugzoo import BugZoo as BugZooDaemon
from bugzoo import Bug as BugZooSnapshot
from bugzoo import Container as BugZooContainer

from .exceptions import RozzyException
from .system import System
from .system_instance import SystemInstance

logger = logging.getLogger(__name__)  # type: logging.Logger
logger.setLevel(logging.DEBUG)


class Rozzy:
    def __init__(self,
                 dir_workspace: Optional[str] = None,
                 daemon_bugzoo: Optional[BugZooDaemon] = None
                 ) -> None:
        if not dir_workspace:
            logger.debug("no workspace specified: using default workspace.")
            dir_home = os.path.expanduser("~")
            dir_workspace = os.path.join(dir_home, ".rozzy")
            logger.debug("default workspace: %s", dir_workspace)
            if not os.path.exists(dir_workspace):
                logger.debug("initialising default workspace")
                os.mkdir(dir_workspace)
        else:
            logger.debug("using specified workspace: %s", dir_workspace)
            if not os.path.exists(dir_workspace):
                m = "workspace not found: {}".format(dir_workspace)
                raise RozzyException(m)

        self.__dir_workspace = os.path.abspath(dir_workspace)

        if not daemon_bugzoo:
            logger.debug("no bugzoo daemon provided; creating one instead")
            self.__bugzoo = BugZooDaemon()
        else:
            logger.debug("using provided bugzoo daemon")
            self.__bugzoo = daemon_bugzoo

    @property
    def workspace(self) -> str:
        """
        The absolute path to the workspace directory.
        """
        return self.__dir_workspace

    @property
    def bugzoo(self) -> BugZooDaemon:
        """
        The BugZoo daemon used by Rozzy.
        """
        return self.__bugzoo

    @contextlib.contextmanager
    def launch(self, system: System) -> Iterator[SystemInstance]:
        bz: BugZooDaemon = self.bugzoo
        snapshot: BugZooSnapshot = bz.bugs[system.image]

        # generate a unique identifier for the container
        uuid = uuid4()
        logger.debug("UUID for container: %s", uuid)
        dir_host = os.path.join(self.workspace, 'containers', uuid.hex)
        dir_container = '/.rozzy'
        volumes = {dir_host: {'bind': dir_host, 'mode': 'rw'}}
        bz_container = None  # type: Optional[BugZooContainer]

        try:
            logger.debug("creating container directory: %s", dir_host)
            os.makedirs(dir_host, exist_ok=True)
            logger.debug("created container directory: %s", dir_host)

            # FIXME launch as user
            logger.debug("launching docker container")
            bz_container = bz.containers.provision(
                snapshot,
                volumes=volumes)
            logger.debug("launched docker container")
            instance = SystemInstance(bz, bz_container, uuid, dir_host)
            yield instance

        finally:
            if bz_container:
                bzid = bz_container.id
                logger.debug("destroying docker container: %s", bzid)
                del bz.containers[bzid]
                logger.debug("destroyed docker container: %s", bzid)

            logger.debug("destroying container directory: %s", dir_host)
            if os.path.exists(dir_host):
                shutil.rmtree(dir_host)
            logger.debug("destroyed container directory: %s", dir_host)
