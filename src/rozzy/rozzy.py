__all__ = ('Rozzy',)

from typing import Optional, Dict, Iterator
from uuid import uuid4
import os
import pathlib
import logging
import contextlib
import shutil

from bugzoo import BugZoo as BugZooDaemon
from bugzoo import Bug as BugZooSnapshot

from .exceptions import RozzyException
from .system import System, SystemDescription
from .proxy import ContainerProxy

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
    def launch(self, desc: SystemDescription) -> Iterator[System]:
        snapshot: BugZooSnapshot = self.bugzoo.bugs[desc.image]
        with ContainerProxy.launch(self.bugzoo,
                                   self.workspace,
                                   snapshot) as container:
            container = container
            yield System(container)
