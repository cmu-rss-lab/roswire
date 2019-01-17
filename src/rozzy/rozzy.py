from typing import Optional, Dict
import os
import pathlib
import logging
import contextlib

import bugzoo

from .exceptions import RozzyException

logger = logging.getLogger(__name__)  # type: logging.Logger
logger.setLevel(logging.DEBUG)


class Rozzy(object):
    def __init__(self,
                 dir_workspace: Optional[str] = None,
                 daemon_bugzoo: Optional[bugzoo.BugZoo] = None
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
            self.__bugzoo = bugzoo.BugZoo()
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
    def bugzoo(self) -> bugzoo.BugZoo:
        """
        The BugZoo daemon used by Rozzy.
        """
        return self.__bugzoo
