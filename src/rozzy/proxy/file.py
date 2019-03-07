__all__ = ['FileProxy']

from bugzoo import BugZoo as BugZooDaemon
from bugzoo import Container as BugZooContainer


class FileProxy:
    def __init__(self,
                 daemon_bugzoo: BugZooDaemon,
                 container_bugzoo: BugZooContainer,
                 ws_host: str
                 ) -> None:
        self.__daemon_bugzoo: BugZooDaemon = daemon_bugzoo
        self.__container_bugzoo: BugZooContainer = container_bugzoo

        self.__dir_ws_host: str = ws_host
        self.__dir_ws_container: str = '/.rozzy'
