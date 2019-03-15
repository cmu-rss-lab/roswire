__all__ = ('ContainerProxy', )

from typing import Iterator, Optional
from uuid import UUID, uuid4
from ipaddress import IPv4Address, IPv6Address
import ipaddress
import contextlib
import logging
import shutil
import os

from docker import APIClient as DockerAPIClient
from docker.models.containers import Container as DockerContainer
from bugzoo import BugZoo as BugZooDaemon
from bugzoo import Container as BugZooContainer
from bugzoo import Bug as BugZooSnapshot

from .file import FileProxy
from .shell import ShellProxy
from ..exceptions import RozzyException

logger = logging.getLogger(__name__)  # type: logging.Logger
logger.setLevel(logging.DEBUG)


class ContainerProxy:
    @staticmethod
    @contextlib.contextmanager
    def launch(bz: BugZooDaemon,
               dir_host_workspace: str,
               snapshot: BugZooSnapshot
               ) -> Iterator['ContainerProxy']:
        # generate a unique identifier for the container
        uuid = uuid4()
        logger.debug("UUID for container: %s", uuid)

        # create a dedicated shared directory for the container
        dir_host = os.path.join(dir_host_workspace, 'containers', uuid.hex)
        dir_container = '/.rozzy'
        volumes = {dir_host: {'bind': dir_host, 'mode': 'rw'}}
        bz_container: Optional[BugZooContainer] = None

        try:
            logger.debug("creating container directory: %s", dir_host)
            os.makedirs(dir_host, exist_ok=True)
            logger.debug("created container directory: %s", dir_host)

            # FIXME launch as user
            logger.debug("launching docker container")
            bz_container = bz.containers.provision(snapshot, volumes=volumes)
            logger.debug("launched docker container")
            container = ContainerProxy(bz, bz_container, uuid, dir_host)
            yield container

        finally:
            if bz_container:
                bzid = bz_container.id
                logger.debug("destroying docker container: %s", bzid)
                del bz.containers[bzid]
                logger.debug("destroyed docker container: %s", bzid)

            # destroy shared directory on host machine
            if os.path.exists(dir_host):
                logger.debug("destroying container directory: %s", dir_host)
                shutil.rmtree(dir_host)
                logger.debug("destroyed container directory: %s", dir_host)

    def __init__(self,
                 daemon_bugzoo: BugZooDaemon,
                 container_bugzoo: BugZooContainer,
                 uuid: UUID,
                 ws_host: str
                 ) -> None:
        self.__uuid = uuid
        self.__daemon_bugzoo = daemon_bugzoo
        self.__container_bugzoo = container_bugzoo
        self.__ws_host = ws_host

        dockerc = self._docker
        self.__shell = ShellProxy(dockerc)
        self.__files = FileProxy(dockerc, self.__shell)

    @property
    def uuid(self) -> UUID:
        return self.__uuid

    @property
    def shell(self) -> ShellProxy:
        return self.__shell

    @property
    def files(self) -> FileProxy:
        return self.__files

    @property
    def ws_host(self) -> str:
        return self.__ws_host

    @property
    def _docker(self) -> DockerContainer:
        bzid: str = self.__container_bugzoo.id
        bzc = self.__daemon_bugzoo.containers
        return getattr(bzc, '_ContainerManager__dockerc')[bzid]

    @property
    def ip_address(self) -> str:
        container = self._docker
        api = DockerAPIClient(base_url='unix://var/run/docker.sock')
        docker_info = api.inspect_container(container.id)
        address = docker_info['NetworkSettings']['IPAddress']
        try:
            return str(IPv4Address(address))
        except ipaddress.AddressValueError:
            return str(IPv6Address(address))
