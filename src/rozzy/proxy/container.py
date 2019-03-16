__all__ = ('ContainerProxy', )

from typing import Iterator, Optional, Union
from uuid import UUID, uuid4
from ipaddress import IPv4Address, IPv6Address
import ipaddress
import shlex
import contextlib
import logging
import shutil
import os

from docker import DockerClient
from docker import APIClient as DockerAPIClient
from docker.models.images import Image as DockerImage
from docker.models.containers import Container as DockerContainer

from .file import FileProxy
from .shell import ShellProxy
from ..util import Stopwatch
from ..exceptions import RozzyException

logger: logging.Logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)


class ContainerProxy:
    @staticmethod
    @contextlib.contextmanager
    def _build_shared_directory(dir_host_ws: str, uuid: UUID) -> Iterator[str]:
        dir_host = os.path.join(dir_host_ws, 'containers', uuid.hex)
        try:
            logger.debug("creating container directory: %s", dir_host)
            os.makedirs(dir_host, exist_ok=True)
            logger.debug("created container directory: %s", dir_host)
            yield dir_host
        finally:
            if os.path.exists(dir_host):
                logger.debug("destroying container directory: %s", dir_host)
                shutil.rmtree(dir_host)
                logger.debug("destroyed container directory: %s", dir_host)

    @staticmethod
    @contextlib.contextmanager
    def _build_container(client_docker: DockerClient,
                         image_or_name: Union[str, DockerImage],
                         dir_host_shared: str,
                         uuid: UUID
                         ) -> Iterator[DockerContainer]:
        api_docker = DockerAPIClient(base_url='unix://var/run/docker.sock')
        cmd_env_file = ("export | tee /.environment > /dev/null && "
                        "chmod 444 /.environment && "
                        "echo 'ENVFILE CREATED' && /bin/bash")
        cmd_env_file = f"/bin/bash -c {shlex.quote(cmd_env_file)}"
        logger.debug("building docker container: %s", uuid)
        dockerc = client_docker.containers.create(
            image_or_name,
            cmd_env_file,
            user='root',
            name=uuid,
            volumes={dir_host_shared: {'bind': '/.rozzy', 'mode': 'rw'}},
            stdin_open=True,
            tty=False,
            detach=True)
        logger.debug("created docker container")
        try:
            dockerc.start()
            logger.debug("started docker container")

            # wait until .environment file is ready
            env_is_ready = False
            for line in api_docker.logs(dockerc.id, stream=True):
                line = line.strip().decode('utf-8')
                if line == 'ENVFILE CREATED':
                    env_is_ready = True
                    break
            assert env_is_ready

            logger.debug("finished building container: %s", uuid)
            yield dockerc
        finally:
            logger.debug("destroying docker container: %s", uuid)
            dockerc.remove(force=True)
            logger.debug("destroyed docker container: %s", uuid)

    @staticmethod
    @contextlib.contextmanager
    def launch(client_docker: DockerClient,
               dir_host_ws: str,
               image_or_name: Union[str, DockerImage]
               ) -> Iterator['ContainerProxy']:
        stopwatch = Stopwatch()
        stopwatch.start()
        uuid = uuid4()
        logger.debug("UUID for container: %s", uuid)
        with ContainerProxy._build_shared_directory(dir_host_ws, uuid) as dir_host:  # noqa
            args = [client_docker, image_or_name, dir_host_ws, uuid]
            with ContainerProxy._build_container(*args) as dockerc:
                c = ContainerProxy(client_docker, dockerc, uuid, dir_host)
                stopwatch.stop()
                logger.debug("launched container (took %.3f secs)",
                             stopwatch.duration)
                yield c

    def __init__(self,
                 client_docker: DockerClient,
                 container_docker: DockerContainer,
                 uuid: UUID,
                 ws_host: str
                 ) -> None:
        self.__uuid = uuid
        self.__api_docker = \
            DockerAPIClient(base_url='unix://var/run/docker.sock')
        self.__client_docker = client_docker
        self.__container_docker = container_docker
        self.__ws_host = ws_host
        self.__shell = ShellProxy(self.__container_docker)
        self.__files = FileProxy(self.__container_docker, self.__shell)

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
    def ip_address(self) -> str:
        container = self.__container_docker
        api = self.__api_docker
        docker_info = api.inspect_container(container.id)
        address = docker_info['NetworkSettings']['IPAddress']
        try:
            return str(IPv4Address(address))
        except ipaddress.AddressValueError:
            return str(IPv6Address(address))
