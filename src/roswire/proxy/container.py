# -*- coding: utf-8 -*-
__all__ = ('ContainerProxy', 'ContainerProxyManager')

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
import attr

from .file import FileProxy
from .shell import ShellProxy
from ..util import Stopwatch
from ..exceptions import ROSWireException

logger: logging.Logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)


@attr.s(frozen=True)
class ContainerProxy:
    """Provides an interface to a Docker container running on this host.

    Attributes
    ----------
    uuid: UUID
        A unique identifier for this container.
    shell: ShellProxy
        Provides access to a bash shell for this container.
    files: FileProxy
        Provides access to the filesystem for this container.
    pid: int
        The PID of this container process on the host machine.
    ws_host: str
        The absolute path to the shared directory for this container's
        workspace on the host machine.
    ip_address: str
        The IP address for this container on the host network.
    """
    uuid = attr.ib(type=UUID)
    pid = attr.ib(type=int)
    shell = attr.ib(type=ShellProxy)
    files = attr.ib(type=FileProxy)
    ws_host = attr.ib(type=str)
    _api_docker = attr.ib(type=DockerAPIClient)
    _container_docker = attr.ib(type=DockerContainer)

    @staticmethod
    def _from_docker(api_docker: DockerAPIClient,
                     container_docker: DockerContainer,
                     uuid: UUID,
                     ws_host: str
                     ) -> 'ContainerProxy':
        """Constructs a proxy from a given Docker container."""
        info = api_docker.inspect_container(container_docker.id)
        pid = int(info['State']['Pid'])
        shell = ShellProxy(api_docker, container_docker, pid)
        files = FileProxy(container_docker, shell)
        return ContainerProxy(uuid=uuid,
                              shell=shell,
                              files=files,
                              pid=pid,
                              ws_host=ws_host,
                              _api_docker=api_docker,
                              _container_docker=container_docker)

    @property
    def ip_address(self) -> str:
        container = self._container_docker
        api = self._api_docker
        docker_info = api.inspect_container(container.id)
        address = docker_info['NetworkSettings']['IPAddress']
        try:
            return str(IPv4Address(address))
        except ipaddress.AddressValueError:
            return str(IPv6Address(address))

    def persist(self,
                repo: Optional[str] = None,
                tag: Optional[str] = None
                ) -> DockerImage:
        """Persists this container to an image.

        Parameters
        ----------
        repo: str, optional
            The name of the repository to which the image should belong.
        tag: str, optional
            The tag that should be used for the image.

        Returns
        -------
        DockerImage
            A description of the persisted image.
        """
        return self._container_docker.commit(repo, tag)


class ContainerProxyManager:
    """
    Provides an interface for accessing and inspecting Docker images, and
    launching Docker containers.
    """
    def __init__(self,
                 client_docker: DockerClient,
                 api_docker: DockerAPIClient,
                 dir_host_ws: str
                 ) -> None:
        self.__dir_host_ws = dir_host_ws
        self.__client_docker = client_docker
        self.__api_docker = api_docker

    @property
    def client_docker(self) -> DockerClient:
        return self.__client_docker

    @property
    def api_docker(self) -> DockerAPIClient:
        return self.__api_docker

    @contextlib.contextmanager
    def _build_shared_directory(self, uuid: UUID) -> Iterator[str]:
        dir_host = os.path.join(self.__dir_host_ws, 'containers', uuid.hex)
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

    @contextlib.contextmanager
    def _container(self,
                   image_or_name: Union[str, DockerImage],
                   dir_host_shared: str,
                   uuid: UUID
                   ) -> Iterator[DockerContainer]:
        cmd_env_file = ("export | tee /.environment > /dev/null && "
                        "chmod 444 /.environment && "
                        "echo 'ENVFILE CREATED' && /bin/bash")
        cmd_env_file = f"/bin/bash -c {shlex.quote(cmd_env_file)}"
        logger.debug("building docker container: %s", uuid)
        dockerc = self.__client_docker.containers.create(
            image_or_name,
            cmd_env_file,
            user='root',
            name=uuid,
            volumes={dir_host_shared: {'bind': '/.roswire', 'mode': 'rw'}},
            stdin_open=True,
            tty=False,
            detach=True)
        logger.debug("created docker container")
        try:
            dockerc.start()
            logger.debug("started docker container")

            # wait until .environment file is ready
            env_is_ready = False
            for line in self.__api_docker.logs(dockerc.id, stream=True):
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

    @contextlib.contextmanager
    def launch(self,
               image_or_name: Union[str, DockerImage]
               ) -> Iterator['ContainerProxy']:
        """
        Launches a context-managed Docker container for a given image. Upon
        exiting the context, whether gracefully or abruptly (i.e., via an
        unhandled exception), the container will be automatically destroyed.

        Parameters
        ----------
        image_or_name: Union[str, DockerImage]
            The image that should be used to create the container, or the name
            of that image.

        Yields
        ------
        ContainerProxy
            The constructed container.
        """
        api_docker = self.__api_docker
        uuid = uuid4()
        logger.debug("UUID for container: %s", uuid)
        with self._build_shared_directory(uuid) as dir_shared:
            with self._container(image_or_name, dir_shared, uuid) as dockerc:
                yield ContainerProxy._from_docker(api_docker,
                                                  dockerc,
                                                  uuid,
                                                  dir_shared)

    def image(self, tag: str) -> DockerImage:
        """Retrieves the Docker image with a given tag."""
        return self.__client_docker.images.get(tag)

    def image_sha256(self, tag_or_image: Union[str, DockerImage]) -> str:
        """Computes the SHA256 for a given Docker image."""
        image: DockerImage
        if isinstance(tag_or_image, str):
            image = self.image(tag_or_image)
        else:
            image = tag_or_image
        return image.id[7:]
