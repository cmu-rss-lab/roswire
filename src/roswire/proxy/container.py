# -*- coding: utf-8 -*-
__all__ = ('ContainerProxy', 'ContainerProxyManager')

from typing import Dict, Iterator, Optional, Sequence, Union
from uuid import UUID, uuid4
from ipaddress import IPv4Address, IPv6Address
import ipaddress
import shlex
import contextlib
import logging
import shutil
import os

import attr
import dockerblade
from docker import DockerClient
from docker import APIClient as DockerAPIClient
from docker.models.images import Image as DockerImage
from docker.models.containers import Container as DockerContainer
from loguru import logger

from .file import FileProxy
from .shell import ShellProxy
from ..util import Stopwatch
from ..exceptions import ROSWireException


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
    uuid: UUID = attr.ib()
    pid: int = attr.ib()
    shell: ShellProxy = attr.ib()
    files: FileProxy = attr.ib()
    ws_host: str = attr.ib()
    _api_docker: DockerAPIClient = attr.ib()
    _container_docker: DockerContainer = attr.ib()

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
                              api_docker=api_docker,
                              container_docker=container_docker)

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


@attr.s(auto_attribs=True)
class ContainerProxyManager:
    """
    Provides an interface for accessing and inspecting Docker images, and
    launching Docker containers.
    """
    _dir_host_workspace: str
    _dockerblade: dockerblade.DockerDaemon = \
        attr.ib(factory=dockerblade.DockerDaemon)

    @property
    def docker_client(self) -> DockerClient:
        return self._dockerblade.client

    @property
    def docker_api(self) -> DockerAPIClient:
        return self._dockerblade.api

    @contextlib.contextmanager
    def _build_shared_directory(self, uuid: UUID) -> Iterator[str]:
        dir_host = os.path.join(self._dir_host_workspace,
                                'containers',
                                uuid.hex)
        try:
            logger.debug(f"creating container directory: {dir_host}")
            os.makedirs(dir_host, exist_ok=True)
            logger.debug(f"created container directory: {dir_host}")
            yield dir_host
        finally:
            if os.path.exists(dir_host):
                logger.debug(f"destroying container directory: {dir_host}")
                shutil.rmtree(dir_host)
                logger.debug(f"destroyed container directory: {dir_host}")

    @contextlib.contextmanager
    def _container(self,
                   image_or_name: Union[str, DockerImage],
                   dir_host_shared: str,
                   uuid: UUID,
                   *,
                   ports: Optional[Dict[int, int]] = None
                   ) -> Iterator[DockerContainer]:
        cmd_env_file = ("export | tee /.environment > /dev/null && "
                        "chmod 444 /.environment && "
                        "echo 'ENVFILE CREATED' && /bin/bash")
        cmd_env_file = f"/bin/bash -c {shlex.quote(cmd_env_file)}"
        logger.debug(f"building docker container: {uuid}")
        dockerc = self.docker_client.containers.create(
            image_or_name,
            cmd_env_file,
            user='root',
            name=uuid,
            volumes={dir_host_shared: {'bind': '/.roswire', 'mode': 'rw'}},
            ports=ports,
            stdin_open=True,
            tty=False,
            detach=True)
        logger.debug("created docker container")
        try:
            dockerc.start()
            logger.debug("started docker container")

            # wait until .environment file is ready
            env_is_ready = False
            for line in self.docker_api.logs(dockerc.id, stream=True):
                line = line.strip().decode('utf-8')
                if line == 'ENVFILE CREATED':
                    env_is_ready = True
                    break
            assert env_is_ready

            logger.debug(f"finished building container: {uuid}")
            yield dockerc
        finally:
            logger.debug(f"destroying docker container: {uuid}")
            dockerc.remove(force=True)
            logger.debug(f"destroyed docker container: {uuid}")

    @contextlib.contextmanager
    def launch(self,
               image_or_name: Union[str, DockerImage],
               sources: Sequence[str],
               *,
               ports: Optional[Dict[int, int]] = None
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
        sources: Sequence[str]
            The sequence of setup files that should be used to load the ROS
            workspace.
        ports: Dict[int, int], optional
            An optional dictionary specifying port mappings between the host
            and container, where keys represent container ports and values
            represent host ports.

        Yields
        ------
        ContainerProxy
            The constructed container.
        """
        uuid = uuid4()
        logger.debug(f"UUID for container: {uuid}")
        with self._build_shared_directory(uuid) as dir_shared:
            with self._container(image_or_name, dir_shared, uuid, ports=ports) as dockerc:  # noqa: pycodestyle
                yield ContainerProxy._from_docker(self.docker_api,
                                                  dockerc,
                                                  uuid,
                                                  dir_shared)

    def image(self, tag: str) -> DockerImage:
        """Retrieves the Docker image with a given tag."""
        return self.docker_client.images.get(tag)

    def image_sha256(self, tag_or_image: Union[str, DockerImage]) -> str:
        """Computes the SHA256 for a given Docker image."""
        image: DockerImage
        if isinstance(tag_or_image, str):
            image = self.image(tag_or_image)
        else:
            image = tag_or_image
        return image.id[7:]
