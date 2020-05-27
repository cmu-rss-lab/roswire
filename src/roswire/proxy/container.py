# -*- coding: utf-8 -*-
__all__ = ('Container', 'ContainerManager')

from typing import Dict, Iterator, Mapping, Optional, Sequence, Union
from uuid import UUID, uuid4
import contextlib
import shutil
import os

import attr
import dockerblade
from docker import DockerClient
from docker import APIClient as DockerAPIClient
from docker.models.images import Image as DockerImage
from docker.models.containers import Container as DockerContainer
from loguru import logger

from ..exceptions import SourceNotFoundError


@attr.s(frozen=True)
class Container:
    """Provides an interface to a Docker container running on this host.

    Attributes
    ----------
    dockerblade: dockerblade.container.Container
        Provides access to the underlying Docker container via Dockerblade.
    sources: Sequence[str]
        The sequence of setup files that should be used to load the ROS
        workspace.
    environment: Mapping[str, str]
        A set of additional environment variables, indexed by name, that should
        be used by the container.
    uuid: UUID
        A unique identifier for this container.
    shell: dockerblade.shell.Shell
        Provides access to a bash shell for this container.
    files: dockerblade.files.FileSystem
        Provides access to the filesystem for this container.
    pid: int
        The PID of this container process on the host machine.
    ws_host: str
        The absolute path to the shared directory for this container's
        workspace on the host machine.
    ip_address: str
        The IP address for this container on the host network.

    Raises
    ------
    SourceNotFoundError:
        If at least one of the given sources could not be found inside the
        container.
    """
    _dockerblade: dockerblade.container.Container = attr.ib(repr=False)
    _sources: Sequence[str] = attr.ib(repr=False)
    uuid: UUID = attr.ib(repr=True)
    ws_host: str = attr.ib(repr=False)
    _environment: Mapping[str, str] = attr.ib(repr=False, factory=dict)
    shell: dockerblade.shell.Shell = attr.ib(init=False, repr=False)
    files: dockerblade.files.FileSystem = attr.ib(init=False, repr=False)

    def __attrs_post_init__(self) -> None:
        files = self._dockerblade.filesystem()

        # #338 ensure that the given sources exist
        for source in self._sources:
            if not files.exists(source):
                raise SourceNotFoundError(source)

        shell = self._dockerblade.shell('/bin/bash',
                                        sources=self._sources,
                                        environment=self._environment)
        object.__setattr__(self, 'shell', shell)
        object.__setattr__(self, 'files', files)

    @property
    def pid(self) -> int:
        return self._dockerblade.pid

    @property
    def ip_address(self) -> str:
        return self._dockerblade.ip_address

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
        return self._dockerblade.persist(repo, tag)


@attr.s(auto_attribs=True)
class ContainerManager:
    """
    Provides an interface for accessing and inspecting Docker images, and
    launching Docker containers.
    """
    _dir_host_workspace: str
    _dockerblade: dockerblade.daemon.DockerDaemon = \
        attr.ib(factory=dockerblade.daemon.DockerDaemon)

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
        logger.debug(f"building docker container: {uuid}")
        dockerc = self.docker_client.containers.create(
            image_or_name,
            '/bin/sh',
            user='root',
            name=uuid,
            entrypoint='/bin/sh -c',
            volumes={dir_host_shared: {'bind': '/.roswire', 'mode': 'rw'}},
            ports=ports,
            stdin_open=True,
            tty=False,
            detach=True)
        logger.debug("created docker container")
        try:
            dockerc.start()
            logger.debug(f"started docker container: {uuid}")
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
               ports: Optional[Dict[int, int]] = None,
               environment: Optional[Dict[str, str]] = None
               ) -> Iterator['Container']:
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
        environment: Mapping[str, str], optional
            An optional set of additional environment variables, indexed by
            name, that should be used by the container.

        Yields
        ------
        Container
            The constructed container.
        """
        uuid = uuid4()
        logger.debug(f"UUID for container: {uuid}")
        if not environment:
            environment = {}
        with contextlib.ExitStack() as stack:
            dir_shared = stack.enter_context(self._build_shared_directory(uuid))  # noqa
            dockerc = stack.enter_context(self._container(image_or_name, dir_shared, uuid, ports=ports))  # noqa
            dockerb = self._dockerblade.attach(dockerc.id)
            yield Container(dockerblade=dockerb,
                            sources=sources,
                            uuid=uuid,
                            ws_host=dir_shared,
                            environment=environment)

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
