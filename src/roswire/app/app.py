# -*- coding: utf-8 -*-
__all__ = ("App",)

import os
import tempfile
import typing
from typing import Any, Mapping, Optional, Sequence

import attr
import docker
from docker.models.images import Image as DockerImage

from .description import AppDescription
from .instance import AppInstance
from .. import exceptions as exc

if typing.TYPE_CHECKING:
    from .. import ROSWire


@attr.s(frozen=True, auto_attribs=True, slots=True)
class App:
    """
    Specifies a ROS application in terms of its associated
    Docker image and source files.

    Attributes
    ----------
    image: str
        The name of the Docker image that provides the application.
    sources: Sequence[str]
        The sequence of setup files that should be used to load the ROS
        workspace.

    Raises
    ------
    ImageNotFound
        If the given Docker image could not be found.
    """

    image: str = attr.ib(eq=False)
    sources: Sequence[str]
    _roswire: "ROSWire" = attr.ib(repr=False, eq=False)
    _description: Optional[AppDescription] = attr.ib(
        eq=False, repr=False, default=None
    )
    sha256: str = attr.ib(repr=False, init=False)

    def __attrs_post_init__(self) -> None:
        # compute sha256 of the associated image
        docker_client = self._roswire._dockerblade.client
        try:
            image: DockerImage = docker_client.images.get(self.image)
        except docker.errors.ImageNotFound as err:
            raise exc.ImageNotFound(self.image) from err
        sha256: str = image.id[7:]
        object.__setattr__(self, "sha256", sha256)

    @property
    def description(self) -> AppDescription:
        """Attempts to retrieve the static description of this application.

        Raises
        ------
        NoDescriptionError
            If no description has been computed for this application.
        """
        if not self._description:
            raise exc.NoDescriptionError(app=self)
        return self._description

    def describe(
        self, *, save: bool = True, force: bool = False
    ) -> AppDescription:
        """Produces a static description of this application.

        Parameters
        ----------
        save: bool
            If set to :code:`True`, the description for this application will
            be saved to disk after being computed.
        force: bool
            If set to :code:`True`, a description for this application will be
            recomputed regardless of whether or not a description has already
            been computed to saved to disk.
        """
        if not force and self._description:
            return self._description

        if not force and AppDescription.saved(self):
            description = AppDescription.load(self)
        else:
            description = AppDescription.for_app(self)
        object.__setattr__(self, "_description", description)

        if save:
            description.save()

        return description

    def launch(
        self,
        *,
        name: Optional[str] = None,
        ports: Optional[Mapping[int, int]] = None,
        environment: Optional[Mapping[str, str]] = None,
        volumes: Optional[Mapping[str, Any]] = None,
        require_description: bool = True
    ) -> AppInstance:
        """Launches an instance of this application.

        Parameters
        ----------
        name: str, optional
            An optional name that should be used by the container.
        ports: Mapping[int, int], optional
            An optional dictionary specifying port mappings between the host
            and container, where keys represent container ports and values
            represent host ports.
        environment: Mapping[str, str], optional
            An optional set of additional environment variables, indexed by
            name, that should be used by the system.
        volumes: t.Mapping[str, t.Any], optional
            an optional set of volumes that should be mounted inside the
            container, specified as a dictionary where keys represent a host
            path or volume name, and values are a dictionary containing
            the following keys: :code:`bind`, the path to mount the volume
            inside the container, and :code:`mode`, specifies whether the
            mount should be read-write :code:`rw` or read-only :code:`ro`.
        require_description: bool
            If :code:`True`, this method call will ensure that there is a
            description of the application before launching. If :code:`False`,
            no check will be performed.

        Returns
        -------
        AppInstance
            An interface to the newly created application instance.
        """
        dockerblade = self._roswire._dockerblade
        if require_description and not self._description:
            self.describe()

        environment = dict(environment) if environment else {}
        volumes = dict(volumes) if volumes else {}

        # generate a temporary shared directory
        dir_containers = os.path.join(self._roswire.workspace, "containers")
        os.makedirs(dir_containers, exist_ok=True)
        host_workspace = tempfile.mkdtemp(dir=dir_containers)

        container = dockerblade.provision(
            image=self.image,
            command="/bin/sh",
            user="root",
            name=name,
            entrypoint="/bin/sh -c",
            environment=environment,
            volumes={
                host_workspace: {"bind": "/.roswire", "mode": "rw"},
                **volumes,
            },
            ports=ports,
        )

        instance = AppInstance(
            app=self,
            dockerblade=container,
            host_workspace=host_workspace,
        )
        return instance

    def attach(
        self,
        container_name_or_id: str,
        *,
        require_description: bool = True
    ) -> AppInstance:
        """Attaches to a running container for this application.

        Parameters
        ----------
        container_name_or_id: str
            The name or ID of the container to which ROSWire should be
            attached.
        require_description: bool
            If :code:`True`, this method call will ensure that there is a
            description of the application before launching. If :code:`False`,
            no check will be performed.

        Returns
        -------
        AppInstance
            A ROSWire interface around the given container.
        """
        dockerblade = self._roswire._dockerblade

        if require_description and not self._description:
            self.describe()

        container = dockerblade.attach(container_name_or_id)
        instance = AppInstance(
            app=self,
            dockerblade=container,
            host_workspace=None
        )
        return instance
