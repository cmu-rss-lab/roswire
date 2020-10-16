# -*- coding: utf-8 -*-
__all__ = ('Service',)

import typing
from typing import Optional

import attr
import dockerblade
import yaml

from ... import exceptions
from ...definitions import Message, MsgFormat, SrvFormat

if typing.TYPE_CHECKING:
    from ...app import AppDescription


@attr.s(slots=True, auto_attribs=True)
class Service:
    """Provides access to a ROS service.

    Attributes
    ----------
    name: str
        The fully qualified name of this service.
    url: str
        The URL of this service.
    format: SrvFormat
        The :code:`.srv` definition for this service.
    """
    name: str
    url: str
    format: SrvFormat
    _description: 'AppDescription'
    _shell: dockerblade.Shell

    def call(self, message: Optional[Message] = None) -> Optional[Message]:
        """Calls this service.

        Parameters
        ----------
        message: Message, optional
            The message, if any, that should be sent to the service.

        Returns
        -------
        Optional[Message]
            The reply produced by the service, if any.
        """
        if not message:
            yml = '{}'
        else:
            yml = yaml.dump(message.to_dict())
        command = f"rosservice call {self.name} '{yml}'"
        try:
            output = self._shell.check_output(command, text=True)
        except dockerblade.exceptions.CalledProcessError as error:
            if error.returncode == 2:
                raise exceptions.ROSWireException('illegal service call args') from error  # noqa
            raise exceptions.ROSWireException('unexpected error during service call') from error  # noqa

        fmt_response: Optional[MsgFormat] = self.format.response
        if not fmt_response:
            return None

        d = yaml.safe_load(output)
        db_type = self._description.types
        return db_type.from_dict(fmt_response, d)
