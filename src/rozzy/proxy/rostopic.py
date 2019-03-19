__all__ = ('ROSTopicProxy',)

import yaml
import logging

from .shell import ShellProxy
from .. import exceptions
from ..description import SystemDescription
from ..definitions import Message

logger: logging.Logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)


class ROSTopicProxy:
    def __init__(self,
                 description: SystemDescription,
                 shell: ShellProxy
                 ) -> None:
        self.__description = description
        self.__shell = shell

    def publish(self, topic: str, message: Message) -> None:
        """
        Publishes a given message on a named topic.

        Warning:
            this method blocks for three seconds as the given message is
            latched.
        """
        yml = yaml.dump(message.to_dict())
        cmd = f"rostopic pub --once {topic} {message.format.fullname} '{yml}'"
        retcode, output, duration = self.__shell.execute(cmd)
        if retcode != 0:
            m = 'unexpected error during rostopic pub call'
            raise exceptions.RozzyException(m)
