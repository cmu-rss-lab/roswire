__all__ = ('ROSTopicProxy',)

import yaml

from .shell import ShellProxy
from .. import exceptions
from ..description import SystemDescription
from ..definitions import Message


class ROSTopicProxy:
    def __init__(self,
                 description: SystemDescription,
                 shell: ShellProxy
                 ) -> None:
        self.__description = description
        self.__shell = shell

    def publish(self, topic: str, message: Message) -> None:
        yml = yaml.dump(message.to_dict())
        cmd = f"rostopic pub {topic} {message.format.fullname} '{yml}'"
        retcode, output, duration = self.__shell.execute(cmd)
        if retcode != 0:
            m = 'unexpected error during rostopic pub call'
            raise exceptions.RozzyException(m)
