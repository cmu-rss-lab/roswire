# -*- coding: utf-8 -*-
__all__ = ('ROS2StateProbe',)

import attr
import typing
from typing import Dict, List, Optional

import dockerblade
from loguru import logger

from ..proxy import SystemState

if typing.TYPE_CHECKING:
    from ..app import AppInstance


@attr.s(frozen=True, auto_attribs=True)
class ROS2StateProbe:
    """Provides an interface for obtaining the instantaneous state of a ROS
    system in terms of its publishers, subscribers, and services."""
    _app_instance: 'AppInstance' = attr.ib()

    @classmethod
    def for_app_instance(cls, app_instance: 'AppInstance') -> 'ROS2StateProbe':
        return ROS2StateProbe(app_instance=app_instance)

    def probe(self) -> SystemState:
        """Obtains the instantaneous state of the associated ROS system."""
        shell = self._app_instance.shell
        node_to_state: Dict[Optional[str], Dict[str, List[str]]] = {'pub': {},
                                                                    'sub': {},
                                                                    'serv': {}}
        command = "ros2 node list"
        try:
            output = shell.check_output(command, text=True)
        except dockerblade.exceptions.CalledProcessError:
            logger.debug("Unable to retrieve rosnode list from command line")
            raise
        node_names = output.split('\r\n')
        for node_name in node_names:
            if node_name == '':
                continue
            command_info = f"ros2 node info '{node_name}'"
            mode: Optional[str] = None
            try:
                output = shell.check_output(command_info, text=True)
            except dockerblade.exceptions.CalledProcessError:
                logger.debug(f"Unable to retrieve {command_info}")
                raise
            output = output.replace(' ', '')
            lines = output.split('\r\n')
            for line in lines:
                if "Publishers:" in line:
                    mode = 'pub'
                    continue
                elif "Subscribers:" in line:
                    mode = 'sub'
                    continue
                elif "Services:" in line:
                    mode = 'serv'
                    continue
                elif "Action Servers:" in line:
                    break

                if mode:
                    name = line.partition(':')[0]
                    if name in node_to_state[mode]:
                        node_to_state[mode][name].append(node_name)
                    else:
                        node_to_state[mode][name] = [node_name]

        state = SystemState(publishers=node_to_state['pub'],
                            subscribers=node_to_state['sub'],
                            services=node_to_state['serv'])
        return state

    __call__ = probe
