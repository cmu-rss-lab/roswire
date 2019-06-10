# -*- coding: utf-8 -*-
"""
This file implements a proxy for parsing the contents of launch files.
"""
__all__ = ('LaunchFileReader',)

import logging
import xml.etree.ElementTree as ET

from .file import FileProxy
from ..exceptions import FailedToParseLaunchFile

logger: logging.Logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)


class LaunchFileReader:
    def __init__(self, files: FileProxy) -> None:
        self.__files = files

    def read(self, fn: str) -> None:
        """Parses the contents of a given launch file.

        Reference
        ---------
            http://wiki.ros.org/roslaunch/XML/node
            http://docs.ros.org/kinetic/api/roslaunch/html/roslaunch.xmlloader.XmlLoader-class.html
        """
        root = ET.fromstring(self.__files.read(fn))
        if root.tag != 'launch':
            m = 'root of launch file must have <launch></launch> tags'
            raise FailedToParseLaunchFile(m)

        # TODO handle remap tags

        # TODO handle env tags

        # TODO handle param tags

        # TODO handle include tags: file

        # TODO parse arg tags (name, default, value)
        for xml_arg in root.findall('arg'):
            try:
                name = xml_arg.attrib['name']
            except:
                m = "<arg> tag missing 'name' attribute"
                raise FailedToParseLaunchFile(m)

            logger.debug("found attribute: %s", name)

        # TODO parse rosparam tags

        # TODO parse node tags
