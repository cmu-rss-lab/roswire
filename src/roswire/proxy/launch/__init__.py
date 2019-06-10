# -*- coding: utf-8 -*-
"""
This file implements a proxy for parsing the contents of launch files.
"""
__all__ = ('LaunchFileReader',)

from typing import List, Optional, Sequence, Collection
import logging
import xml.etree.ElementTree as ET

import attr

from ..file import FileProxy
from ...exceptions import FailedToParseLaunchFile

logger: logging.Logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)

_TAG_TO_LOADER = {}


def tag(name: str, legal_attributes: Collection[str] = tuple()):
    legal_attributes = frozenset(legal_attributes)
    def wrap(loader):
        def wrapped(elem: ET.Element):
            for attribute in elem.attrib:
                if attribute not in legal_attributes:
                    raise FailedToParseLaunchFile(m)
            return loader(elem)
        _TAG_TO_LOADER[name] = wrapped
        return staticmethod(wrapped)
    return wrap


@attr.s(slots=True)
class LaunchConfig:
    nodes = attr.ib(type=List[str])


@attr.s(frozen=True, slots=True)
class NodeConfig:
    name: str = attr.ib()
    typ: str = attr.ib()
    pkg: str = attr.ib()


class LaunchFileReader:
    def __init__(self, files: FileProxy) -> None:
        self.__files = files

    def _parse_file(self, fn: str) -> ET.Element:
        """Parses a given XML launch file to a root XML element."""
        root = ET.fromstring(self.__files.read(fn))
        if root.tag != 'launch':
            m = 'root of launch file must have <launch></launch> tags'
            raise FailedToParseLaunchFile(m)
        return root

    def _load_tags(self, tags: Sequence[ET.Element]) -> None:
        for tag in (t for t in tags if t.tag in _TAG_TO_LOADER):
            loader = _TAG_TO_LOADER[tag.tag]
            loader(tag)

    @tag('arg', ['name', 'type', 'pkg'])
    def _load_node_tag(tag: ET.Element) -> None:
        name = tag.attrib['name']
        pkg = tag.attrib['type']
        node_type = tag.attrib['type']

        node = NodeConfig(name=name,
                          pkg=pkg,
                          node_type=node_type)
        logger.debug("found node: %s", node)

    @tag('arg', ['name', 'default', 'value', 'doc'])
    def _load_arg_tag(tag: ET.Element) -> None:
        name = tag.attrib['name']
        logger.debug("found attribute: %s", name)

    @tag('include', ['file'])
    def _load_include_tag(tag: ET.Element) -> None:
        include_filename = tag.attrib['file']
        logger.debug("include file: %s", include_filename)

    def read(self, fn: str, argv: Optional[Sequence[str]] = None) -> None:
        """Parses the contents of a given launch file.

        Reference
        ---------
            http://wiki.ros.org/roslaunch/XML/node
            http://docs.ros.org/kinetic/api/roslaunch/html/roslaunch.xmlloader.XmlLoader-class.html
        """
        # TODO load system arguments into context
        if not argv:
            argv = []

        launch = self._parse_file(fn)
        self._load_tags(list(launch))
