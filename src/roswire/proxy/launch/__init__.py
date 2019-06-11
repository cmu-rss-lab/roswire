# -*- coding: utf-8 -*-
"""
This file implements a proxy for parsing the contents of launch files.
"""
__all__ = ('LaunchFileReader',)

from typing import (List, Optional, Sequence, Collection, Dict, Any, Mapping,
                    Tuple)
import logging
import xml.etree.ElementTree as ET

import attr

from ..substitution import resolve as resolve_args
from ..shell import ShellProxy
from ..file import FileProxy
from ...exceptions import FailedToParseLaunchFile

logger: logging.Logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)

_TAG_TO_LOADER = {}


@attr.s(slots=True)
class LaunchConfig:
    nodes = attr.ib(type=List[str])


@attr.s(frozen=True, slots=True)
class NodeConfig:
    namespace: str = attr.ib()
    name: str = attr.ib()
    typ: str = attr.ib()
    pkg: str = attr.ib()


@attr.s(frozen=True, slots=True)
class LaunchContext:
    filename: str = attr.ib()
    resolve_dict: Mapping[str, Any] = attr.ib()
    include_resolve_dict: Mapping[str, Any] = attr.ib()
    parent: 'LaunchContext' = attr.ib(default=None)
    namespace: str = attr.ib(default='/')
    arg_names: Tuple[str, ...] = attr.ib(default=tuple())
    env_args: Tuple[Tuple[str, str], ...] = attr.ib(default=tuple())

    def with_argv(self, argv: Sequence[str]) -> 'LaunchContext':
        # ignore parameter assignment mappings
        logger.debug("loading argv: %s", argv)
        mappings: Dict[str, str] = {}
        for arg in (a for a in argv if ':=' in a):
            var, sep, val = [a.strip() for a in arg.partition(':=')]
            if not var.startswith('__'):
                mappings[var] = val
        logger.debug("loaded argv: %s", mappings)
        resolve_dict = self.resolve_dict.copy()
        resolve_dict['arg'] = mappings
        return attr.evolve(self, resolve_dict=resolve_dict)

    def with_env_arg(self, var: str, val: Any) -> 'LaunchContext':
        env_args = self.env_args + ((var, val),)
        return attr.evolve(self, env_args=env_args)


def tag(name: str, legal_attributes: Collection[str] = tuple()):
    legal_attributes = frozenset(legal_attributes)
    def wrap(loader):
        def wrapped(self,
                    ctx: LaunchContext,
                    elem: ET.Element
                    ) -> LaunchContext:
            logger.debug("parsing <%s> tag", name)
            for attribute in elem.attrib:
                if attribute not in legal_attributes:
                    raise FailedToParseLaunchFile(m)
            ctx = loader(self, ctx, elem)
            logger.debug("new context: %s", ctx)
            return ctx
        _TAG_TO_LOADER[name] = wrapped
        return wrapped
    return wrap


class LaunchFileReader:
    def __init__(self, shell: ShellProxy, files: FileProxy) -> None:
        self.__shell = shell
        self.__files = files

    def _parse_file(self, fn: str) -> ET.Element:
        """Parses a given XML launch file to a root XML element."""
        root = ET.fromstring(self.__files.read(fn))
        if root.tag != 'launch':
            m = 'root of launch file must have <launch></launch> tags'
            raise FailedToParseLaunchFile(m)
        return root

    def _load_tags(self,
                   ctx: LaunchContext,
                   tags: Sequence[ET.Element]
                   ) -> LaunchContext:
        for tag in (t for t in tags if t.tag in _TAG_TO_LOADER):
            loader = _TAG_TO_LOADER[tag.tag]
            ctx = loader(self, ctx, tag)
        return ctx

    @tag('arg', ['name', 'type', 'pkg'])
    def _load_node_tag(self,
                       ctx: LaunchContext,
                       tag: ET.Element
                       ) -> LaunchContext:
        name = tag.attrib['name']
        pkg = tag.attrib['type']
        node_type = tag.attrib['type']

        allowed = {'remap', 'rosparam', 'env', 'param'}
        self._load_tags([t for t in tags if t.tag in allowed])

        node = NodeConfig(name=name,
                          namespace=ctx.namespace,
                          pkg=pkg,
                          node_type=node_type)
        logger.debug("found node: %s", node)
        return ctx

    @tag('arg', ['name', 'default', 'value', 'doc'])
    def _load_arg_tag(self,
                      ctx: LaunchContext,
                      tag: ET.Element
                      ) -> LaunchContext:
        name = tag.attrib['name']
        logger.debug("found attribute: %s", name)
        return ctx

    @tag('env', ['name', 'value'])
    def _load_env_tag(self,
                      ctx: LaunchContext,
                      tag: ET.Element
                      ) -> LaunchContext:
        name = tag.attrib['name']
        value = tag.attrib['value']
        logger.debug("found env tag [%s]: %s", name, value)
        return ctx.with_env_arg(name, value)

    @tag('include', ['file', 'pass_all_args', 'ns', 'clear_params'])
    def _load_include_tag(self,
                          ctx: LaunchContext,
                          tag: ET.Element
                          ) -> LaunchContext:
        include_filename = resolve_args(self.__shell,
                                        self.__files,
                                        tag.attrib['file'])
        logger.debug("include file: %s", include_filename)

        # TODO should all arguments be passed?
        return ctx

    def read(self, fn: str, argv: Optional[Sequence[str]] = None) -> None:
        """Parses the contents of a given launch file.

        Reference
        ---------
            http://wiki.ros.org/roslaunch/XML/node
            http://docs.ros.org/kinetic/api/roslaunch/html/roslaunch.xmlloader.XmlLoader-class.html
        """
        ctx = LaunchContext(namespace='/',
                            filename=fn,
                            resolve_dict={},
                            include_resolve_dict={},
                            arg_names=[])
        if argv:
            ctx = ctx.with_argv(argv)

        launch = self._parse_file(fn)
        self._load_tags(ctx, list(launch))
