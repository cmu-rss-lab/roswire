# -*- coding: utf-8 -*-
"""
This file implements a proxy for parsing the contents of launch files.
"""
__all__ = ('LaunchFileReader',)

from typing import (List, Optional, Sequence, Collection, Dict, Any, Mapping,
                    Tuple)
from copy import deepcopy
import logging
import xml.etree.ElementTree as ET

import attr

from .config import ROSConfig, NodeConfig
from .context import LaunchContext
from ..substitution import resolve as resolve_args
from ..shell import ShellProxy
from ..file import FileProxy
from ...name import namespace_join
from ...exceptions import FailedToParseLaunchFile

logger: logging.Logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)

_TAG_TO_LOADER = {}


def _parse_bool(attr: str, val: Optional[str], default: bool) -> bool:
    """Parses a boolean value from an XML attribute."""
    if val is None:
        return default

    val = val.lower()
    if val == 'true':
        return True
    if val == 'false':
        return False

    m = f'illegal boolean attribute [{attr}]: {val}'
    raise FailedToParseLaunchFile(m)


def _parse_float(attr: str, val: Optional[str], default: float) -> float:
    """Parses a float value from an XML attribute."""
    if val is None:
        return default
    if not val:
        m = f'empty string used by float attribute [{attr}]'
        raise FailedToParseLaunchFile(m)
    try:
        return float(val)
    except ValueError:
        m = f'failed to parse attribute [{attr}] to float: {val}'
        raise FailedToParseLaunchFile(m)


def tag(name: str, legal_attributes: Collection[str] = tuple()):
    legal_attributes = frozenset(legal_attributes)
    def wrap(loader):
        def wrapped(self,
                    ctx: LaunchContext,
                    cfg: ROSConfig,
                    elem: ET.Element
                    ) -> Tuple[LaunchContext, ROSConfig]:
            logger.debug("parsing <%s> tag", name)
            for attribute in elem.attrib:
                if attribute not in legal_attributes:
                    m = '<{}> tag contains illegal attribute: {}'
                    raise FailedToParseLaunchFile(m.format(name, attribute))
            ctx, cfg = loader(self, ctx, cfg, elem)
            return ctx, cfg
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
                   cfg: ROSConfig,
                   tags: Sequence[ET.Element]
                   ) -> LaunchContext:
        for tag in (t for t in tags if t.tag in _TAG_TO_LOADER):
            loader = _TAG_TO_LOADER[tag.tag]
            ctx, cfg = loader(self, ctx, cfg, tag)
        return ctx, cfg

    @tag('node', ['name', 'type', 'pkg', 'required', 'clear_params',
                  'respawn', 'namespace', 'output'])
    def _load_node_tag(self,
                       ctx: LaunchContext,
                       cfg: ROSConfig,
                       tag: ET.Element
                       ) -> Tuple[LaunchContext, ROSConfig]:
        name = tag.attrib['name']
        package = tag.attrib['pkg']
        node_type = tag.attrib['type']
        output = tag.attrib.get('output')
        if output:
            output = self._resolve_args(output, ctx)
        required = _parse_bool('required', tag.attrib.get('required'), False)
        respawn = _parse_bool('respawn', tag.attrib.get('respawn'), False)

        # TODO respawn_delay

        allowed = {'remap', 'rosparam', 'env', 'param'}
        # self._load_tags([t for t in tags if t.tag in allowed])

        # TODO determine remappings
        remappings = tuple()

        node = NodeConfig(name=name,
                          namespace=ctx.namespace,
                          package=package,
                          required=required,
                          respawn=respawn,
                          output=output,
                          remappings=remappings,
                          filename=ctx.filename,
                          typ=node_type)
        cfg = cfg.with_node(node)
        return ctx, cfg

    @tag('arg', ['name', 'default', 'value', 'doc'])
    def _load_arg_tag(self,
                      ctx: LaunchContext,
                      cfg: ROSConfig,
                      tag: ET.Element
                      ) -> Tuple[LaunchContext, ROSConfig]:
        value: Optional[str] = tag.attrib.get('value')
        default: Optional[str] = tag.attrib.get('default')
        if value:
            value = self._resolve_args(value, ctx)
        if default:
            default = self._resolve_args(default, ctx)
        ctx = ctx.with_arg(name=tag.attrib['name'],
                           value=value,
                           default=default,
                           doc=tag.attrib.get('doc'))
        return ctx, cfg

    @tag('env', ['name', 'value'])
    def _load_env_tag(self,
                      ctx: LaunchContext,
                      cfg: ROSConfig,
                      tag: ET.Element
                      ) -> Tuple[LaunchContext, ROSConfig]:
        ctx = ctx.with_env_arg(tag.attrib['name'], tag.attrib['value'])
        return ctx, cfg

    @tag('include', ['file', 'pass_all_args', 'ns', 'clear_params'])
    def _load_include_tag(self,
                          ctx: LaunchContext,
                          cfg: ROSConfig,
                          tag: ET.Element
                          ) -> Tuple[LaunchContext, ROSConfig]:
        include_filename = self._resolve_args(tag.attrib['file'], ctx)
        logger.debug("include file: %s", include_filename)
        cfg = cfg.with_roslaunch_file(include_filename)

        # construct child context
        ctx_child = self._handle_ns_and_clear_params(ctx,
                                                     tag,
                                                     include_filename=include_filename)

        # if instructed to pass along args, then those args must be added to
        # the child context
        if 'pass_all_args' in tag.attrib:
            s_pass_all_args = tag.attrib['pass_all_args'].value
            s_pass_all_args = self._resolve_args(s_pass_all_args, ctx)
            if _parse_bool('pass_all_args', s_pass_all_args, False):
                ctx_child = ctx_child.with_pass_all_args()

        # handle child tags
        child_tags = [t for t in tag if t.tag in ('env', 'arg')]
        ctx_child, cfg = self._load_tags(ctx_child, cfg, child_tags)
        ctx_child = ctx_child.process_include_args()
        logger.debug("prepared include context: %s", ctx_child)

        logger.debug("loading include file")
        launch = self._parse_file(include_filename)
        ctx_child, cfg = self._load_tags(ctx_child, cfg, list(launch))

        return ctx, cfg

    def _handle_ns_and_clear_params(self,
                                    ctx: LaunchContext,
                                    tag: ET.Element,
                                    include_filename: Optional[str] = None
                                    ) -> LaunchContext:
        ns: Optional[str] = None
        if 'namespace' in tag.attrib:
            ns = tag.attrib['namespace']
            ns = self._resolve_args(ns, ctx)
            if not ns:
                m = f"<{tag.tag}> has empty attribute [namespace]"
                raise FailedToParseLaunchFile(m)

        if include_filename:
            ctx_child = ctx.include_child(ns, include_filename)
        else:
            ctx_child = ctx.child(ns)

        # TODO clear params

        return ctx_child

    def _read_optional(self,
                       elem: ET.Element,
                       attrib: str,
                       ctx: LaunchContext
                       ) -> Optional[str]:
        """Reads the string value of an optional attribute of a DOM element."""
        if attrib not in elem.attrib:
            return None
        return self._resolve_args(elem.attrib[val])

    def _resolve_args(self, s: str, ctx: LaunchContext) -> str:
        """Resolves all substitution args in a given string."""
        logger.debug("resolve [%s] with context: %s", s, ctx.resolve_dict)
        return resolve_args(self.__shell, self.__files, s, ctx.resolve_dict)

    def read(self, fn: str, argv: Optional[Sequence[str]] = None) -> None:
        """Parses the contents of a given launch file.

        Reference
        ---------
            http://wiki.ros.org/roslaunch/XML/node
            http://docs.ros.org/kinetic/api/roslaunch/html/roslaunch.xmlloader.XmlLoader-class.html
        """
        cfg = ROSConfig()
        ctx = LaunchContext(namespace='/', filename=fn)
        if argv:
            ctx = ctx.with_argv(argv)

        launch = self._parse_file(fn)
        ctx, cfg = self._load_tags(ctx, cfg, list(launch))
        logger.debug("launch configuration: %s", cfg)
