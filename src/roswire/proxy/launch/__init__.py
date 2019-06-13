# -*- coding: utf-8 -*-
"""
This file implements a proxy for parsing the contents of launch files.
"""
__all__ = ('LaunchFileReader',)

from typing import (List, Optional, Sequence, Collection, Dict, Any, Mapping,
                    Tuple, Union, overload)
from copy import deepcopy
import logging
import xml.etree.ElementTree as ET

import attr

from .config import ROSConfig, NodeConfig, Parameter
from .context import LaunchContext
from ..substitution import resolve as resolve_args
from ..shell import ShellProxy
from ..file import FileProxy
from ...name import (namespace_join, global_name, namespace, name_is_global,
                     name_is_private)
from ...exceptions import FailedToParseLaunchFile

logger: logging.Logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)

_TAG_TO_LOADER = {}


def _parse_bool(attr: str, val: str) -> bool:
    """Parses a boolean value from an XML attribute."""
    val = val.lower()
    if val == 'true':
        return True
    if val == 'false':
        return False

    m = f'illegal boolean attribute [{attr}]: {val}'
    raise FailedToParseLaunchFile(m)


def _parse_float(attr: str, val: str) -> float:
    """Parses a float value from an XML attribute."""
    if not val:
        m = f'empty string used by float attribute [{attr}]'
        raise FailedToParseLaunchFile(m)
    try:
        return float(val)
    except ValueError:
        m = f'failed to parse attribute [{attr}] to float: {val}'
        raise FailedToParseLaunchFile(m)


def convert_str_to_type(s: str, typ: str) -> Union[bool, int, str, float]:
    if typ == 'auto':
        if s.lower() in ('true', 'false'):
            return convert_str_to_type(s, 'bool')
        if s.isnumeric() and '.' in s:
            return convert_str_to_type(s, 'float')
        if s.isnumeric():
            return convert_str_to_type(s, 'int')
        return convert_str_to_type(s, 'str')

    if typ in ('str', 'string'):
        return s
    if typ in ('float', 'double'):
        return float(s)
    if typ == 'int':
        return int(s)
    if typ in ('bool', 'boolean'):
        s = s.lower()
        if s in ('true', '1'):
            return True
        if s in ('false', '0'):
            return False
        raise ValueError(f"illegal 'bool' value: {s}")

    raise ValueError(f'unknown parameter type: {typ}')


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
                   ) -> Tuple[LaunchContext, ROSConfig]:
        for tag in (t for t in tags if t.tag in _TAG_TO_LOADER):
            loader = _TAG_TO_LOADER[tag.tag]
            ctx, cfg = loader(self, ctx, cfg, tag)
        return ctx, cfg

    @tag('param', ['name', 'value', 'type', 'textfile', 'binfile', 'command'])
    def _load_param_tag(self,
                        ctx: LaunchContext,
                        cfg: ROSConfig,
                        tag: ET.Element
                        ) -> Tuple[LaunchContext, ROSConfig]:
        name = self._read_required(tag, 'name', ctx)
        typ = self._read_optional(tag, 'type', ctx) or 'auto'
        logger.debug("adding parameter [%s] with type [%s]", name, typ)

        # obtain value from either 'value', 'textfile', 'binfile', or
        # 'command' exclusively.
        val = self._read_optional(tag, 'value', ctx)
        textfile = self._read_optional(tag, 'textfile', ctx)
        binfile = self._read_optional(tag, 'binfile', ctx)
        command = self._read_optional(tag, 'command', ctx)
        value_candidates = [val, textfile, binfile, command]

        if len([c for c in value_candidates if c is not None]) != 1:
            m = '<param> must have exactly one value/textfile/binfile/command'
            raise FailedToParseLaunchFile(m)

        value: Any
        if val is not None:
            value = convert_str_to_type(val, typ)
        if textfile is not None:
            value = self.__files.read(textfile)
        if binfile is not None:
            value = self.__files.read(binfile, binary=True)
        if command is not None:
            retcode, value, duration = self.__shell.execute(command)

        logger.debug("obtained value for parameter [%s]: %s", name, value)

        # compute the fully qualified name
        if name_is_global(name):
            fullname = name
        elif name_is_private(name):
            fullname = namespace_join(ctx.namespace, name[1:])
        else:
            fullname = namespace_join(ctx.namespace, name)

        # register the parameter
        cfg = cfg.with_param(fullname, value)

        return ctx, cfg

    @tag('remap', ['from', 'to'])
    def _load_remap_tag(self,
                        ctx: LaunchContext,
                        cfg: ROSConfig,
                        tag: ET.Element
                        ) -> Tuple[LaunchContext, ROSConfig]:
        frm = self._read_required(tag, 'from', ctx)
        to = self._read_required(tag, 'to', ctx)
        ctx = ctx.with_remapping(frm, to)
        return ctx, cfg

    @tag('node', ['name', 'type', 'pkg', 'required', 'clear_params',
                  'respawn', 'namespace', 'output'])
    def _load_node_tag(self,
                       ctx: LaunchContext,
                       cfg: ROSConfig,
                       tag: ET.Element
                       ) -> Tuple[LaunchContext, ROSConfig]:
        name = self._read_required(tag, 'name', ctx)
        package = self._read_required(tag, 'pkg', ctx)
        node_type = self._read_required(tag, 'type', ctx)
        output = self._read_optional(tag, 'output', ctx)
        launch_prefix = self._read_optional(tag, 'launch-prefix', ctx)
        cwd = self._read_optional(tag, 'cwd', ctx)
        args = self._read_optional(tag, 'args', ctx)
        required = self._read_optional_bool(tag, 'required', ctx, False)
        respawn = self._read_optional_bool(tag, 'respawn', ctx, False)
        respawn_delay = \
            self._read_optional_float(tag, 'respawn', ctx, 0.0)

        # create context
        ns = self._read_namespace(ctx, tag)
        ctx_child = ctx.node_child(ns, name)

        # handle 'clear_params'
        if self._read_optional_bool(tag, 'clear_params', ctx, False):
            clear_ns = global_name(ctx_child.namespace)
            cfg = cfg.with_clear_param(clear_ns)

        # handle nested tags
        allowed = {'env', 'remap', 'param', 'rosparam'}
        nested_tags = [t for t in tag if t.tag in allowed]
        ctx_child, cfg = self._load_tags(ctx_child, cfg, nested_tags)

        node = NodeConfig(name=name,
                          namespace=namespace(ctx_child.namespace),
                          package=package,
                          cwd=cwd,
                          args=args,
                          required=required,
                          respawn=respawn,
                          respawn_delay=respawn_delay,
                          output=output,
                          remappings=ctx_child.remappings,
                          launch_prefix=launch_prefix,
                          filename=ctx.filename,
                          env_args=ctx_child.env_args,
                          typ=node_type)
        cfg = cfg.with_node(node)
        return ctx, cfg

    @tag('arg', ['name', 'default', 'value', 'doc'])
    def _load_arg_tag(self,
                      ctx: LaunchContext,
                      cfg: ROSConfig,
                      tag: ET.Element
                      ) -> Tuple[LaunchContext, ROSConfig]:
        name = self._read_required(tag, 'name', ctx)
        value = self._read_optional(tag, 'value', ctx)
        default = self._read_optional(tag, 'default', ctx)
        doc = self._read_optional(tag, 'doc', ctx)
        ctx = ctx.with_arg(name=name,
                           value=value,
                           default=default,
                           doc=doc)
        return ctx, cfg

    @tag('env', ['name', 'value'])
    def _load_env_tag(self,
                      ctx: LaunchContext,
                      cfg: ROSConfig,
                      tag: ET.Element
                      ) -> Tuple[LaunchContext, ROSConfig]:
        name = self._read_required(tag, 'name', ctx)
        value = self._read_required(tag, 'value', ctx)
        ctx = ctx.with_env_arg(name, value)
        return ctx, cfg

    @tag('include', ['file', 'pass_all_args', 'ns', 'clear_params'])
    def _load_include_tag(self,
                          ctx: LaunchContext,
                          cfg: ROSConfig,
                          tag: ET.Element
                          ) -> Tuple[LaunchContext, ROSConfig]:
        include_filename = self._read_required(tag, 'file', ctx)
        logger.debug("include file: %s", include_filename)
        cfg = cfg.with_roslaunch_file(include_filename)

        # create context
        ns = self._read_namespace(ctx, tag)
        ctx_child = ctx.include_child(ns, include_filename)

        # handle 'clear_params'
        if self._read_optional_bool(tag, 'clear_params', ctx, False):
            if not ns:
                m = "'ns' must be specified to use 'clear_params'"
                raise FailedToParseLaunchFile(m)
            cfg = cfg.with_clear_param(ctx_child.namespace)

        # if instructed to pass along args, then those args must be added to
        # the child context
        if self._read_optional_bool(tag, 'pass_all_args', ctx, False):
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

    def _read_namespace(self,
                        ctx: LaunchContext,
                        tag: ET.Element
                        ) -> Optional[str]:
        """Fetches the value of the optional 'namespace' attribute."""
        ns = self._read_optional(tag, 'namespace', ctx)
        if ns == '':
            m = f"<{tag.tag}> has empty attribute [namespace]"
            raise FailedToParseLaunchFile(m)
        return ns

    @overload
    def _read_optional_bool(self,
                            elem: ET.Element,
                            attrib: str,
                            ctx: LaunchContext,
                            default: None
                            ) -> Optional[bool]: ...

    @overload
    def _read_optional_bool(self,
                            elem: ET.Element,
                            attrib: str,
                            ctx: LaunchContext,
                            default: bool
                            ) -> bool: ...

    def _read_optional_bool(self, elem, attrib, ctx, default=None):
        s = self._read_optional(elem, attrib, ctx)
        if s is None:
            return default
        return _parse_bool(attrib, s)

    @overload
    def _read_optional_float(self,
                             elem: ET.Element,
                             attrib: str,
                             ctx: LaunchContext,
                             default: None
                             ) -> Optional[float]: ...

    @overload
    def _read_optional_float(self,
                             elem: ET.Element,
                             attrib: str,
                             ctx: LaunchContext,
                             default: float
                             ) -> float: ...

    def _read_optional_float(self, elem, attrib, ctx, default=None):
        s = self._read_optional(elem, attrib, ctx)
        if s is None:
            return default
        return _parse_float(attrib, s)

    def _read_optional(self,
                       elem: ET.Element,
                       attrib: str,
                       ctx: LaunchContext
                       ) -> Optional[str]:
        """Reads the string value of an optional attribute of an element."""
        if attrib not in elem.attrib:
            return None
        return self._read_required(elem, attrib, ctx)

    def _read_required(self,
                       elem: ET.Element,
                       attrib: str,
                       ctx: LaunchContext
                       ) -> str:
        """Reads the string value of a required attribute of an element."""
        return self._resolve_args(elem.attrib[attrib], ctx)

    def _resolve_args(self, s: str, ctx: LaunchContext) -> str:
        """Resolves all substitution args in a given string."""
        logger.debug("resolve [%s] with context: %s", s, ctx.resolve_dict)
        return resolve_args(self.__shell, self.__files, s, ctx.resolve_dict)

    def read(self, fn: str, argv: Optional[Sequence[str]] = None) -> ROSConfig:
        """Parses the contents of a given launch file.

        Returns
        -------
        ROSConfig
            A description of the launch configuration.

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
        return cfg
