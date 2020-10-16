__all__ = ('ROS1LaunchFileReader',)

import os
import shlex
import subprocess
import typing
import xml.etree.ElementTree as ET
from typing import (Any, Callable, Collection, Optional,
                    overload, Sequence, Tuple, Union)

import attr
import dockerblade
from loguru import logger

from ...exceptions import FailedToParseLaunchFile
from ...name import global_name, name_is_global, name_is_private, namespace, namespace_join
from ...proxy.roslaunch.config import ExecutableType, LaunchConfig, NodeConfig
from ...proxy.roslaunch.context import LaunchContext
from ...proxy.roslaunch.reader import LaunchFileReader
from ...proxy.roslaunch.rosparam import load_from_yaml_string as load_rosparam_from_string
from ...proxy.roslaunch.substitution import ArgumentResolver

if typing.TYPE_CHECKING:
    from roswire import AppInstance

_TAG_TO_LOADER = {}

Loader = \
    Callable[['ROS1LaunchFileReader', LaunchContext, LaunchConfig, ET.Element],
             Tuple[LaunchContext, LaunchConfig]]


def _read_contents(tag: ET.Element) -> str:
    """Reads the text contents of an XML element."""
    # FIXME add support for CDATA -- possibly via lxml or xml.dom?
    return ''.join(t.text for t in tag if t.text)


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


def tag(name: str,
        legal_attributes: Collection[str] = tuple()
        ) -> Callable[[Loader], Loader]:
    legal_attributes = frozenset(list(legal_attributes) + ['if', 'unless'])

    def wrap(loader: Loader) -> Loader:
        def wrapped(self: 'ROS1LaunchFileReader',
                    ctx: LaunchContext,
                    cfg: LaunchConfig,
                    elem: ET.Element
                    ) -> Tuple[LaunchContext, LaunchConfig]:
            logger.debug(f"parsing <{name}> tag")
            for attribute in elem.attrib:
                if attribute not in legal_attributes:
                    m = f'<{name}> tag contains illegal attribute: {attribute}'
                    raise FailedToParseLaunchFile(m)

            # should we process this element?
            if not self._ifunless_check(elem, ctx):
                return ctx, cfg
            return loader(self, ctx, cfg, elem)

        _TAG_TO_LOADER[name] = wrapped
        return wrapped

    return wrap


@attr.s(auto_attribs=True)
class ROS1LaunchFileReader(LaunchFileReader):
    _shell: dockerblade.Shell
    _files: dockerblade.FileSystem

    @classmethod
    def for_app_instance(cls, app_instance: 'AppInstance') -> LaunchFileReader:
        return ROS1LaunchFileReader(shell=app_instance.shell,
                                    files=app_instance.files)

    def _parse_file(self, fn: str) -> ET.Element:
        """Parses a given XML launch file to a root XML element."""
        contents = self._files.read(fn)
        logger.info(f'parsing launch file [{fn}]:\n{contents}')
        root = ET.fromstring(contents)
        if root.tag != 'launch':
            m = 'root of launch file must have <launch></launch> tags'
            raise FailedToParseLaunchFile(m)
        return root

    def _load_tags(self,
                   ctx: LaunchContext,
                   cfg: LaunchConfig,
                   tags: Sequence[ET.Element]
                   ) -> Tuple[LaunchContext, LaunchConfig]:
        for tag in (t for t in tags if t.tag in _TAG_TO_LOADER):
            loader = _TAG_TO_LOADER[tag.tag]
            ctx, cfg = loader(self, ctx, cfg, tag)
        return ctx, cfg

    @tag('group', ['ns'])
    def _load_group_tag(self,
                        ctx: LaunchContext,
                        cfg: LaunchConfig,
                        tag: ET.Element
                        ) -> Tuple[LaunchContext, LaunchConfig]:
        # create context for group
        ns = self._read_namespace(ctx, tag)
        ctx_child = ctx.child(ns)

        # handle nested tags
        nested_tags = [t for t in tag]
        ctx_child, cfg = self._load_tags(ctx_child, cfg, nested_tags)

        return ctx, cfg

    @tag('param', ['name', 'value', 'type', 'textfile', 'binfile', 'command'])
    def _load_param_tag(self,
                        ctx: LaunchContext,
                        cfg: LaunchConfig,
                        tag: ET.Element
                        ) -> Tuple[LaunchContext, LaunchConfig]:
        name = self._read_required(tag, 'name', ctx)
        typ = self._read_optional(tag, 'type', ctx) or 'auto'
        logger.debug(f"adding parameter [{name}] with type [{typ}]")

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
            value = self._files.read(textfile)
        if binfile is not None:
            value = self._files.read(binfile, binary=True)
        if command is not None:
            logger.debug(f'obtaining value for parameter [{name}] '
                         f'via command:\n{command}')
            value = self._shell.check_output(command, text=True, stderr=False)

        logger.debug(f"obtained value for parameter [{name}]: {value}")

        # compute the fully qualified name
        if name_is_global(name):
            fullname = name
        elif name_is_private(name):
            fullname = namespace_join(ctx.namespace, name[1:])
        else:
            fullname = namespace_join(ctx.namespace, name)

        # register the parameter
        cfg = cfg.with_param(name=fullname,
                             typ=typ,
                             value=value,
                             command=command)

        return ctx, cfg

    @tag('rosparam', ['command', 'ns', 'file', 'param', 'subst_value'])
    def _load_rosparam_tag(self,
                           ctx: LaunchContext,
                           cfg: LaunchConfig,
                           tag: ET.Element
                           ) -> Tuple[LaunchContext, LaunchConfig]:
        filename = self._read_optional(tag, 'file', ctx)
        subst_value = self._read_optional_bool(tag, 'subst_value', ctx, False)
        ns = self._read_optional(tag, 'ns', ctx) or ''
        param = self._read_optional(tag, 'param', ctx) or ''
        param = namespace_join(ns, param)
        full_param = namespace_join(ctx.namespace, param)
        value = _read_contents(tag)

        cmd: str = self._read_optional(tag, 'command', ctx) or 'load'
        if cmd not in ('load', 'delete', 'dump'):
            m = f"<rosparam> unsupported 'command': {cmd}"
            raise FailedToParseLaunchFile(m)

        if cmd == 'load' and filename is not None:
            if not self._files.isfile(filename):
                m = f"<rosparam> file does not exist: {filename}"
                raise FailedToParseLaunchFile(m)

        if cmd == 'delete' and filename is not None:
            m = "<rosparam> command:delete does not support filename"
            raise FailedToParseLaunchFile(m)

        # handle load command
        if cmd == 'load':
            if filename is None:
                yml_text = value
            else:
                yml_text = self._files.read(filename)

            if subst_value:
                yml_text = self._resolve_args(yml_text, ctx)
            logger.debug(f"parsing rosparam YAML:\n{yml_text}")
            data = load_rosparam_from_string(yml_text)
            logger.debug(f"rosparam values: {data}")
            if not isinstance(data, dict) and not param:
                m = "<rosparam> requires 'param' for non-dictionary values"
                raise FailedToParseLaunchFile(m)
            cfg = cfg.with_rosparam(name=full_param, value=data)

        # handle dump command
        if cmd == 'dump':
            m = "'dump' command is currently not supported in <rosparam>"
            raise NotImplementedError(m)

        # handle delete command
        if cmd == 'delete':
            m = "'delete' command is currently not supported in <rosparam>"
            raise NotImplementedError(m)

        return ctx, cfg

    @tag('remap', ['from', 'to'])
    def _load_remap_tag(self,
                        ctx: LaunchContext,
                        cfg: LaunchConfig,
                        tag: ET.Element
                        ) -> Tuple[LaunchContext, LaunchConfig]:
        frm = self._read_required(tag, 'from', ctx)
        to = self._read_required(tag, 'to', ctx)
        ctx = ctx.with_remapping(frm, to)
        return ctx, cfg

    @tag('node', ['name', 'type', 'pkg', 'required', 'clear_params',
                  'respawn', 'ns', 'output', 'args', 'ns', 'launch-prefix'])
    def _load_node_tag(self,
                       ctx: LaunchContext,
                       cfg: LaunchConfig,
                       tag: ET.Element
                       ) -> Tuple[LaunchContext, LaunchConfig]:
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
            self._read_optional_float(tag, 'respawn_delay', ctx, 0.0)

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

        # locate node executable and determine the type
        executable_path = self.locate_node_binary(package, node_type)
        executable_type = self._get_executable_type(executable_path)

        node = NodeConfig(name=name,
                          namespace=namespace(ctx_child.namespace),
                          package=package,
                          executable_path=executable_path,
                          executable_type=executable_type,
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

    def _get_executable_type(self, path: str) -> ExecutableType:
        try:
            first_line = self._files.read(path).partition('\n')[0]
            if 'python' in first_line:
                return ExecutableType.PYTHON
            else:
                return ExecutableType.LIKELY_CPP
        except UnicodeDecodeError:
            return ExecutableType.LIKELY_CPP

    @tag('arg', ['name', 'default', 'value', 'doc'])
    def _load_arg_tag(self,
                      ctx: LaunchContext,
                      cfg: LaunchConfig,
                      tag: ET.Element
                      ) -> Tuple[LaunchContext, LaunchConfig]:
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
                      cfg: LaunchConfig,
                      tag: ET.Element
                      ) -> Tuple[LaunchContext, LaunchConfig]:
        name = self._read_required(tag, 'name', ctx)
        value = self._read_required(tag, 'value', ctx)
        cfg = cfg.with_env(name, value)
        ctx = ctx.with_env_arg(name, value)
        return ctx, cfg

    @tag('include', ['file', 'pass_all_args', 'ns', 'clear_params'])
    def _load_include_tag(self,
                          ctx: LaunchContext,
                          cfg: LaunchConfig,
                          tag: ET.Element
                          ) -> Tuple[LaunchContext, LaunchConfig]:
        include_filename = self._read_required(tag, 'file', ctx)
        logger.debug(f"include file: {include_filename}")
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
        logger.debug(f"prepared include context: {ctx_child}")

        logger.debug("loading include file")
        launch = self._parse_file(include_filename)
        ctx_child, cfg = self._load_tags(ctx_child, cfg, list(launch))

        return ctx, cfg

    def _read_namespace(self,
                        ctx: LaunchContext,
                        tag: ET.Element
                        ) -> Optional[str]:
        """Fetches the value of the optional 'ns' attribute."""
        ns = self._read_optional(tag, 'ns', ctx)
        if ns == '':
            m = f"<{tag.tag}> has empty attribute [ns]"
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

    def _read_optional_bool(self,
                            elem: ET.Element,
                            attrib: str,
                            ctx: LaunchContext,
                            default: Optional[bool] = None
                            ) -> Optional[bool]:
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

    def _read_optional_float(self,
                             elem: ET.Element,
                             attrib: str,
                             ctx: LaunchContext,
                             default: Optional[float] = None
                             ) -> Optional[float]:
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

    def _ifunless_check(self, elem: ET.Element, ctx: LaunchContext) -> bool:
        """Determines whether an element should be parsed."""
        if_val = self._read_optional(elem, 'if', ctx)
        unless_val = self._read_optional(elem, 'unless', ctx)
        if if_val is not None and unless_val is not None:
            m = 'tag may not provide both "if" and "unless" attributes'
            raise FailedToParseLaunchFile(m)
        if if_val is not None:
            return _parse_bool('if', if_val)
        if unless_val is not None:
            return not _parse_bool('unless', unless_val)
        return True

    def _resolve_args(self, s: str, ctx: LaunchContext) -> str:
        """Resolves all substitution args in a given string."""
        logger.debug(f"resolve [{s}] with context: {ctx.resolve_dict}")
        resolve_ctx = ctx.resolve_dict or {}
        resolver = ArgumentResolver(shell=self._shell,
                                    files=self._files,
                                    context=resolve_ctx)
        return resolver.resolve(s)

    def read(self,
             fn: str,
             argv: Optional[Sequence[str]] = None
             ) -> LaunchConfig:
        """Parses the contents of a given launch file.

        Returns
        -------
        LaunchConfig
            A description of the launch configuration.

        Reference
        ---------
        * http://wiki.ros.org/roslaunch/XML/node
        * http://docs.ros.org/kinetic/api/roslaunch/html/roslaunch.xmlloader.XmlLoader-class.html
        """
        cfg = LaunchConfig()
        ctx = LaunchContext(namespace='/', filename=fn)
        if argv:
            ctx = ctx.with_argv(argv)

        launch = self._parse_file(fn)
        ctx, cfg = self._load_tags(ctx, cfg, list(launch))
        logger.debug(f"launch configuration: {cfg}")
        return cfg

    def locate_node_binary(self,
                           package: str,
                           node_type: str) -> str:
        """Attempts to locate the binary for a given node.

        Returns
        -------
        str
            The absolute path of the binary for that node.

        Raises
        ------
        ValueError
            If the given package could not be found.
        ValueError
            If no binary can be located for the given node.
        """
        path: Optional[str] = None
        logger.debug(f'locating binary for node_type [{node_type}] '
                     f'in package [{package}]')
        shell = self._shell
        files = self._files

        # start by looking in libexec
        command = ('catkin_find --first-only --libexec '
                   f'{shlex.quote(package)} {shlex.quote(node_type)}')
        try:
            path = shell.check_output(command, stderr=False, text=True)
        except subprocess.CalledProcessError:
            pass

        if not path:
            # look in the scripts directory of the package's source directory
            command = ('rospack find '
                       f'{shlex.quote(package)}')
            try:
                package_dir = shell.check_output(command, stderr=False, text=True)
            except subprocess.CalledProcessError:
                raise ValueError(f"package not found: {package}")

            path_in_scripts_dir = os.path.join(package_dir, 'scripts', node_type)
            path_in_nodes_dir = os.path.join(package_dir, 'nodes', node_type)
            if files.isfile(path_in_scripts_dir) and \
                    files.access(path_in_scripts_dir, os.X_OK):
                path = path_in_scripts_dir
            elif files.isfile(path_in_nodes_dir) and \
                    files.access(path_in_nodes_dir, os.X_OK):
                path = path_in_nodes_dir

        if not path:
            m = (f"unable to locate binary for node_type [{node_type}] "
                 f"in package [{package}]")
            raise ValueError(m)

        logger.debug(f'located binary for node_type [{node_type}] '
                     f'in package [{package}]: {path}')
        return path
