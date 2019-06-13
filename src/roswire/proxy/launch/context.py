# -*- coding: utf-8 -*-
"""
This file provides data structures that represent ROS launch configurations.
"""
__all__ = ('LaunchContext',)

from typing import Tuple, Mapping, Any, Optional, Sequence, Dict
from copy import deepcopy
import logging

import attr

from ...name import canonical_name, name_is_legal, namespace_join
from ...exceptions import FailedToParseLaunchFile

logger: logging.Logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)


@attr.s(frozen=True, slots=True)
class LaunchContext:
    filename: str = attr.ib()
    resolve_dict: Dict[str, Any] = attr.ib(factory=dict)
    parent: Optional['LaunchContext'] = attr.ib(default=None)
    namespace: str = attr.ib(default='/')
    arg_names: Tuple[str, ...] = attr.ib(default=tuple())
    env_args: Tuple[Tuple[str, str], ...] = attr.ib(default=tuple())
    pass_all_args: bool = attr.ib(default=False)
    include_resolve_dict: Optional[Dict[str, Any]] = attr.ib(default=None)
    remappings: Tuple[Tuple[str, str], ...] = attr.ib(default=tuple())
    node_name: Optional[str] = attr.ib(default=None)

    @property
    def is_node_context(self) -> bool:
        """Determines whether or not this context is node-local."""
        return self.node_name is not None

    def include_child(self,
                      ns: Optional[str],
                      filename: str
                      ) -> 'LaunchContext':
        ctx = self.child(ns)
        ctx = attr.evolve(ctx,
                          filename=filename,
                          arg_names=tuple(),
                          include_resolve_dict={})
        return ctx

    def node_child(self, ns: Optional[str], name: str) -> 'LaunchContext':
        ctx = self.child(ns).child(name)
        ctx = attr.evolve(ctx,
                          node_name=name,
                          include_resolve_dict={})
        return ctx

    def child(self, ns: Optional[str] = None) -> 'LaunchContext':
        """Creates a child context that inherits from this context."""
        if ns is None:
            child_ns = self.namespace
        elif ns.startswith('/') or ns == '~':
            child_ns = ns
        else:
            child_ns = namespace_join(self.namespace, ns)
        return attr.evolve(self,
                           parent=self,
                           namespace=child_ns,
                           pass_all_args=False)

    def with_remapping(self, frm: str, to: str) -> 'LaunchContext':
        """Adds a name remapping."""
        frm = canonical_name(frm)
        to = canonical_name(to)
        if not frm or not to:
            m = "'from' and 'to' attributes must be specified for <remap>"
            raise FailedToParseLaunchFile(m)
        if not name_is_legal(frm):
            m = f"<remap>: invalid ROS name [from]: {frm}"
            raise FailedToParseLaunchFile(m)
        if not name_is_legal(to):
            m = f"<remap>: invalid ROS name [to]: {to}"
            raise FailedToParseLaunchFile(m)

        # overwrite any existing remapping from the given source before adding
        # the given remapping
        remappings = tuple(r for r in self.remappings if r[0] != frm)
        remappings = remappings + ((frm, to), )
        return attr.evolve(self, remappings=remappings)

    def with_pass_all_args(self) -> 'LaunchContext':
        ctx = self
        if self.parent and 'arg' in self.parent.resolve_dict:
            for var, val in self.parent.resolve_dict['arg'].items():
                ctx = ctx.with_arg(var, value=val)
        return attr.evolve(ctx, pass_all_args=True)

    def process_include_args(self) -> 'LaunchContext':
        if self.include_resolve_dict is None:
            return attr.evolve(self, arg_names=tuple())

        arg_dict = self.include_resolve_dict.get('arg', {})
        for arg in self.arg_names:
            if arg not in arg_dict:
                m = f'include arg [{arg}] is missing value.'
                raise FailedToParseLaunchFile(m)

        return attr.evolve(self,
                           arg_names=tuple(),
                           resolve_dict=deepcopy(self.include_resolve_dict),
                           include_resolve_dict=None)

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

    def with_arg(self,
                 name: str,
                 default: Optional[Any] = None,
                 value: Optional[Any] = None,
                 doc: Optional[str] = None
                 ) -> 'LaunchContext':
        logger.debug("adding arg [%s] to context", name)
        arg_names = self.arg_names
        if name in self.arg_names:
            if not self.pass_all_args:
                m = f"arg [{name}] has already been declared"
                raise FailedToParseLaunchFile(m)
        else:
            arg_names = arg_names + (name,)

        # decide which resolve dictionary should be used
        use_include_resolve_dict = self.include_resolve_dict is not None
        if use_include_resolve_dict:
            assert self.include_resolve_dict is not None  # stupid mypy
            resolve_dict = self.include_resolve_dict.copy()
        else:
            resolve_dict = self.resolve_dict.copy()

        arg_dict = resolve_dict['arg'] = resolve_dict.get('arg', {}).copy()

        if value is not None:
            if name in arg_dict and not self.pass_all_args:
                m = f"arg [{name}] value has already been defined."
                raise FailedToParseLaunchFile(m)
            arg_dict[name] = value
        elif default is not None:
            arg_dict[name] = arg_dict.get(name, default)

        # construct new context
        ctx = attr.evolve(self, arg_names=arg_names)
        if use_include_resolve_dict:
            return attr.evolve(ctx, include_resolve_dict=resolve_dict)
        else:
            return attr.evolve(ctx, resolve_dict=resolve_dict)
