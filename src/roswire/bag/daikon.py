# -*- coding: utf-8 -*-
"""
This module provides functionality for converting the contents of ROS bag
files into Daikon trace (and declaration) files.
"""
from typing import Dict, Type, Set, FrozenSet, Tuple

import attr

from ..definitions import Message, MsgFormat
from ..description import SystemDescription


@attr.s(frozen=True, str=False)
class VarDecl:
    name: str = attr.ib()

    @property
    def lines(self) -> List[str]:
        return [f'variable {self.name}',
                '  var-kind variable',
                f'  dec-type {self.dec_type}',
                f'  rep-type {self.rep_type}']

    def __str__(self) -> str:
        return '\n'.join(self.lines)


@attr.s(frozen=True, str=False)
class GenericProgramPoint:
    name: str = attr.ib()
    variables: FrozenSet[VarDecl] = attr.ib(converter=frozenset)

    @property
    def lines(self) -> List[str]:
        ls = [f'ppt {self.name}:::POINT', 'ppt-type point']
        for var in self.variables:
            ls += var.lines
        return ls

    def __str__(self) -> str:
        return '\n'.join(self.lines)


@attr.s(frozen=True, str=False)
class Declarations:
    points: FrozenSet[GenericProgramPoint] = attr.ib(converter=frozenset)

    @property
    def lines(self) -> List[str]:
        ls = ['decl-version 2.0', 'var-comparability none']
        for ppt in self.points:
            ls += ppt.lines
        return ls

    def __str__(self) -> str:
        return '\n'.join(self.lines)


def topic_to_ppt(topic_name: str,
                 topic_fmt: MsgFormat
                 ) -> GenericProgramPoint:
    """Creates a program point for a given ROS topic."""
    decls: Set[VarDecl] = set()
    for field_ctx, field in topic_fmt.flatten(sys_desc.formats.messages):
        field_name = '.'.join(field_ctx) + field.name
        decl = VarDecl(field_name)
        decls.add(decl)
    return GenericProgramPoint(topic_name, decls)


def build_decls(fn_bag: str,
                fn_decls: str,
                sys_desc: SystemDescription
                ) -> None:
    """Builds a .decls file for a given ROS bag.

    Parameters
    ----------
    fn_bag: str
        the path to the bag file.
    fn_decls: str
        the path to the .decls file that should be created.
    sys_desc: SystemDescription
        a description of the system used to produce the bag.
    """
    # transform each topic to a program point
    ppts: Set[GenericProgramPoint] = set()
    topic_to_type: Dict[str, Type[Message]] = {}
    for topic_name, topic_type in topic_to_type.items():
        ppt = topic_to_ppt(topic_name, topic_type.format)
        ppts.add(ppt)
    raise NotImplementedError
