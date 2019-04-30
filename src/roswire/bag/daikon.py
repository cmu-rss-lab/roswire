# -*- coding: utf-8 -*-
"""
This module provides functionality for converting the contents of ROS bag
files into Daikon trace (and declaration) files.

Right now, this module does not handle arrays, timestamps, and durations.

References
----------
https://plse.cs.washington.edu/daikon/download/doc/developer/File-formats.html#Data-trace-records
"""
__all__ = ('bag_to_decls', 'bag_to_daikon')

from typing import (Dict, Type, Set, FrozenSet, List, Union, Iterator,
                    Collection, Mapping)
from functools import reduce

import attr

from .reader import BagReader
from ..description import SystemDescription
from ..definitions import Message, MsgFormat


TYPE_TO_DAIKON = {
    'bool': 'boolean',
    'int8': 'byte',
    'uint8': 'byte',
    'int16': 'short',
    'uint16': 'short',
    'int32': 'int',
    'uint32': 'int',
    'int64': 'long',
    'uint64': 'long',
    'float32': 'double',
    'float64': 'double',
    'string': 'java.lang.String'}


@attr.s(frozen=True, str=False)
class VarDecl:
    name: str = attr.ib()
    dec_type: str = attr.ib()
    rep_type: str = attr.ib()

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
        ls = [f'ppt {self.fullname}', 'ppt-type point']
        for var in self.variables:
            ls += var.lines
        return ls

    @property
    def fullname(self) -> str:
        return f'{self.name}:::POINT'

    def __str__(self) -> str:
        return '\n'.join(self.lines)


class Declarations(Mapping[str, GenericProgramPoint]):
    def __init__(self, points: Collection[GenericProgramPoint]) -> None:
        self.__points = {p.name: p for p in points}

    def __len__(self) -> int:
        return len(self.__points)

    def __getitem__(self, name: str) -> GenericProgramPoint:
        return self.__points[name]

    def __iter__(self) -> Iterator[str]:
        yield from self.__points

    @property
    def lines(self) -> List[str]:
        ls = ['decl-version 2.0', 'var-comparability none']
        for ppt in self.values():
            ls += ppt.lines
        return ls

    def __str__(self) -> str:
        return '\n'.join(self.lines)

    def save(self, filename: str) -> None:
        with open(filename, 'w') as f:
            f.write(str(self))


class TraceWriter:
    """Used to write to Daikon trace files."""
    def __init__(self, filename: str) -> None:
        self.__filename = filename
        self.__fp = open(filename, 'w')

    def add(self,
            ppt: GenericProgramPoint,
            vals: Dict[str, Union[bool, float, str, str]]
            ) -> None:
        """Writes an entry to the trace file."""
        fp = self.__fp
        lines: List[str] = [ppt.fullname]
        for var_name, var_val in vals.items():
            if isinstance(var_val, bool):
                str_val = 'true' if var_val else 'false'
            elif isinstance(var_val, str):
                str_val = var_val.replace('\\', '\\\\')
                str_val = str_val.replace('"', '\\"')
                str_val = f'"{str_val}"'
            else:
                str_val = str(var_val)
            lines += [var_name, str_val, '1']
        self.__fp.writelines(f'{l}\n' for l in lines)
        self.__fp.write('\n')

    def close(self) -> None:
        self.__fp.close()


def topic_to_ppt(topic_name: str,
                 topic_fmt: MsgFormat,
                 sys_desc: SystemDescription
                 ) -> GenericProgramPoint:
    """Creates a program point for a given ROS topic."""
    decls: Set[VarDecl] = set()
    for field_ctx, field in topic_fmt.flatten(sys_desc.formats.messages):
        if field.typ not in TYPE_TO_DAIKON:
            continue

        field_name = field.name
        if field_ctx:
            field_name = f"{'.'.join(field_ctx)}.{field_name}"

        field_type = TYPE_TO_DAIKON[field.typ]
        dec_type = field_type
        rep_type = field_type
        decl = VarDecl(field_name, dec_type, rep_type)
        decls.add(decl)
    return GenericProgramPoint(topic_name, decls)  # type: ignore


def bag_to_decls(fn_bag: str, sys_desc: SystemDescription) -> Declarations:
    """Builds a .decls file for a given ROS bag.

    Parameters
    ----------
    fn_bag: str
        the path to the bag file.
    sys_desc: SystemDescription
        a description of the system used to produce the bag.

    Returns
    -------
    Declarations:
        a declarations description.
    """
    # determine the set of topics (and their types) represented in the bag
    reader = BagReader(fn_bag, sys_desc.types)
    topic_to_type = reader.topics_to_types

    # transform each topic to a program point
    ppts: Set[GenericProgramPoint] = set()
    for topic_name, topic_type in topic_to_type.items():
        ppt = topic_to_ppt(topic_name, topic_type.format, sys_desc)
        ppts.add(ppt)

    return Declarations(ppts)  # type: ignore


def bag_to_daikon(fn_bag: str,
                  fn_dtrace: str,
                  fn_decls: str,
                  sys_desc: SystemDescription
                  ) -> None:
    """Transforms a ROS bag into a Daikon trace.

    Parameters
    ----------
    fn_bag: str
        the path to the bag file.
    fn_dtrace: str
        the path to the output Daikon trace file.
    fn_decls: str
        the path to the output Daikon decls file.
    sys_desc: SystemDescription
        a description of the system used to produce the bag.
    """
    decls = bag_to_decls(fn_bag, sys_desc)
    decls.save(fn_decls)

    def read_val(m: Message, name_field: str) -> str:
        return reduce(getattr, name_field.split('.'), m)

    trace_writer = TraceWriter(fn_dtrace)
    bag_reader = BagReader(fn_bag, sys_desc.types)
    for message in bag_reader:
        ppt = decls[message.topic]
        vals = {}
        for var_decl in ppt.variables:
            vals[var_decl.name] = read_val(message.message, var_decl.name)
        trace_writer.add(ppt, vals)
