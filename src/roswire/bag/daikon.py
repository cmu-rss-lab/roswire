# -*- coding: utf-8 -*-
"""
This module provides functionality for converting the contents of ROS bag
files into Daikon trace (and declaration) files.
"""
from typing import Dict, Type

from ..definitions import Message, MsgFormat
from ..description import SystemDescription


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
    # create a program point for each topic
    topic_to_type: Dict[str, Type[Message]] = {}
    for topic_name, topic_type in topic_to_type.items():
        topic_fmt: MsgFormat = topic_type.format
        for field_ctx, field in topic_fmt.flatten(sys_desc.formats.messages):
            field_name = '.'.join(field_ctx) + field.name

    raise NotImplementedError
