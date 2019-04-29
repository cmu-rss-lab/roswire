# -*- coding: utf-8 -*-
"""
This module provides functionality for converting the contents of ROS bag
files into Daikon trace (and declaration) files.
"""
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
    raise NotImplementedError
