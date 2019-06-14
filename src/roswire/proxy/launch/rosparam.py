# -*- coding: utf-8 -*-
"""
This file provides utilities for interacting with rosparam.
"""


def load_radians(loader, node) -> float:
    """Safely converts rad(num) to a float value.
    
    Note
    ----
    This does not support evaluation of expressions.
    """
    expr_s = loader.construct_scalar(node).strip()
    if expr_s.startswith('rad('):
        expr_s = expr_s[4:-1]

    # TODO safely parse and evaluate expression
    return float(expr_s)
