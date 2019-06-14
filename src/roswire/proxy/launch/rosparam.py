# -*- coding: utf-8 -*-
"""
This file provides utilities for interacting with rosparam.
"""
import math

import yaml

# allow both PyYAML and LibYAML
try:
    from yaml import CLoader as YAMLLoader
except ImportError:
    from yaml import YAMLLoader


@yaml.add_constructor('!radians')
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


@yaml.add_constructor('!degrees')
def load_degrees(loader, node) -> float:
    """Safely converts deg(num) to a float value.
    
    Note
    ----
    This does not support evaluation of expressions.
    """
    expr_s = loader.construct_scalar(node).strip()
    if expr_s.startswith('def('):
        expr_s = expr_s[4:-1]
    return float(expr_s) * math.pi / 180.0


yaml.add_implicit_resolve('!degrees', r'^deg\([^\)]*\)$', first='deg(')
yaml.add_implicit_resolve('!radians', r'^rad\([^\)]*\)$', first='rad(')
