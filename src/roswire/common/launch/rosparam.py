# -*- coding: utf-8 -*-
"""This file provides utilities for interacting with rosparam."""
__all__ = ("load_from_yaml_string",)

import math
import re
from typing import Any, Dict

import yaml


class YAMLLoader(yaml.SafeLoader):
    """A custom YAML loader for rosparam files."""


def load_from_yaml_string(s: str) -> Dict[str, Any]:
    """Parses the contents of a rosparam file to a dictionary."""
    return yaml.load(s, Loader=YAMLLoader) or {}


def __load_radians(loader: YAMLLoader, node: yaml.YAMLObject) -> float:
    """Safely converts rad(num) to a float value.

    Note
    ----
    This does not support evaluation of expressions.
    """
    expr_s = loader.construct_scalar(node).strip()
    if expr_s.startswith("rad("):
        expr_s = expr_s[4:-1]

    # TODO safely parse and evaluate expression
    return float(expr_s)


def __load_degrees(loader: YAMLLoader, node: yaml.YAMLObject) -> float:
    """Safely converts deg(num) to a float value."""
    expr_s = loader.construct_scalar(node).strip()
    if expr_s.startswith("def("):
        expr_s = expr_s[4:-1]
    return float(expr_s) * math.pi / 180.0


YAMLLoader.add_constructor("!degrees", __load_degrees)
YAMLLoader.add_implicit_resolver(
    "!degrees", re.compile("^deg\([^\)]*\)$"), first="deg("
)
YAMLLoader.add_constructor("!radians", __load_radians)
YAMLLoader.add_implicit_resolver(
    "!radians", re.compile("^rad\([^\)]*\)$"), first="rad("
)
