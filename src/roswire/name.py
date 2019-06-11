# -*- coding: utf-8 -*-
__all__ = (
    'name_is_private',
    'name_is_global')


def name_is_private(name: str) -> bool:
    """Determines whether a given name is private."""
    return name.startswith('~')


def name_is_global(name: str) -> bool:
    """Determines whether a given name is global."""
    return name.startswith('/')
