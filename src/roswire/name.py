# -*- coding: utf-8 -*-
__all__ = (
    'global_name',
    'namespace',
    'name_is_private',
    'name_is_global')


def global_name(name: str) -> str:
    """Converts a name to its canonical global form."""
    if name_is_private(name):
        m = f'cannot convert private name [{name}] into global name'
        raise ValueError(m)
    if not name_is_global(name):
        name = '/' + name
    if name[-1] != '/':
        name = name + '/'
    return name


def name_is_private(name: str) -> bool:
    """Determines whether a given name is private."""
    return name.startswith('~')


def name_is_global(name: str) -> bool:
    """Determines whether a given name is global."""
    return name.startswith('/')


def namespace(name: str) -> str:
    """Returns the corresponding namespace for a given name."""
    if name_is_private(name):
        return name
    name = global_name(name)[:-1]
    ns, sep, name = name.rpartition('/')
    return ns if ns else '/'
