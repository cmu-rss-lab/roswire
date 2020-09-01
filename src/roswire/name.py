# -*- coding: utf-8 -*-
__all__ = (
    'canonical_name',
    'global_name',
    'namespace',
    'namespaces_of',
    'namespace_join',
    'name_is_private',
    'name_is_global',
    'name_is_legal')

import re
from typing import List

RE_NAME = re.compile(r'^[\~\/A-Za-z][\w\/]*$')


def global_name(name: str) -> str:
    """Converts a name to its global form."""
    if name_is_private(name):
        m = f'cannot convert private name [{name}] into global name'
        raise ValueError(m)
    if not name_is_global(name):
        name = '/' + name
    if name[-1] != '/':
        name = name + '/'
    return name


def canonical_name(name: str) -> str:
    """Returns the canonical form of a given name."""
    if not name:
        return ''
    if name[0] == '/':
        return '/' + '/'.join(n for n in name[1:].split('/') if n)
    return '/'.join(n for n in name.split('/') if n)


def namespaces_of(name: str) -> List[str]:
    """Returns a list of each of the namespaces for a given name."""
    if not name:
        return ['/']
    parts = [n for n in name.split('/') if n]
    return ['/'] + ['/' + '/'.join(parts[:i]) for i in range(1, len(parts))]


def name_is_legal(name: str) -> bool:
    """Determines whether a given name is a legal ROS name."""
    if name in ('', '/'):
        return True
    return RE_NAME.match(name) is not None and '//' not in name


def name_is_private(name: str) -> bool:
    """Determines whether a given name is private."""
    return name.startswith('~')


def name_is_global(name: str) -> bool:
    """Determines whether a given name is global."""
    return name.startswith('/')


def namespace(name: str) -> str:
    """Returns the corresponding namespace for a given name."""
    if name_is_private(name):
        m = f'cannot determine namespace of private name: {name}'
        raise ValueError(m)
    name = global_name(name)[:-1]
    ns, sep, name = name.rpartition('/')
    return ns if ns else '/'


def namespace_join(ns: str, name: str) -> str:
    """Concatenates a given name to a namespace."""
    if not ns or name_is_private(name) or name_is_global(name):
        return name
    if ns == '~':
        return '~' + name
    return global_name(ns) + name
