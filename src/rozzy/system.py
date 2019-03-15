__all__ = ['System']

import attr

from .definitions import TypeDatabase, FormatDatabase, PackageDatabase


@attr.s
class System:
    image: str = attr.ib()
    types: TypeDatabase = attr.ib()
    formats: FormatDatabase = attr.ib()
    packages: PackageDatabase = attr.ib()
