__all__ = ['System']

import attr


@attr.s
class System:
    image: str = attr.ib()
