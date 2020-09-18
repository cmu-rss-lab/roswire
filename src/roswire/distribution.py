# -*- coding: utf-8 -*-
__all__ = ('ROSDistribution', 'ROSVersion')

import enum
from typing import Sequence


class ROSVersion(enum.IntEnum):
    ROS1 = 1
    ROS2 = 2

    @property
    def distributions(self) -> Sequence['ROSDistribution']:
        """Returns a list of all distributions for this version of ROS,
        ordered alphabetically, and therefore, also ordered by release date."""
        return ROSDistribution.for_version(self)


@enum.unique
class ROSDistribution(enum.Enum):
    """Describes a ROS distribution.

    Attributes
    ----------
    ros: ROSVersion
        The ROS version associated with this distribution.
    name: str
        The name of this distribution.
    """
    FOXY = ('foxy', 'ROS2')
    ELOQUENT = ('eloquent', 'ROS2')
    DASHING = ('dashing', 'ROS2')
    CRYSTAL = ('crystal', 'ROS2')
    BOUNCY = ('bouncy', 'ROS2')
    ARDENT = ('ardent', 'ROS2')

    NOETIC = ('noetic', 'ROS1')
    MELODIC = ('melodic', 'ROS1')
    LUNAR = ('lunar', 'ROS1')
    KINETIC = ('kinetic', 'ROS1')
    JADE = ('jade', 'ROS1')
    INDIGO = ('indigo', 'ROS1')
    HYDRO = ('hydro', 'ROS1')
    GROOVY = ('groovy', 'ROS1')
    FUERTE = ('fuerte', 'ROS1')
    ELECTRIC = ('electric', 'ROS1')
    DIAMONDBACK = ('diamondback', 'ROS1')

    def __init__(self, display_name: str, ros: str) -> None:
        self.display_name = display_name
        self.ros = ROSVersion[ros]

    @classmethod
    def with_name(cls, name: str) -> 'ROSDistribution':
        """Retrieves the ROS distribution with a given name.

        Raises
        ------
        ValueError
            If no distribution is found with the given name.
        """
        name_uppercase = name.upper()
        if name_uppercase not in cls:
            raise ValueError(f"ROS distribution not found: {name}")
        return cls[name_uppercase]

    @classmethod
    def for_version(cls, version: ROSVersion) -> Sequence['ROSDistribution']:
        """Returns a list of all distributions for a given version of ROS,
        ordered alphabetically, and therefore, also ordered by release date."""
        return sorted([d for d in cls if d.ros == version],
                      key=lambda d: d.name)
