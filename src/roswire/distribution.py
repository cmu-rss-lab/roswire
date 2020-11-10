# -*- coding: utf-8 -*-
__all__ = ("ROSDistribution", "ROSVersion")

import enum
import functools
from typing import Any, Sequence


class ROSVersion(enum.IntEnum):
    """Describes a ROS major version, either ROS 1 or ROS 2."""

    ROS1 = 1
    ROS2 = 2

    @property
    def distributions(self) -> Sequence["ROSDistribution"]:
        """
        Return a list of all distributions for this version of ROS.

        Th list is ordered alphabetically, and therefore, also
        ordered by release date.
        """
        return ROSDistribution.for_version(self)


@functools.total_ordering
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

    FOXY = ("foxy", "ROS2")
    ELOQUENT = ("eloquent", "ROS2")
    DASHING = ("dashing", "ROS2")
    CRYSTAL = ("crystal", "ROS2")
    BOUNCY = ("bouncy", "ROS2")
    ARDENT = ("ardent", "ROS2")

    NOETIC = ("noetic", "ROS1")
    MELODIC = ("melodic", "ROS1")
    LUNAR = ("lunar", "ROS1")
    KINETIC = ("kinetic", "ROS1")
    JADE = ("jade", "ROS1")
    INDIGO = ("indigo", "ROS1")
    HYDRO = ("hydro", "ROS1")
    GROOVY = ("groovy", "ROS1")
    FUERTE = ("fuerte", "ROS1")
    ELECTRIC = ("electric", "ROS1")
    DIAMONDBACK = ("diamondback", "ROS1")

    def __init__(self, display_name: str, ros: str) -> None:
        self.display_name = display_name
        self.ros = ROSVersion[ros]

    def __ne__(self, other: Any) -> bool:
        return not (self == other)

    def __lt__(self, other: Any) -> int:
        if not isinstance(other, ROSDistribution):
            m = f"can only compare ROSDistribution objects"
            raise ValueError(m)
        if self.ros != other.ros:
            m = f"can only compare ROSDistributions for same version"
            raise ValueError(m)
        return self.display_name < other.display_name

    @classmethod
    def with_name(cls, name: str) -> "ROSDistribution":
        """
        Retrieves the ROS distribution with a given name.

        Raises
        ------
        ValueError
            If no distribution is found with the given name.
        """
        name_uppercase = name.upper()
        try:
            return cls[name_uppercase]
        except KeyError:
            raise ValueError(f"ROS distribution not found: {name}")

    @classmethod
    def for_version(cls, version: ROSVersion) -> Sequence["ROSDistribution"]:
        """
        Returns a list of all distributions for a given version of ROS.

        The list is ordered alphabetically, and therefore, also ordered
        by release date.
        """
        return sorted(
            (d for d in cls if d.ros == version), key=lambda d: d.name
        )
