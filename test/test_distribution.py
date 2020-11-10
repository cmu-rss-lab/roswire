# -*- coding: utf-8 -*-
import pytest

from roswire.distribution import ROSDistribution


def test_ordering():
    assert ROSDistribution.FOXY != ROSDistribution.ELOQUENT
    assert ROSDistribution.NOETIC != ROSDistribution.FOXY

    # compare ROS2 versions
    assert ROSDistribution.FOXY > ROSDistribution.ELOQUENT
    assert ROSDistribution.FOXY >= ROSDistribution.ELOQUENT
    assert ROSDistribution.ELOQUENT < ROSDistribution.FOXY
    assert ROSDistribution.ELOQUENT <= ROSDistribution.FOXY

    # compare ROS1 versions
    assert ROSDistribution.NOETIC > ROSDistribution.KINETIC
    assert ROSDistribution.NOETIC >= ROSDistribution.KINETIC
    assert ROSDistribution.KINETIC < ROSDistribution.NOETIC
    assert ROSDistribution.KINETIC <= ROSDistribution.NOETIC

    # compare ROS1 vs. ROS2
    with pytest.raises(ValueError):
        ROSDistribution.FOXY < ROSDistribution.KINETIC

    # sorting
    distributions = [
        ROSDistribution.KINETIC,
        ROSDistribution.INDIGO,
        ROSDistribution.JADE,
        ROSDistribution.MELODIC,
    ]

    expected_ascending = ["INDIGO", "JADE", "KINETIC", "MELODIC"]
    expected_descending = list(reversed(expected_ascending))
    assert [d.name for d in sorted(distributions)] == expected_ascending
    assert [
        d.name for d in sorted(distributions, reverse=True)
    ] == expected_descending
