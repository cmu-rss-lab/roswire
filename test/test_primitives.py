import pytest

from roswire.definitions.base import Time, Duration


def test_duration_between():
    sec_to_nsec = 1000000000
    t_start = Time(150, 300)
    t_stop = Time(150, 350)
    assert Duration.between(t_start, t_stop) == Duration(0, 50)
    assert Duration.between(t_start, t_start) == Duration(0, 0)
    with pytest.raises(AssertionError):
        Duration.between(t_stop, t_start)

    t_start = Time(150, 300)
    t_stop = Time(200, 300)
    assert Duration.between(t_start, t_stop) == Duration(50, 0)

    t_start = Time(150, 300)
    t_stop = Time(200, 50)
    assert Duration.between(t_start, t_stop) == Duration(49, sec_to_nsec - 250)

    t_start = Time(150, 50)
    t_stop = Time(200, 300)
    assert Duration.between(t_start, t_stop) == Duration(50, 250)


def test_time_compare():
    t = Time
    assert t(100, 250) < t(100, 300)
    assert t(100, 250) <= t(100, 300)
