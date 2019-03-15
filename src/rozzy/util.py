__all__ = ('Stopwatch',)

from timeit import default_timer as timer
import warnings


class Stopwatch:
    def __init__(self) -> None:
        """
        Constructs a new, paused timer.
        """
        self.__offset: float = 0.0
        self.__paused: bool = True
        self.__time_start: float = 0.0

    def stop(self) -> None:
        """
        Freezes the timer.
        """
        if not self.__paused:
            self.__offset += timer() - self.__time_start
            self.__paused = True

    def start(self) -> None:
        """
        Resumes the timer.
        """
        if self.__paused:
            self.__time_start = timer()
            self.__paused = False
        else:
            warnings.warn("timer is already running")

    def reset(self) -> None:
        """
        Resets and freezes the timer.
        """
        self.__offset = 0.0
        self.__paused = True

    @property
    def paused(self) -> bool:
        """
        Returns True if this stopwatch is paused, or False if not.
        """
        return self.__paused

    @property
    def duration(self) -> float:
        """
        The number of seconds that the stopwatch has been running.
        """
        d = self.__offset
        if not self.__paused:
            d += timer() - self.__time_start
        return d
