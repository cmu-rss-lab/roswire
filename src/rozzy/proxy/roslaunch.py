__all__ = ['ROSLaunchProxy']

from .shell import ShellProxy


# TODO
# - valgrind
# - callgrind
# - Kcachegrind
# - gprof
# - gcov
class ROSLaunchProxy:
    def __init__(self, shell: ShellProxy) -> None:
        self.__shell = shell

    def launch(self, fn_launch_container: str) -> None:
        cmd = "roslaunch '{}'"
        cmd = cmd.format(fn_launch_container)
        # should pass back an object that allows launch process to be killed
        # - need to know PID?
        pass
