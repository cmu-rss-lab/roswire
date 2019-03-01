__all__ = ['BagRecorderProxy']

from typing import Optional, Collection

from .shell import ShellProxy


class BagRecorderProxy:
    def __init__(self,
                 shell: ShellProxy,
                 excluded_topics: Optional[Collection[str]] = None
                 ) -> None:
        """
        Parameters:
            shell: a shell proxy.
            excluded_topics: an optional list of topics that should be excluded
                from the bag.
        """
        self.__shell: ShellProxy = shell

        # TODO create a temporary file inside the shared directory
        fn_container: str = "/tmp/"
        fn_host: str = "???"

        # launch rosbag process
        cmd: str = f"rosbag record -q -a -O {fn_host}"
        self.__process = self.__shell.non_blocking_execute(cmd)

    def __enter__(self) -> None:
        return self

    def __exit__(self) -> None:
        pass
