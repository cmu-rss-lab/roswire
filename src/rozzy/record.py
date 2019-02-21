import os


class ROSBagRecorder(object):
    """
    Under the hood, this should use rosbag.
    Should save the bag files to a shared directory on the host machine.
    """
    # which topics should be recorded?
    fn_container_bag = attr.ib(type=str)
    fn_host_bag = attr.ib(type=str)
    excluded_topics = attr.ib(type=Tuple[str], default=tuple())

    # can we use rosbag?
    def resume() -> None:
        cmd = "rosbag record -q -a -O {}"
        cmd = cmd.format(self.fn_host_bag)

        # launch the process using a separate thread

    def pause() -> None:
        if self._process:
            # terminate process

        # if unresponsive, kill the process
