class ROSBagProxy(object):
    # which topics should be recorded?
    fn_container_bag = attr.ib(type=str)
    fn_host_bag = attr.ib(type=str)
    excluded_topics = attr.ib(type=Tuple[str], default=tuple())

    # can we use rosbag?
    def resume() -> None:
        cmd = "rosbag record -q -a -O {}"
        cmd = cmd.format(self.fn_host_bag)
        self._process = self._proxy.nonblocking(cmd)

    def pause() -> None:
        if self._process:
            self._process.terminate(kill_after=10)
