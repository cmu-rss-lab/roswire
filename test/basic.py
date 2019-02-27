import logging
import time

from rozzy import Rozzy


def main():
    logging.basicConfig()
    rozzy = Rozzy()
    with rozzy.launch() as container:
        print(f"IP address: {container.ip_address}")
        with container.roscore() as ros:
            print(f"ROS Master URI: {ros.uri}")
            time.sleep(10)
            print(ros.topic_to_type)
            # print(ros.lookup_node('rosout'))

            """
            ros.parameters['/chris'] = 'hello!'
            print(list(ros.parameters))
            print(f"/rosversion: {ros.parameters['/rosversion']}")
            print(f"/chris: {ros.parameters['/chris']}")
            print(f"contains /chris? {'yes' if '/chris' in ros.parameters else 'no'}")
            print(f"contains /pikachu? {'yes' if '/pikachu' in ros.parameters else 'no'}")
            del ros.parameters['/rosversion']
            print(list(ros.parameters))
            """

            # TODO connect to simulator: allow visualisation

            # launch SITL on 5760
            # sim_vehicle.py -C -v ArduCopter --daemon --no-rebuild
            cmd = ' '.join([
                "/ros_ws/src/ArduPilot/build/sitl/bin/arducopter",
                "--model copter"
            ])
            container.shell.non_blocking_execute(cmd)

            # launch mavros
            cmd = ' '.join([
                "roslaunch",
                "mavros apm.launch",
                "fcu_url:=tcp://127.0.0.1:5760@5760"
            ])
            container.shell.non_blocking_execute(cmd)

            time.sleep(30)

            # launch_mavros = ros.launch("mavros apm.launch",
            #                            {'fcu_url': 'tcp://127.0.0.1:5760:5760'})


if __name__ == '__main__':
    main()
