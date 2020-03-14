import time

import roswire

FN_SITL = '/ros_ws/src/ArduPilot/build/sitl/bin/arducopter'
FN_PARAMS = '/ros_ws/src/ArduPilot/copter.parm'

rsw = roswire.ROSWire()

sources = ['/opt/ros/indigo/setup.bash', '/ros_ws/devel/setup.bash']
with rsw.launch('roswire/example:mavros', sources) as system:
    with system.roscore() as ros:
        # for this example, we need to separately launch a software-in-the-loop
        # simulator for the robot platform
        ps_sitl = system.shell.popen(f'{FN_SITL} --model copter --defaults {FN_PARAMS}')

        # use roslaunch to launch the application inside the ROS session
        ros.launch('apm.launch', package='mavros', args={'fcu_url': 'tcp://127.0.0.1:5760@5760'})

        # to record all ROS topic data for 300 seconds
        with ros.record('filepath-on-host-machine.bag') as recorder:
            time.sleep(300)

        # alternatively, we can use a recorder directly
        recorder = ros.record('filepath-on-host-machine2.bag')
        recorder.start()
        time.sleep(300)
        recorder.stop()
