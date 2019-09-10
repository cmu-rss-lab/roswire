import time
import logging
import roswire

logging.basicConfig()

FN_SITL = '/ros_ws/src/ArduPilot/build/sitl/bin/arducopter'
FN_PARAMS = '/ros_ws/src/ArduPilot/copter.parm'

rsw = roswire.ROSWire()
with rsw.launch('roswire/example:mavros') as system:

    # launch a temporary ROS session inside the app container
    # once the context is closed, the ROS session will be terminated and all
    # of its associated nodes will be automatically killed.
    with system.roscore() as ros:
        # for this example, we need to separately launch a software-in-the-loop
        # simulator for the robot platform
        ps_sitl = system.shell.popen(f'{FN_SITL} --model copter --defaults {FN_PARAMS}')

        # use roslaunch to launch the application inside the ROS session
        ros.launch('apm.launch', 'mavros', args={'fcu_url': 'tcp://127.0.0.1:5760@5760'})
        time.sleep(5)

        # let's wait some time for the copter to become armable
        time.sleep(60)

        # disable prearm checking
        # ros.services['/mavros/param/set'].call(request_prearm)

        # arm the copter
        request_arm = system.messages['mavros_msgs/CommandBoolRequest'](value=True)
        response_arm = ros.services['/mavros/cmd/arming'].call(request_arm)
        print(response_arm)

        # switch to guided mode
        request_guided = system.messages['mavros_msgs/SetModeRequest'](base_mode=216, custom_mode='')
        ros.services['/mavros/set_mode'].call(request_guided)

        # takeoff to 50 metres above the ground
        request_takeoff = system.messages['mavros_msgs/CommandTOLRequest'](min_pitch=0.0,
                                                                           yaw=0.0,
                                                                           latitude=0.0,
                                                                           longitude=0.0,
                                                                           altitude=50.0)
        response_takeoff = ros.services['/mavros/cmd/takeoff'].call(request_takeoff)
        print(response_takeoff)

        ps_sitl.kill()
