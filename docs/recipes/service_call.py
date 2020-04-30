import time
import logging
import roswire

logging.basicConfig()

FN_SITL = '/ros_ws/src/ArduPilot/build/sitl/bin/arducopter'
FN_PARAMS = '/ros_ws/src/ArduPilot/copter.parm'

rsw = roswire.ROSWire()
sources = ['/opt/ros/indigo/setup.bash', '/ros_ws/devel/setup.bash']
with rsw.launch('roswire/example:mavros', sources) as system:
    # Fetch the dynamically generated types for the messages that we want to send
    SetModeRequest = system.messages['mavros_msgs/SetModeRequest']
    CommandBoolRequest = system.messages['mavros_msgs/CommandBoolRequest']
    CommandTOLRequest = system.messages['mavros_msgs/CommandTOLRequest']

    # launch a temporary ROS session inside the app container
    # once the context is closed, the ROS session will be terminated and all
    # of its associated nodes will be automatically killed.
    with system.roscore() as ros:
        # for this example, we need to separately launch a software-in-the-loop
        # simulator for the robot platform
        ps_sitl = system.shell.popen(f'{FN_SITL} --model copter --defaults {FN_PARAMS}')

        # use roslaunch to launch the application inside the ROS session
        ros.roslaunch('apm.launch', package='mavros', args={'fcu_url': 'tcp://127.0.0.1:5760@5760'})

        # let's wait some time for the copter to become armable
        time.sleep(60)

        # arm the copter
        request_arm = CommandBoolRequest(value=True)
        response_arm = ros.services['/mavros/cmd/arming'].call(request_arm)
        assert response_arm.success

        # switch to guided mode
        request_guided = SetModeRequest(base_mode=0, custom_mode='GUIDED')
        response_guided = ros.services['/mavros/set_mode'].call(request_guided)
        assert response_arm.success

        # takeoff to 50 metres above the ground
        request_takeoff = CommandTOLRequest(min_pitch=0.0,
                                            yaw=0.0,
                                            latitude=0.0,
                                            longitude=0.0,
                                            altitude=50.0)
        response_takeoff = ros.services['/mavros/cmd/takeoff'].call(request_takeoff)
        assert response_takeoff.success

        # wait for the copter to reach the target altitude
        print("waiting for copter to reach altitude...")
        time.sleep(30)
        print("finished waiting")

        # kill the simulator
        ps_sitl.kill()
