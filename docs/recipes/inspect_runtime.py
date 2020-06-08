import time

from loguru import logger

import roswire

# enable logging
logger.enable('roswire')

# first, we create a new ROSWire session
rsw = roswire.ROSWire()

# we then launch a container for a given robot.
# once the end of the given context is reached, including the event where an
# exception occurs, the container will be automatically destroyed.
image = 'therobotcooperative/turtlebot3'
sources = ['/opt/ros/kinetic/setup.bash', '/ros_ws/devel/setup.bash']
environment = {'TURTLEBOT3_MODEL': 'burger'}
with rsw.launch(image, sources, environment=environment) as system:

    # we then launch ROS inside the container
    # again, once the end of the context is reached, ROS is killed.
    with system.roscore() as ros:

        # let's bring up the application
        ros.roslaunch('turtlebot3_house.launch',
                      package='turtlebot3_gazebo',
                      args={'gui': 'false'})

        # we need to wait for the nodes to finish their initialisation
        time.sleep(30)

        # get an overview of the system state
        state = ros.state
        print(f"System state: {state}")

        # get topic types
        topic_to_type = ros.topic_to_type
        print(f"Topic to type: {topic_to_type}")

        # get service formats
        service_to_format = {}
        for service_name in state.services:
            service = ros.services[service_name]
            service_to_format[service_name] = service.format.name
        print(f"Service to format: {service_to_format}")
