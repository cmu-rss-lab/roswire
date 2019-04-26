FROM hello-world:latest

COPY bug.diff ${ROS_WSPACE}
COPY bug.bag ${ROS_WSPACE}
RUN cd ${ROS_WSPACE}/src/ros_tutorials/ && \
    git apply ${ROS_WSPACE}/bug.diff && \
    cd ${ROS_WSPACE} && \
    /bin/bash -c "source /opt/ros/indigo/setup.bash && catkin_make"
