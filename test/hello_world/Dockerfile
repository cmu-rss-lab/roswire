FROM ros:kinetic as base
ENV ROS_WSPACE /ros_ws
WORKDIR "${ROS_WSPACE}"
RUN apt-get update \
 && apt-get install -y build-essential qtbase5-dev \
      vim \
      wget \
      curl \
      clang-4.0 \
      python-catkin-tools \
      python-rosinstall-generator \
      python-pip \
 && apt-get clean \
 && rm -rf /var/lib/apt/lists
RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
    && cd ${ROS_WSPACE} \
    && wstool init src \
    && cd ${ROS_WSPACE}/src \
    && wstool set ros_tutorials --git git://github.com/ros/ros_tutorials.git -y \
    && wstool update \
    && cd ${ROS_WSPACE} \
    && /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && catkin build"

# create entrypoint
WORKDIR "${ROS_WSPACE}"
RUN echo "#!/bin/bash \n\
set -e \n\
source \"/opt/ros/${ROS_DISTRO}/setup.bash\" \n\
source \"${ROS_WSPACE}/devel/setup.bash\" \n\
exec \"\$@\"" > "${ROS_WSPACE}/entrypoint.sh" \
 && chmod +x "${ROS_WSPACE}/entrypoint.sh"
ENTRYPOINT ["/ros_ws/entrypoint.sh"]
CMD ["/bin/bash"]

# create launch files
RUN echo "<launch><node name='listener' pkg='roscpp_tutorials' type='listener' output='screen'/></launch>" \
      > "${ROS_WSPACE}/src/ros_tutorials/roscpp_tutorials/launch/listener.launch" \
 && echo "<launch><node name='listener' pkg='roscpp_tutorials' type='talker' output='screen'/></launch>" \
      > "${ROS_WSPACE}/src/ros_tutorials/roscpp_tutorials/launch/talker.launch"

FROM base as buggy
COPY bug.diff ${ROS_WSPACE}
COPY bug.bag ${ROS_WSPACE}
COPY non-bug.bag ${ROS_WSPACE}
RUN cd ${ROS_WSPACE}/src/ros_tutorials \
 && git apply ${ROS_WSPACE}/bug.diff \
 && cd ${ROS_WSPACE} \
 && /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && catkin build"
