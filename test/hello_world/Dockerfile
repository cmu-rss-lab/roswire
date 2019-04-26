FROM ros:indigo

ENV ROS_WSPACE "/ros_ws"
WORKDIR "${ROS_WSPACE}"
RUN apt-get update \
 && apt-get install -y build-essential qtbase5-dev \
      vim \
      wget \
      curl \
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
    && /bin/bash -c "source /opt/ros/indigo/setup.bash && catkin_make"

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
