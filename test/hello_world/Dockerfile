FROM ros:indigo-ros-base
RUN apt-get update \
 && apt-get install -y build-essential qtbase5-dev \
      ros-indigo-ros-tutorials \
      ros-indigo-common-tutorials \
      vim \
      wget \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists

RUN mkdir /root/overlay_ws \
    && cd /root/overlay_ws \
    && wstool init src \
    && cd /root/overlay_ws/src \
    && wstool set ros_tutorials --git git://github.com/ros/ros_tutorials.git -y \
    && wstool update 

RUN cd /root/overlay_ws \
    && /bin/bash -c "source /opt/ros/indigo/setup.bash && catkin_make"

# create entrypoint
ENV ROS_WSPACE "/root/overlay_ws"
WORKDIR "${ROS_WSPACE}"
RUN echo "#!/bin/bash \n\
set -e \n\
source \"/opt/ros/indigo/setup.bash\" \n\
source \"${ROS_WSPACE}/devel/setup.bash\" \n\
exec \"\$@\"" > "${ROS_WSPACE}/entrypoint.sh" \
 && chmod +x "${ROS_WSPACE}/entrypoint.sh"
ENTRYPOINT ["/root/overlay_ws/entrypoint.sh"]
CMD ["/bin/bash"]
