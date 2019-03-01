FROM ros:indigo

# install core utilities
WORKDIR /ros_ws
RUN apt-get update \
 && apt-get install -y --no-install-recommends \
      apt-utils \
      sudo \
      xvfb \
      vim \
      wget \
      curl \
      python-catkin-tools \
      python-rosinstall-generator \
      python-pip \
 && pip install future \
 && apt-get clean \
 && rm -rf /var/lib/apt/lists/*

# install ROS dependencies
RUN rosinstall_generator mavros mavros_extras --deps --rosdistro "${ROS_DISTRO}" > pkgs.rosinstall
RUN wstool init -j8 src pkgs.rosinstall
RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
 && apt-get update \
 && rosdep update \
 && rosdep install -i -y -r --from-paths src \
                        --ignore-src \
                        --skip-keys="python-rosdep python-catkin-pkg python-rospkg" \
                        --rosdistro="${ROS_DISTRO}" \
 && apt-get clean \
 && rm -rf /var/lib/apt/lists/*

# RUN /ros_ws/src/mavros/mavros/scripts/install_geographiclib_datasets.sh

# build ArduPilot 3.6.4
RUN git clone https://github.com/ArduPilot/ArduPilot src/ArduPilot \
 && cd src/ArduPilot \
 && git checkout Copter-3.6.4 \
 && ./waf configure \
 && ./waf configure \
 && ./waf build

# build mavros and dependencies
RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
 && catkin build

# create entrypoint
ENV ROS_WSPACE /ros_ws
WORKDIR "${ROS_WSPACE}"
RUN echo "#!/bin/bash \n\
set -e \n\
source \"/opt/ros/\${ROS_DISTRO}/setup.bash\" \n\
source \"${ROS_WSPACE}/devel/setup.bash\" \n\
exec \"\$@\"" > "${ROS_WSPACE}/entrypoint.sh" \
 && chmod +x "${ROS_WSPACE}/entrypoint.sh"
ENTRYPOINT ["/ros_ws/entrypoint.sh"]
CMD ["/bin/bash"]
