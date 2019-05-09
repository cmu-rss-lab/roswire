FROM ros:kinetic as base
ENV ROS_WSPACE /ros_ws
WORKDIR "${ROS_WSPACE}"
RUN apt-get update \
 && apt-get install -y \
      build-essential \
      software-properties-common \
      qtbase5-dev \
      vim \
      wget \
      curl \
      python-catkin-tools \
      python-rosinstall-generator \
      python-pip \
 && wget -O - https://apt.llvm.org/llvm-snapshot.gpg.key | apt-key add - \
 && apt-add-repository "deb http://apt.llvm.org/xenial/ llvm-toolchain-xenial-6.0 main" \
 && apt-get update \
 && apt-get install -y clang-6.0 clang-tools-6.0 \
 && apt-get clean \
 && rm -rf /var/lib/apt/lists \
 && update-alternatives --install /usr/bin/clang++ clang++ /usr/bin/clang++-6.0 100 \
 && update-alternatives --install /usr/bin/clang clang /usr/bin/clang-6.0 100 \
 && update-alternatives --install /usr/bin/sancov sancov /usr/bin/sancov-6.0 100 \
 && update-alternatives --config clang \
 && update-alternatives --config sancov \
 && update-alternatives --config clang++
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
