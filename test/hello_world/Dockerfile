FROM ros:indigo-ros-base
# install ros tutorials packages
RUN apt-get update && apt-get install -y \
    ros-indigo-ros-tutorials \
    ros-indigo-common-tutorials \
    vim wget \
    && rm -rf /var/lib/apt/lists/ \
    && apt-get clean
