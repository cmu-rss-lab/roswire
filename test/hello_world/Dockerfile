FROM ros:indigo-ros-base
RUN apt-get update \
 && apt-get install -y \
      ros-indigo-ros-tutorials \
      ros-indigo-common-tutorials \
      vim \
      wget \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists
