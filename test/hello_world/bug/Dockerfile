FROM hello-world:latest

COPY bug.diff /root/
RUN cd /root/overlay_ws/src/ros_tutorials/rospy_tutorials/ && \
    git apply /root/bug.diff
