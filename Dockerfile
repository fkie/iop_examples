ARG ROS_DISTRO=melodic
FROM ros:${ROS_DISTRO}

LABEL maintainer="alexander.tiderko@fkie.fraunhofer.de"

RUN apt-get update && apt-get install --no-install-recommends -y \
    vim mc bash-completion wget \
    python-wstool \
    python-lxml \
    python-catkin-tools \
    default-jdk \
    ros-${ROS_DISTRO}-moveit-msgs \
    ros-${ROS_DISTRO}-moveit-planners \
    ros-${ROS_DISTRO}-move-base \
    ros-${ROS_DISTRO}-rqt-gui \
    ros-${ROS_DISTRO}-rviz

ARG WS=/ws_iop
ARG WS_SRC=${WS}/src

RUN mkdir -p ${WS_SRC}/iop
#RUN sh -c "git clone https://github.com/fkie/iop_examples.git ${WS_SRC}/iop_examples"
RUN sh -c 'wstool init ${WS_SRC}/iop'
RUN sh -c 'wstool merge -t ${WS_SRC}/iop https://raw.githubusercontent.com/fkie/iop_node_manager/master/iop_node_manager.rosinstall'
RUN sh -c 'wstool merge -t ${WS_SRC}/iop https://raw.githubusercontent.com/fkie/iop_core/master/iop.rosinstall'
RUN sh -c 'wstool merge -t ${WS_SRC}/iop https://raw.githubusercontent.com/fkie/iop_examples/master/iop_examples.rosinstall'
RUN sh -c 'wstool update -t ${WS_SRC}/iop'
RUN rosdep install --from-paths ${WS_SRC} --ignore-src --rosdistro ${ROS_DISTRO} -y

# build ros packages
RUN ["/bin/bash", "-c", "source /opt/ros/${ROS_DISTRO}/setup.bash && cd ${WS} && catkin init && catkin build"]

# cleanup
RUN rm -rf /var/lib/apt/lists/*

COPY entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]