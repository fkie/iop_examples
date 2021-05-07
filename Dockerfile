ARG ROS_DISTRO=foxy
FROM ros:${ROS_DISTRO}

LABEL maintainer="alexander.tiderko@fkie.fraunhofer.de"

RUN apt-get update && apt-get install --no-install-recommends -y \
    vim mc bash-completion wget \
    python3-wstool \
    default-jdk \
    xterm \
    && rm -rf /var/lib/apt/lists/*

ARG ROS_PATH=/opt/ros/foxy
ARG WS=/ws_iop
ARG WS_SRC=${WS}/src

RUN mkdir -p ${WS_SRC}/iop
#RUN sh -c "git clone https://github.com/fkie/iop_examples.git ${WS_SRC}/iop_examples"
RUN sh -c 'wstool init ${WS_SRC}/iop'
RUN sh -c 'wstool merge -t ${WS_SRC}/iop https://raw.githubusercontent.com/fkie/iop_node_manager/foxy-devel/iop_node_manager.rosinstall'
RUN sh -c 'wstool merge -t ${WS_SRC}/iop https://raw.githubusercontent.com/fkie/iop_core/foxy-devel/iop.rosinstall'
RUN sh -c 'wstool merge -t ${WS_SRC}/iop https://raw.githubusercontent.com/fkie/iop_examples/foxy-devel/iop_examples.rosinstall'
RUN sh -c 'wstool update -t ${WS_SRC}/iop'
# install dependencies
RUN apt-get update \
    && rosdep install --from-paths ${WS_SRC} --ignore-src --rosdistro ${ROS_DISTRO} -y \
    --skip-keys catkin --skip-keys topic_tools --skip-keys stage_ros --skip-keys rviz \
    --skip-keys move_base --skip-keys hector_mapping \
    && rm -rf /var/lib/apt/lists/*

# build
RUN ["/bin/bash", "-c", "source ${ROS_PATH}/setup.bash && cd ${WS} && colcon build"]

# setup ROS if open only bash
RUN echo "source ${WS}/install/setup.bash" >> /root/.bashrc

COPY entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]