This repository is part of [ROS/IOP Bridge](https://github.com/fkie/iop_core).

It contains ROS-packages with different examples for using ROS/IOP-Bridge.

[fkie_iop_cfg_sim_turtle](https://github.com/fkie/iop_examples/tree/foxy-devel/fkie_iop_cfg_sim_turtle): simple example shows the usage of PrimitiveDriver with turtle simulator.  
[fkie_iop_cfg_sim_stage_waypoint](https://github.com/fkie/iop_examples/tree/foxy-devel/fkie_iop_cfg_sim_stage_waypoint):  shows the integration of local waypoint navigation with stage simulator. Local robot position and  laser scans are also visualized on the OCU-site.  
[fkie_iop_cfg_sim_stage](https://github.com/fkie/iop_examples/tree/foxy-devel/fkie_iop_cfg_sim_stage): complex example with multi-robot, map and video integration using stage simulator.
[fkie_iop_cfg_sim_video](https://github.com/fkie/iop_examples/tree/foxy-devel/fkie_iop_cfg_sim_video): shows how to provide video on robot and consume the IOP video on OCU.



## Use in a Docker

    docker build -t fkie:iop .


Launch example:

    # for Rviz, Rqt and Stage GUI
    xhost +local:root

    # 1. Terminal
    docker run -it --env="DISPLAY=$DISPLAY" --env="QT_X11_NO_MITSHM=1"  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" --env="XAUTHORITY=$XAUTHORITY" --volume="$XAUTHORITY:$XAUTHORITY" --device=/dev/dri:/dev/dri --security-opt apparmor:unconfined -v /tmp:/tmp --rm fkie:iop ros2 launch fkie_iop_cfg_sim_turtle robot_nm.launch.xml

    # 2. Terminal
    docker run -it --env="DISPLAY=$DISPLAY" --env="QT_X11_NO_MITSHM=1"  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" --env="XAUTHORITY=$XAUTHORITY" --volume="$XAUTHORITY:$XAUTHORITY" --device=/dev/dri:/dev/dri --security-opt apparmor:unconfined -v /tmp:/tmp --rm fkie:iop ros2 launch fkie_iop_cfg_sim_turtle control_nm.launch.xml




