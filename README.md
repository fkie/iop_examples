This repository is part of [ROS/IOP Bridge](https://github.com/fkie/iop_core/blob/master/README.md).

It contains ROS-packages with different examples for using ROS/IOP-Bridge.

[fkie_iop_cfg_sim_turtle](https://github.com/fkie/iop_examples/tree/master/fkie_iop_cfg_sim_turtle): simple example shows the usage of PrimitiveDriver with turtle simulator.  
[fkie_iop_cfg_sim_stage_waypoint](https://github.com/fkie/iop_examples/tree/master/fkie_iop_cfg_sim_stage_waypoint):  shows the integration of local waypoint navigation with stage simulator. Local robot position and  laser scans are also visualized on the OCU-site.  
[fkie_iop_cfg_sim_stage](https://github.com/fkie/iop_examples/tree/master/fkie_iop_cfg_sim_stage): complex example with multi-robot, map and video integration using stage simulator.
[fkie_iop_cfg_sim_video](https://github.com/fkie/iop_examples/tree/master/fkie_iop_cfg_sim_video): shows how to provide video on robot and consume the IOP video on OCU.



## Use in a Docker

    docker build -t fkie:iop .


Launch example:

    # for Rviz, Rqt and Stage GUI
    xhost +local:root

    # 1. Terminal
    docker run -it --network host  --env="DISPLAY=$DISPLAY" --env="QT_X11_NO_MITSHM=1"  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" --env="XAUTHORITY=$XAUTHORITY" --volume="$XAUTHORITY:$XAUTHORITY" --device=/dev/dri:/dev/dri --security-opt apparmor:unconfined -v /tmp:/tmp --rm fkie:iop rosrun fkie_iop_node_manager rosiopnodemanager.py

    # 2. Terminal
    docker run -it --network host  --env="DISPLAY=$DISPLAY" --env="QT_X11_NO_MITSHM=1"  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" --env="XAUTHORITY=$XAUTHORITY" --volume="$XAUTHORITY:$XAUTHORITY" --device=/dev/dri:/dev/dri --security-opt apparmor:unconfined -v /tmp:/tmp --rm fkie:iop roslaunch fkie_iop_cfg_sim_stage_waypoint robot.launch

    # 3. Terminal
    docker run -it --network host  --env="DISPLAY=$DISPLAY" --env="QT_X11_NO_MITSHM=1"  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" --env="XAUTHORITY=$XAUTHORITY" --volume="$XAUTHORITY:$XAUTHORITY" --device=/dev/dri:/dev/dri --security-opt apparmor:unconfined -v /tmp:/tmp --rm fkie:iop roslaunch fkie_iop_cfg_sim_stage_waypoint control.launch




