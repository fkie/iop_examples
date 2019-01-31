Setup of a simulation environment to test basic IOP cababilities

## What you need:

Install the following ROS packages (replace `kinetic` for other ROS version):

```
sudo apt install \
  ros-kinetic-hector-mapping \
  ros-kinetic-stage-ros \
  ros-kinetic-topic-tools \
  ros-kinetic-rqt-gui \
  ros-kinetic-rqt-robot-steering \
```

You should also install "teleop-twist-keyboard" if you want control your robot by keyboard.

## Get the IOP/ROS-Bridge

To install ROS/IOP-Bridge packages follow the instruction [here](https://github.com/fkie/iop_core/blob/master/README.md)

Merge the iop_cfg_sim_stage.rosinstall file and fetch code.
```
wstool merge -t src/iop https://raw.githubusercontent.com/fkie/iop_examples/master/iop_examples.rosinstall
wstool update -t src/iop
```

Finally you need to compile the sources:
```
catkin build
```

## Launch the example

There are example launch files for robots and OCU in the **fkie_iop_cfg_sim_stage** package:

- **jaus_node_manager.launch**
Starts the JAUS Node Manager. The JAUS Node Manager should be started on each host first.
> You can also start it with ```rosrun jaustoolset jaus_node_manager.sh start```. In this case `Ctrl+C` wouldn't work. Use ```rosrun jaustoolset jaus_node_manager.sh stop``` instead.

- **multirobot_example.launch**
contains the simulation configuration and also the ROS/IOP-Bridge components to make two simulated robots IOP compliant. It contains also the **JAUS Node Manager**, so you do not need to start it separately!
>requires installed multimaster_fkie

- **control_multirobot_example.launch**
contains the RQT and RVIZ gui with corresponding ROS/IOP-Bridge components for two robots. In this configuration you can control two simulated robots simultaneously. Or control one and see the sensor data of the another one.

Start JAUS Node Manager first:
```
roslaunch fkie_iop_cfg_sim_stage jaus_node_manager.launch
```

Start the launch files in two different terminals:

```
roslaunch fkie_iop_cfg_sim_stage multirobot_example.launch

roslaunch fkie_iop_cfg_sim_stage control_multirobot_example.launch
```
>you cal also use the `node_manager`, a graphical user interface from `multimaster_fkie`, to load the launch files and start the nodes.

## Control the robot

You will not see any sensor data in RViz after you start the launch files. First you have to get access to the robot. Therefor click on the **robot_0** or **robot_1** button in the "AccessControl" _rqt_ plugin. Now you should be able to see the position and laser data in RViz. You can also control the robot in the "Robot Steering" rqt-plugin.
Alternatively, you can control the robot by keyboard using the keys **i**, **j**, **k**, **l**. You have to start a new terminal and type (if you launched control_example.launch):
```
rosrun teleop_twist_keyboard teleop_twist_keyboard.py  __ns:=/ocu
```
If you launched **control_multirobot_example.launch** you need to add robot name to the namespace, e.g.:
```
rosrun teleop_twist_keyboard teleop_twist_keyboard.py  __ns:=/ocu/robot_0
```


## Examples with video

The package contains also launch files with video example:

- **multirobot_video.launch**
adds video suport to the ***multirobot_example.launch***

- **control_multirobot_video.launch**
adds video suport to the ***control_multirobot_example.launch***
