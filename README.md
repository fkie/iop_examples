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
wstool merge -t src https://raw.githubusercontent.com/fkie/iop_cfg_sim_stage_fkie/master/iop_cfg_sim_stage.rosinstall
wstool update -t src
```

Finally you need to compile the sources:
```
catkin build
```

## Launch the example

There are two example launch files in the **iop_cfg_sim_stage_fkie** package:

- **iop_robot.launch**
contains the simulation configuration and also the ROS/IOP-Bridge components to make the simulated robot IOP compliant. It contains also the **JAUS Node Manager**, so you do not need to start it separately!
>requires installed multimaster_fkie

- **iop_control.launch**
contains the RQT and RVIZ gui with corresponding ROS/IOP-Bridge components. So you can control the simulated robot by IOP commands.

Start the launch files in two different terminals:

```
roslaunch iop_cfg_sim_stage_fkie iop_robot.launch

roslaunch iop_cfg_sim_stage_fkie iop_control.launch
```

## Control the robot

You will not see any sensor data in RViz after you start the launch files. First you have to get access to the robot. Therefor click on the **bobrob** button in the "AccessControl" _rqt_ plugin. Now you should be able to see the position and laser data in RViz. You can also control the robot in the "Robot Steering" rqt-plugin.
Alternatively, you can control the robot by keyboard using the keys **i**, **j**, **k**, **l**. You have to select the terminal where you launched the iop_control.launch to grab the keys. Or start a new terminal and type:
```
rosrun teleop_twist_keyboard teleop_twist_keyboard.py  __ns:=/control
```
