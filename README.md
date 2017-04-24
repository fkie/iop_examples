Setup of a simulation environment to test basic IOP cababilities

## What you need:

Install the following ROS packages depending on your ROS version:
```
sudo apt install \
  ros-indigo-hector-mapping \
  ros-indigo-stage-ros \
  ros-indigo-topic-tools \
  ros-indigo-rqt-gui \
  ros-indigo-rqt-robot-steering \
```

OR

```
sudo apt install \
  ros-kinetic-hector-mapping \
  ros-kinetic-stage-ros \
  ros-kinetic-topic-tools \
  ros-kinetic-rqt-gui \
  ros-kinetic-rqt-robot-steering \
```

You should also install "teleop-twist-keyboard" if you want control your robot by keyboard.

## Install JAUS Tool Set

See the installation instructions on https://gitlab.fkie.fraunhofer.de/jaus/iop_core/blob/master/doc/install.md to install the JAUS Tool Set.

## Get the IOP/ROS-Bridge

To run the simulator you do not need all ROS/IOP-Bridge packages. It is sufficient to clone the following repositories into your ROS workspace:

```
git clone https://gitlab.fkie.fraunhofer.de/jaus/core
git clone https://gitlab.fkie.fraunhofer.de/jaus/mobility
git clone https://gitlab.fkie.fraunhofer.de/jaus/sensing
git clone https://gitlab.fkie.fraunhofer.de/jaus/mobility_clients
git clone https://gitlab.fkie.fraunhofer.de/jaus/sensing_clients
git clone https://gitlab.fkie.fraunhofer.de/jaus/iop_core
git clone https://gitlab.fkie.fraunhofer.de/jaus/iop_platform
git clone https://gitlab.fkie.fraunhofer.de/jaus/iop_sensing
git clone https://gitlab.fkie.fraunhofer.de/jaus/iop_sensing_clients
git clone https://gitlab.fkie.fraunhofer.de/jaus/iop_gui
git clone https://gitlab.fkie.fraunhofer.de/jaus/iop_cfg_sim_stage_fkie
```

Finally you need to compile the sources:
```
catkin build
```

## Launch the example

Before you start the launch files, you have to start the **JAUS Node Manager**. You can do this by

  ```rosrun iop_builder_fkie jaus_node_manager.sh start```
  
>To exit the script type `rosrun iop_builder_fkie jaus_node_manager.sh stop` in a new terminal window (`CTRL+C` won't work).

There are two example launch files in the **iop_cfg_sim_stage_fkie** package:

- **iop_robot.launch**
contains the simulation configuration and also the ROS/IOP-Bridge components to make the simulated robot IOP compliant.

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
