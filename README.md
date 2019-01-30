A very simple example to show the start with ROS/IOP-Bridge using turtlesim.

## What you need:

Install the following ROS packages (replace `kinetic` for other ROS version):

```
sudo apt install \
  ros-kinetic-turtlesim \
  ros-kinetic-rqt-gui
```

## Get the IOP/ROS-Bridge

To install ROS/IOP-Bridge packages follow the instruction [here](https://github.com/fkie/iop_core/blob/master/README.md)

Merge the iop_cfg_sim_turtle.rosinstall file and fetch code.
```
wstool merge -t src/iop https://raw.githubusercontent.com/fkie/fkie_iop_cfg_sim_turtle/master/iop_cfg_sim_turtle.rosinstall
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

- **turtle.launch**
contains the configuration for turtlesim and also the ROS/IOP-Bridge components to make turtlebot IOP compliant. >requires installed multimaster_fkie

- **control.launch**
contains the RQT gui with AccessControl and ROS/IOP-Bridge components.

Start JAUS Node Manager first:
```
roslaunch fkie_iop_cfg_sim_turtle jaus_node_manager.launch
```

Start the launch files in two different terminals (also on different hosts):

```
roslaunch fkie_iop_cfg_sim_turtle turtle.launch

roslaunch fkie_iop_cfg_sim_turtle control.launch
```
>you cal also use the `node_manager`, a graphical user interface from `multimaster_fkie`, to load the launch files and start the nodes.

>for more complex example see [fkie_iop_cfg_sim_stage](https://github.com/fkie/fkie_iop_cfg_sim_stage/blob/master/README.md)
