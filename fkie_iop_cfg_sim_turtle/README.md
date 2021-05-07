A very simple example to show the start with ROS/IOP-Bridge using turtlesim.

## What you need:

Install the following ROS packages (replace `foxy` for other ROS version):

```
sudo apt install \
  ros-foxy-turtlesim \
  ros-foxy-rqt-gui
```

## Get the IOP/ROS-Bridge

To install ROS/IOP-Bridge packages follow the instruction [here](https://github.com/fkie/iop_core/blob/master/README.md)

Merge the iop_example.rosinstall file and fetch code.
```
wstool merge -t src/iop https://raw.githubusercontent.com/fkie/iop_examples/foxy-devel/iop_examples.rosinstall
wstool update -t src/iop
```

Finally you need to compile the sources:
```
colcon build
```

## Launch the example

There are example launch files for robots and OCU in the **fkie_iop_cfg_sim_turtle** package:

- **iop_node_manager.launch**
Starts the IOP Node Manager. The IOP Node Manager should be started on each host first.

- **turtle.launch.xml**
contains the configuration for turtlesim and also the ROS/IOP-Bridge components to make turtlebot IOP compliant.

- **control.launch.xml**
contains the RQT gui with AccessControl and ROS/IOP-Bridge components.

Start IOP Node Manager first:
```
ros2 launch fkie_iop_cfg_sim_turtle iop_node_manager.launch.xml
```

Start the launch files in two different terminals (also on different hosts):

```
ros2 launch fkie_iop_cfg_sim_turtle turtle.launch.xml

ros2 launch fkie_iop_cfg_sim_turtle control.launch.xml
```
