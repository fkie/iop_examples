An example configuration how to use video with ROS/IOP-Bridge.

## What you need:

Install the following ROS packages (replace `kinetic` for other ROS version):

```
sudo apt install ros-kinetic-rqt-gui
```

## Get the IOP/ROS-Bridge

To install ROS/IOP-Bridge packages follow the instruction [here](https://github.com/fkie/iop_core/blob/master/README.md)

Merge the iop_example.rosinstall file and fetch code.
```
wstool merge -t src/iop https://raw.githubusercontent.com/fkie/iop_examples/master/iop_examples.rosinstall
wstool update -t src/iop
```

Finally you need to compile the sources:
```
catkin build
```

## Launch the example

There are example launch files for robots and OCU in the **fkie_iop_cfg_sim_video** package:

- **jaus_node_manager.launch**
Starts the JAUS Node Manager. The JAUS Node Manager should be started on each host first.
> You can also start it with ```rosrun jaustoolset jaus_node_manager.sh start```. In this case `Ctrl+C` wouldn't work. Use ```rosrun jaustoolset jaus_node_manager.sh stop``` instead.
>requires installed multimaster_fkie

- **robot.launch**
contains the configuration for video on robot and also the ROS/IOP-Bridge components to make video available for IOP.

- **control.launch**
contains the RQT gui with AccessControl, DigitalResourceViewer and ROS/IOP-Bridge components.


Start JAUS Node Manager first:
```
roslaunch fkie_iop_cfg_sim_video jaus_node_manager.launch
```

Start the launch files in two different terminals (also on different hosts):

```
roslaunch fkie_iop_cfg_sim_video robot.launch

roslaunch fkie_iop_cfg_sim_video control.launch
```
>you cal also use the `node_manager`, a graphical user interface from `multimaster_fkie`, to load the launch files and start the nodes.
