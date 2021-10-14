An example of usage the ROS/IOP-Bridge with mapviz.


## Setup

To install ROS/IOP-Bridge packages follow the instruction [here](https://github.com/fkie/iop_core/blob/master/README.md)

Go into your ROS workspace.

Merge the iop_example.rosinstall file and fetch code.
```
wstool merge -t src/iop https://raw.githubusercontent.com/fkie/iop_examples/master/iop_examples.rosinstall
wstool update -t src/iop
```

Install dependecies
```
rosdep install --from-paths src/iop/iop_examples/fkie_iop_cfg_ocu --ignore-src --rosdistro ${ROS_DISTRO} -y
```

Finally you need to compile the sources:
```
catkin build fkie_iop_cfg_ocu
```

## Launch the example

```
roslaunch fkie_iop_cfg_ocu control.launch
```

After rqt was started, load _mapviz_ configuration (iop.mvc) from _fkie_iop_cfg_ocu_ package.
