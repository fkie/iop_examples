cmake_minimum_required(VERSION 3.0.2)
project(fkie_iop_cfg_sim_stage)
find_package(catkin REQUIRED)
catkin_package()

install(
    PROGRAMS
        scripts/tf2satfix.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
    DIRECTORY
    launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
