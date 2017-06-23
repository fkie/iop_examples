cmake_minimum_required(VERSION 2.8.3)
project(iop_cfg_sim_stage_fkie)
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
