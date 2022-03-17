cmake_minimum_required(VERSION 3.0.2)
project(fkie_iop_cfg_sim_turtle)
find_package(catkin REQUIRED)
catkin_package()

install(
    PROGRAMS
        scripts/turtle2iop.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
    DIRECTORY
    launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
