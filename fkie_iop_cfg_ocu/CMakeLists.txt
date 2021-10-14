cmake_minimum_required(VERSION 2.8.3)
project(fkie_iop_cfg_ocu)
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
