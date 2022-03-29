cmake_minimum_required(VERSION 3.0.2)
project(fkie_iop_cfg_ocu)
find_package(catkin REQUIRED)

catkin_python_setup()
catkin_package()

install(
    PROGRAMS
        scripts/rtsp_driver_node.py
        scripts/swri_plan_route.py
        scripts/swri_route2iop_cmd.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
    DIRECTORY
    launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
