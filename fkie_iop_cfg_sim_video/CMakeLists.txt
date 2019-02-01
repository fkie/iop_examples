cmake_minimum_required(VERSION 2.8.3)
project(fkie_iop_cfg_sim_video)
find_package(catkin REQUIRED)
catkin_package()

install(
    DIRECTORY
    launch
    resources
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
