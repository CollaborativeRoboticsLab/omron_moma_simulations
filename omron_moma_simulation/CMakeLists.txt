cmake_minimum_required(VERSION 3.8)
project(omron_moma_simulation)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/sim_gripper_action.py DESTINATION lib/${PROJECT_NAME})

ament_package()