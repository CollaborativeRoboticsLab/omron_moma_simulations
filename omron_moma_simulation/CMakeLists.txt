cmake_minimum_required(VERSION 3.8)
project(omron_moma_simulation)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
	find_package(ament_cmake_pytest REQUIRED)
	ament_add_pytest_test(test_wrist_camera test/test_wrist_camera.py)
	ament_add_pytest_test(test_moveit_launch test/test_moveit_launch.py)
endif()

ament_package()