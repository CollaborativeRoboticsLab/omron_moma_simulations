cmake_minimum_required(VERSION 3.8)
project(omron_moma_onrobot)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/vgc10_simulator.py DESTINATION lib/${PROJECT_NAME} RENAME vgc10_simulator)

if(BUILD_TESTING)
	find_package(ament_cmake_pytest REQUIRED)
	ament_add_pytest_test(test_simulation_launch test/test_simulation_launch.py)
	ament_add_pytest_test(test_moveit_launch test/test_moveit_launch.py)
	ament_add_pytest_test(test_vgc10_simulator test/test_vgc10_simulator.py)
endif()

ament_package()