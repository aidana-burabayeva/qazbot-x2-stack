cmake_minimum_required(VERSION 3.8)
project(qazbot_navigation NONE)

find_package(ament_cmake REQUIRED)

# Python nodes and their helper modules are installed side by side in lib/${PROJECT_NAME}
# (ros2 run qazbot_navigation <node>.py; launch files call them by that path). ament_cmake + install(PROGRAMS)
# keeps the legacy script names/imports unchanged (no setup.py entry-point wrappers).
file(GLOB _scripts "${CMAKE_CURRENT_SOURCE_DIR}/scripts/*")
install(PROGRAMS ${_scripts} DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch bt rviz DESTINATION share/${PROJECT_NAME} USE_SOURCE_PERMISSIONS)

if(BUILD_TESTING AND TRUE)
  find_package(ament_cmake_pytest REQUIRED)
  # Unit tests of the application logic (no robot, no ROS graph). Run with ROS_DOMAIN_ID != 0.
  ament_add_pytest_test(${PROJECT_NAME}_unit test
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/test
    TIMEOUT 900)
endif()

ament_package()
