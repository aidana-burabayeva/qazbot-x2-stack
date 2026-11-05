cmake_minimum_required(VERSION 3.8)
project(qazbot_approach_governor)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
option(ENABLE_COVERAGE "Instrument the core library with gcov" OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# Pure core: no ROS.
add_library(approach_governor_core SHARED src/approach_governor.cpp)
target_include_directories(approach_governor_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
if(ENABLE_COVERAGE)
  target_compile_options(approach_governor_core PRIVATE --coverage -O0)
  target_link_libraries(approach_governor_core PUBLIC --coverage)
endif()

add_library(approach_governor_component SHARED src/approach_governor_node.cpp)
target_link_libraries(approach_governor_component approach_governor_core)
ament_target_dependencies(approach_governor_component
  rclcpp rclcpp_components geometry_msgs nav_msgs diagnostic_msgs tf2 tf2_ros)
rclcpp_components_register_node(approach_governor_component
  PLUGIN "qazbot_approach_governor::ApproachGovernorNode"
  EXECUTABLE approach_governor_composable)

add_executable(approach_governor src/approach_governor_main.cpp)
target_link_libraries(approach_governor approach_governor_component)
ament_target_dependencies(approach_governor rclcpp)

install(TARGETS approach_governor_core approach_governor_component
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(TARGETS approach_governor DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)

  ament_add_gtest(test_approach_governor test/test_approach_governor.cpp)
  target_link_libraries(test_approach_governor approach_governor_core)

  # SIMULATION (not a replay): data-derived plant model, closed loop with a simplified RPP approach.
  ament_add_gtest(test_sim_regression test/test_sim_regression.cpp)
  target_link_libraries(test_sim_regression approach_governor_core)

  add_launch_test(test/test_node_integration.py
    TARGET test_node_integration
    ENV ROS_DOMAIN_ID=79
    TIMEOUT 120)
endif()

ament_package()
