cmake_minimum_required(VERSION 3.8)
project(qazbot_lidar_projector)

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
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# Pure core: no ROS. -ffp-contract=off keeps every operation separately rounded, like the Python/numpy reference
# (a fused multiply-add would change the last bit and could move a point across a bin edge).
add_library(lidar_projection SHARED src/lidar_projection.cpp)
target_include_directories(lidar_projection PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_options(lidar_projection PRIVATE -ffp-contract=off)
if(ENABLE_COVERAGE)
  target_compile_options(lidar_projection PRIVATE --coverage -O0)
  target_link_libraries(lidar_projection PUBLIC --coverage)
endif()

add_library(lidar_projector_component SHARED src/lidar_projector_node.cpp)
target_link_libraries(lidar_projector_component lidar_projection)
ament_target_dependencies(lidar_projector_component
  rclcpp rclcpp_components sensor_msgs nav_msgs geometry_msgs std_msgs tf2_ros)
rclcpp_components_register_node(lidar_projector_component
  PLUGIN "qazbot_lidar_projector::LidarProjectorNode"
  EXECUTABLE lidar_projector_composable)

add_executable(lidar_projector src/lidar_projector_main.cpp)
target_link_libraries(lidar_projector lidar_projector_component)
ament_target_dependencies(lidar_projector rclcpp)

install(TARGETS lidar_projection lidar_projector_component
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(TARGETS lidar_projector DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)

  ament_add_gtest(test_lidar_projection test/test_lidar_projection.cpp)
  target_link_libraries(test_lidar_projection lidar_projection)

  # Offline helpers for the parity test and the benchmark (core library only, no ROS).
  add_executable(parity_dump test/parity_dump.cpp)
  target_link_libraries(parity_dump lidar_projection)
  add_executable(bench_core test/bench_core.cpp)
  target_link_libraries(bench_core lidar_projection)
  install(TARGETS parity_dump bench_core DESTINATION lib/${PROJECT_NAME})

  ament_add_pytest_test(test_parity test/test_parity.py
    ENV PARITY_DUMP=${CMAKE_CURRENT_BINARY_DIR}/parity_dump
        FIXTURE=${CMAKE_CURRENT_SOURCE_DIR}/test/data/static_204153_w50.bin
    TIMEOUT 300)
  add_launch_test(test/test_integration.py
    TARGET test_integration
    ENV ROS_DOMAIN_ID=77 FIXTURE=${CMAKE_CURRENT_SOURCE_DIR}/test/data/static_204153_w50.bin
    TIMEOUT 120)
endif()

ament_package()
