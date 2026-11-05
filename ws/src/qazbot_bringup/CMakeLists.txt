cmake_minimum_required(VERSION 3.8)
project(qazbot_bringup NONE)

find_package(ament_cmake REQUIRED)

file(GLOB _scripts "${CMAKE_CURRENT_SOURCE_DIR}/scripts/*")
install(PROGRAMS ${_scripts} DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME} USE_SOURCE_PERMISSIONS)

# Build manifest (git SHA) regenerated on every build; bringup_info.py logs it at launch.
set(_manifest "${CMAKE_CURRENT_BINARY_DIR}/build_manifest.yaml")
add_custom_target(build_manifest ALL
  COMMAND ${CMAKE_COMMAND} -DSRC_DIR=${CMAKE_CURRENT_SOURCE_DIR} -DOUT=${_manifest}
          -P ${CMAKE_CURRENT_SOURCE_DIR}/cmake/gen_manifest.cmake
  BYPRODUCTS ${_manifest}
  COMMENT "qazbot_bringup: build_manifest.yaml")
install(FILES ${_manifest} DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(${PROJECT_NAME}_unit test
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/test
    TIMEOUT 300)
endif()

ament_package()
