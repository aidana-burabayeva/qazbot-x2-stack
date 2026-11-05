cmake_minimum_required(VERSION 3.8)
project(qazbot_maps NONE)

find_package(ament_cmake REQUIRED)

# Occupancy maps (map.yaml + map.pgm + manifest/meta) -> share/qazbot_maps/maps/<name>/
install(DIRECTORY maps DESTINATION share/${PROJECT_NAME} USE_SOURCE_PERMISSIONS)

ament_package()
