cmake_minimum_required(VERSION 3.5)
project(fishbot_navigation2)

# find dependencies
find_package(ament_cmake REQUIRED)
install(
  DIRECTORY launch param maps
  DESTINATION share/${PROJECT_NAME}
)

ament_package()