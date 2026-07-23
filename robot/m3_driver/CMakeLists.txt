cmake_minimum_required(VERSION 3.5)

project(m3_driver)

# Require C++17
set(CMAKE_CXX_STANDARD 17)

# Find ament and ROS2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

find_package(SDL2 REQUIRED)

include_directories(
  include
  ${SDL2_INCLUDE_DIRS}
)

# Set the variable 'DEPENDENCIES'
set(DEPENDENCIES
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf2
  tf2_ros
  tf2_msgs
  tf2_geometry_msgs
)

# Executable for ISR-M3 driver
add_executable(isr_m3_driver_node
  src/driver/main.cpp
  src/driver/isr_m3_driver.cpp
)
ament_target_dependencies(isr_m3_driver_node
  ${DEPENDENCIES}
)

# Executable for Manual controller
add_executable(manual_controller_node
  src/controller/main.cpp
  src/controller/manual_controller.cpp
  src/controller/joy.cpp
  src/controller/keyboard.cpp
)
ament_target_dependencies(manual_controller_node
  ${DEPENDENCIES}
)
target_link_libraries(manual_controller_node
  ${SDL2_LIBRARIES}
)

install(TARGETS 
  isr_m3_driver_node
  manual_controller_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(
  DIRECTORY
    launch
    config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
