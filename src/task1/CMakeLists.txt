cmake_minimum_required(VERSION 3.5)
project(task1)

# Set C++ standard to C++17.
set(CMAKE_CXX_STANDARD 17)

# Find required packages.
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

set(dependencies
  rclcpp
  sensor_msgs
  geometry_msgs
)

# Define the executable target.
# Adjust the paths if your files are organized differently.
add_executable(node
  src/task1_main.cpp
  src/task1/BumpGoNode1.cpp
)

target_include_directories(node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(node ${dependencies})

install(TARGETS
  node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
