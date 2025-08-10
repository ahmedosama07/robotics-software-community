cmake_minimum_required(VERSION 3.8)
project(temperature_monitor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(temperature_interfaces REQUIRED)

# Include directories
include_directories(
  include
)

# Create executable with both source files
add_executable(temperature_subscriber
  src/temperature_subscriber.cpp
  src/main.cpp
)

# Specify dependencies
ament_target_dependencies(temperature_subscriber
  rclcpp
  temperature_interfaces
)

# Install executables
install(TARGETS
  temperature_subscriber
  DESTINATION lib/${PROJECT_NAME}
)

# Install header files
install(DIRECTORY include/
  DESTINATION include
)



ament_package()