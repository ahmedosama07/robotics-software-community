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

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)

  # Add test executable (don't include main.cpp to avoid multiple main definitions)
  ament_add_gtest(test_temperature_subscriber
    test/test_temperature_subscriber.cpp
    src/temperature_subscriber.cpp  # Include the implementation without main
  )
  target_include_directories(test_temperature_subscriber PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
  ament_target_dependencies(test_temperature_subscriber
    rclcpp
    temperature_interfaces
  )

  ament_lint_auto_find_test_dependencies()
endif()

ament_package()