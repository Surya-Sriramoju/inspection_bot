cmake_minimum_required(VERSION 3.8)
project(inspection_bot)

# Compile options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gazebo_ros REQUIRED)  # Gazebo ROS package
find_package(turtlebot3 REQUIRED)  # TurtleBot3 package
find_package(sensor_msgs REQUIRED) # Add sensor_msgs package

# Include directories
include_directories(include)

# Declare a C++ executable
add_executable(inspector_bot_node src/main.cpp src/InspectorBot.cpp)
ament_target_dependencies(inspector_bot_node rclcpp gazebo_ros turtlebot3 sensor_msgs)

# Install executables and/or libraries
install(TARGETS
  inspector_bot_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Install world files
install(DIRECTORY worlds
  DESTINATION share/${PROJECT_NAME}
)

# Install config files
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# Install include directory
install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

# Check if testing is enabled
if(BUILD_TESTING)
  # Find package GoogleTest required for testing
  find_package(ament_lint_auto REQUIRED)
  find_package(GTest REQUIRED)

  # Add the test executable
  add_executable(inspector_bot_test src/InspectorBotTest.cpp)
  ament_target_dependencies(inspector_bot_test rclcpp GTest)

  # Register the test with CTest
  gtest_discover_tests(inspector_bot_test)
endif()

# Export dependencies
ament_package()
