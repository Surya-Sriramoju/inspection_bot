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
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV 4 REQUIRED)
include_directories(include ${OpenCV_INCLUDE_DIRS})

# Include directories
include_directories(include/)

#Declare a C++ executable
add_executable(inspector_bot_node src/main.cpp src/InspectorBot.cpp)
ament_target_dependencies(inspector_bot_node rclcpp rclcpp_action gazebo_ros turtlebot3 sensor_msgs std_msgs geometry_msgs nav_msgs)

# Install executables and/or libraries
install(TARGETS
  inspector_bot_node
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(detect_aruco src/ArucoMain.cpp src/ArucoDetect.cpp)
ament_target_dependencies(detect_aruco rclcpp geometry_msgs sensor_msgs cv_bridge OpenCV)

# Install executables and/or libraries
install(TARGETS
  detect_aruco
  DESTINATION lib/${PROJECT_NAME}
)

# include_directories(${geometry_msgs_INCLUDE_DIRS})
# include_directories(${sensor_msgs_INCLUDE_DIRS})

include_directories(${geometry_msgs_INCLUDE_DIRS} ${sensor_msgs_INCLUDE_DIRS})

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
# if(BUILD_TESTING)
#   # Find package GoogleTest required for testing
#   find_package(ament_lint_auto REQUIRED)
#   find_package(GTest REQUIRED)

#   # Add the test executable
#   add_executable(inspector_bot_test test/InspectorBotTest.cpp)
#   ament_target_dependencies(inspector_bot_test rclcpp GTest)

#   # Register the test with CTest
#   gtest_discover_tests(inspector_bot_test)
# endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Export dependencies
ament_package()
