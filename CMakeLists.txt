cmake_minimum_required(VERSION 3.14)
project(inspection_bot)

# Define model directory
add_definitions(-DMODELS_DIR="${CMAKE_SOURCE_DIR}/models")

# Set default values for WANT_COVERAGE and CMAKE_BUILD_TYPE
set(WANT_COVERAGE ON CACHE BOOL "Enable coverage by default")
set(CMAKE_BUILD_TYPE Debug CACHE STRING "Set build type to Debug by default")

# Fetch GoogleTest
include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/refs/tags/v1.14.0.zip
)
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

# Set C++ standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find ROS2 packages
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
find_package(image_transport REQUIRED)
find_package(OpenCV 4 REQUIRED)

# Include directories
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
  ${nav_msgs_INCLUDE_DIRS}  # Include nav_msgs directories
)

# Declare a C++ executable
add_executable(inspector_bot_node src/main.cpp src/InspectorBot.cpp)
ament_target_dependencies(inspector_bot_node
  rclcpp
  gazebo_ros
  turtlebot3
  sensor_msgs
  geometry_msgs
  std_msgs
  tf2
  tf2_ros
  nav_msgs
  cv_bridge
  image_transport
)
target_link_libraries(inspector_bot_node ${OpenCV_LIBS})

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

# Install include directory
install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

# Doxygen Support - Check if Doxygen is installed
find_package(Doxygen)
if(DOXYGEN_FOUND)
  set(DOXYGEN_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/docs)
  set(DOXYGEN_IN_DIRECTORY ${PROJECT_SOURCE_DIR}/include)
  set(DOXYGEN_DOXYFILE_IN ${PROJECT_SOURCE_DIR}/Doxyfile)

  add_custom_target(doc_doxygen ALL
    COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYGEN_DOXYFILE_IN}
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
    COMMENT "Generating API documentation with Doxygen"
    VERBATIM
  )
else()
  message(FATAL_ERROR "Doxygen needs to be installed to generate the doxygen documentation")
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON) # Enable compile_commands.json
execute_process(COMMAND cmake -E create_symlink
                "${PROJECT_BINARY_DIR}/compile_commands.json"
                "${PROJECT_SOURCE_DIR}/compile_commands.json")

# Coverage - Add compiler flags for coverage if required
if(WANT_COVERAGE)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O0 -fprofile-arcs -ftest-coverage")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -g -O0 -fprofile-arcs -ftest-coverage")
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -fprofile-arcs -ftest-coverage")

  # Add a target for running tests and generating coverage reports
  find_package(GTest REQUIRED)
  include_directories(${GTEST_INCLUDE_DIRS})

  add_executable(test_inspector_bot test/InspectorBotTest.cpp)
  ament_target_dependencies(test_inspector_bot
    rclcpp
    sensor_msgs
    geometry_msgs
    cv_bridge
    image_transport
    nav_msgs  # Ensure nav_msgs is included for the test executable
  )
  target_link_libraries(test_inspector_bot ${GTEST_LIBRARIES} ${OpenCV_LIBS})
  gtest_discover_tests(test_inspector_bot)

  # Add custom target for generating coverage report
  add_custom_target(coverage
    COMMAND ${CMAKE_COMMAND} -E make_directory "${CMAKE_CURRENT_BINARY_DIR}/coverage"
    COMMAND ${CMAKE_COMMAND} -E chdir "${CMAKE_CURRENT_BINARY_DIR}/coverage" lcov --capture --directory ${CMAKE_CURRENT_BINARY_DIR} --output-file coverage.info
    COMMAND ${CMAKE_COMMAND} -E chdir "${CMAKE_CURRENT_BINARY_DIR}/coverage" lcov --remove coverage.info '/usr/*' '*/test/*' '*/src/*' --output-file coverage_filtered.info
    COMMAND ${CMAKE_COMMAND} -E chdir "${CMAKE_CURRENT_BINARY_DIR}/coverage" lcov --list coverage_filtered.info
    DEPENDS test_inspector_bot
    COMMENT "Collecting lcov coverage data"
  )
endif()

# Export dependencies
ament_package()
