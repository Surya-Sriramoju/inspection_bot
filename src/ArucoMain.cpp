// Copyright 2023 Sai Surya Sriramoju

/**
 * @file ArucoMain.cpp
 * @author Sai Surya Sriramoju
 * @brief Main file for launching the ArUco detection node.
 * @date 12/20/2023
 * @version 1.0
 * 
 * This file contains the main function to initialize and run the ArUcoDetectionNode,
 * which is responsible for detecting ArUco markers in images.
 */

#include "ArucoDetect.hpp"

/**
 * @brief Main function for the ArUco detection node.
 * 
 * Initializes the ROS2 node, creates an instance of ArUcoDetectionNode, and
 * runs the node until it's shut down. This is the entry point for the ArUco
 * marker detection functionality.
 * 
 * @param argc Argument count.
 * @param argv Argument vector.
 * @return int Execution status code.
 */
int main(int argc, char *argv[]) {
  // Initialize ROS2
  rclcpp::init(argc, argv);

  // Create and spin the ArUco detection node
  rclcpp::spin(std::make_shared<ArUcoDetectionNode>());

  // Shutdown ROS2
  rclcpp::shutdown();

  return 0;
}
