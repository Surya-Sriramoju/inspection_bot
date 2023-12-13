// Copyright 2023 Tarun Trilokesh

/**
 * @file main.cpp
 * @author Tarun Trilokesh
 * @date 12/12/2023
 * @version 1.0
 * 
 * @brief Main entry point for the InspectorBot project.
 *
 * This program initializes the ROS 2 node and starts the autonomous
 * navigation and inspection process using SLAM and ArUco markers.
 */

#include "rclcpp/rclcpp.hpp"
#include "InspectorBot.hpp"

/**
 * @brief Main function to run the InspectorBot project.
 * 
 * Initializes the ROS 2 node and starts the InspectorBot for autonomous
 * navigation and inspection in a warehouse environment.
 * 
 * @return int - Returns 0 on successful execution.
 */
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<InspectorBot>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
