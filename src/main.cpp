// Copyright 2023 Tarun Trilokesh

/**
 * @file main.cpp
 * @author Tarun Trilokesh
 * @author Sai Surya Sriramoju
 * @date 12/12/2023
 * @version 1.0
 * 
 * @brief Main entry point for the InspectorBot project.
 *
 * This program initializes the ROS 2 node and starts the autonomous
 * navigation and inspection process using SLAM and ArUco markers.
 */

// #include "rclcpp/rclcpp.hpp"
#include "InspectorBot.hpp"

/**
 * @brief Main function to run the InspectorBot project.
 * 
 * Initializes the ROS 2 node and starts the InspectorBot for autonomous
 * navigation and inspection in a warehouse environment.
 * 
 * @return int - Returns 0 on successful execution.

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    InspectorBot inspector;

    std::vector<std::pair<float, float>> goals = {{1.5, 2.0}, {2.0, -1.0}, {0.0, -2.7}};

    for (auto& goal : goals) {
        inspector.setLoc(goal.first, goal.second);
        inspector.goToLocation();
        // rclcpp::sleep_for(7000ms);
        inspector.rotateBot();
        // rclcpp::sleep_for(2000ms);
    }

    inspector.continueInspection();
    rclcpp::shutdown();
}
