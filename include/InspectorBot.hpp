// Copyright 2023 Tarun Trilokesh

/**
 * @file InspectorBot.hpp
 * @author Tarun Trilokesh
 * @author Sai Surya Sriramoju
 * @date 12/12/2023
 * @version 1.0
 * 
 * @brief InspectorBot class declaration.
 *
 * Declaration of the InspectorBot class, which handles autonomous navigation
 * and inspection tasks.
 */

#ifndef INSPECTOR_BOT_HPP_
#define INSPECTOR_BOT_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <chrono>
#include <iostream>
#include <memory>
#include <vector>

/**
 * @brief The InspectorBot class.
 * 
 * This class encapsulates methods and attributes for autonomous navigation
 * and inspection using SLAM and ArUco markers.
 */
class InspectorBot : public rclcpp::Node {
 public:
    InspectorBot();

 private:
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
};

#endif  // INSPECTOR_BOT_HPP_

