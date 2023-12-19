/**
 * @file InspectorBot.hpp
 * @author Tarun Trilokesh
 * @author Sai Surya Sriramoju
 * @date 12/18/2023
 * @version 1.0
 * 
 * @brief InspectorBot class declaration.
 *
 * Declaration of the InspectorBot class, which handles autonomous navigation
 * and inspection tasks.
 */

#pragma once
#include <chrono>
#include <iostream>
#include <memory>
#include <vector>

#include "geometry_msgs/msg/pose.hpp"
// #include "geometry_msgs/msg/pose_stamped.hpp"
// #include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

using std::placeholders::_1;
// using publisher = rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr;
// using TIMER = rclcpp::TimerBase::SharedPtr;
// using odom = nav_msgs::msg::Odometry;
using namespace std::chrono_literals;
using std::chrono::duration;
// using bot_pose = geometry_msgs::msg::PoseStamped;
using ARUCO_TYPE = geometry_msgs::msg::Pose;

/**
 * @brief Aruco Class
 *
 */
class ArucoDetect : public rclcpp::Node {
 public:
  ArucoDetect();

  /**
   * @brief This the aruco call back function
   *
   * @param aruco_msg
   */
  void arucoCallback(const ARUCO_TYPE::SharedPtr aruco_msg);
};