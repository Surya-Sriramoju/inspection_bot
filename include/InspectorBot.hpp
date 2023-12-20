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

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <iostream>
#include <memory>
#include <vector>

using namespace std::chrono_literals;
using std::chrono::duration;
using std::placeholders::_1;
using publisher = rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr;
using geo_twist = rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr;
using TIMER = rclcpp::TimerBase::SharedPtr;
using odom_pub = nav_msgs::msg::Odometry;
using turtlebot_pose = geometry_msgs::msg::PoseStamped;
using turtlebot_rot = geometry_msgs::msg::Twist;


/**
 * @brief The InspectorBot class.
 * 
 * This class encapsulates methods and attributes for autonomous navigation
 * and inspection using SLAM and ArUco markers.
 */
class InspectorBot : public rclcpp::Node {
 public:
    InspectorBot();
    bool isGoalReached() const;

    void goToLocation();

    void continueInspection();

    void rotateBot();

    void setLoc(float x, float y);

    float getLocx();

    float getLocy();

 private:
    // void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    // rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    // rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;

    geometry_msgs::msg::Pose current_position;
    geometry_msgs::msg::Pose next_position;
    odom_pub::SharedPtr odom_msg_;
    publisher pose_publisher_;
    geo_twist twist_publisher_;
    TIMER timer_;
    float goal_x_;
    float goal_y_;
    bool pose_flag = false;

    void inspectionCallback(const odom_pub::SharedPtr odom_msg_i);

    void continueInspectionCallback(const odom_pub::SharedPtr odom_msg_r);

    turtlebot_rot bot_check_;
    std::shared_ptr<rclcpp::Node> bot_rotate_node =
      rclcpp::Node::make_shared("bot_rotate_node");

};

#endif  // INSPECTOR_BOT_HPP_
