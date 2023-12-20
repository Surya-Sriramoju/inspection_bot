// Copyright 2023 Tarun Trilokesh

/**
 * @file InspectorBot.hpp
 * @author Tarun Trilokesh
 * @author Sai Surya Sriramoju
 * @date 12/20/2023
 * @version 2.0
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

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

using std::chrono_literals::operator""ms;
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

    /**
    * @brief moves the robot to a specific location using the 
    * navigation stack
    */
    void goToLocation();

    /**
     * @brief This method resumes the inspection after reaching a goal
     * 
     */
    void continueInspection();

    /**
     * @brief This function controls the rotation of the bot after reaching a
     * specific goal
     */
    void rotateBot();

    /**
     * @brief Set the goal pose
     * 
     * @param x 
     * @param y 
     */
    void setLoc(float x, float y);

    /**
     * @brief Get the x coordinate of goal
     * 
     * @return float 
     */
    float getLocx();

    /**
     * @brief Get the y coordinate of the goal
     * 
     * @return float 
     */
    float getLocy();

 private:
    geometry_msgs::msg::Pose current_position;
    geometry_msgs::msg::Pose next_position;
    odom_pub::SharedPtr odom_msg_;
    publisher pose_publisher_;
    geo_twist twist_publisher_;
    TIMER timer_;
    float goal_x_;
    float goal_y_;
    bool pose_flag = false;

    /**
     * @brief callback function for moving the robot
     * 
     * @param odom_msg_i 
     */
    void inspectionCallback(const odom_pub::SharedPtr odom_msg_i);

    /**
     * @brief This is the subscriber callback for the continueInspection function
     * 
     * @param odom_msg_r 
     */
    void continueInspectionCallback(const odom_pub::SharedPtr odom_msg_r);

    turtlebot_rot bot_check_;
    std::shared_ptr<rclcpp::Node> bot_rotate_node =
      rclcpp::Node::make_shared("bot_rotate_node");
};
