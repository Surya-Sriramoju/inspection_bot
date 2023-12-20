// Copyright 2023 Tarun Trilokesh

/**
 * @file InspectorBot.cpp
 * @author Tarun Trilokesh
 * @author Sai Surya Sriramoju
 * @date 12/20/2023
 * @version 1.0
 * 
 * @brief InspectorBot class implementation.
 *
 * Implementation of the InspectorBot class, which handles autonomous navigation
 * and inspection tasks.
 */

#include "InspectorBot.hpp"


/**
 * @brief Constructor for the InspectorBot class.
 * 
 * Initializes the node and sets the initial position of the robot.
 */
InspectorBot::InspectorBot() : Node("inspector_bot") {
    current_position.position.x = 0.0;
    next_position.position.x = 0.0;
}

/**
 * @brief Moves the robot to a specified location.
 * 
 * This method publishes a goal pose to the robot and continuously checks
 * the robot's odometry to determine when it has reached the goal.
 */
void InspectorBot::goToLocation() {
    pose_publisher_ = this->create_publisher<turtlebot_pose>("/goal_pose", 10);
    std::shared_ptr<rclcpp::Node> odom_node =
      rclcpp::Node::make_shared("odom_node");

    auto odom_subscriber = odom_node->create_subscription<odom_pub>(
      "odom", 10, std::bind(&InspectorBot::inspectionCallback, this, _1));

    turtlebot_pose goal_pose_;
    goal_pose_.header.frame_id = "map";
    goal_pose_.header.stamp.sec = 0;
    goal_pose_.header.stamp.nanosec = 0;
    goal_pose_.pose.position.x = goal_x_;
    goal_pose_.pose.position.y = goal_y_;
    goal_pose_.pose.position.z = 0;
    goal_pose_.pose.orientation.x = 0;
    goal_pose_.pose.orientation.y = 0;
    goal_pose_.pose.orientation.z = 0;
    goal_pose_.pose.orientation.w = 1;

    while (!pose_flag) {
        rclcpp::spin_some(odom_node);
        pose_publisher_->publish(goal_pose_);
        RCLCPP_INFO(this->get_logger(), "Moving to inspect location");
        rclcpp::sleep_for(500ms);
    }
    pose_flag = false;
}

/**
 * @brief Sets the goal location for the robot.
 * 
 * @param x X-coordinate of the goal location.
 * @param y Y-coordinate of the goal location.
 */
void InspectorBot::setLoc(float x, float y) {
  goal_x_ = x;
  goal_y_ = y;
}

/**
 * @brief Gets the x-coordinate of the goal location.
 * 
 * @return float X-coordinate of the goal.
 */
float InspectorBot::getLocx() { return goal_x_; }

/**
 * @brief Gets the y-coordinate of the goal location.
 * 
 * @return float Y-coordinate of the goal.
 */
float InspectorBot::getLocy() { return goal_y_; }

/**
 * @brief Rotates the robot to inspect the surrounding area.
 * 
 * This method rotates the robot in place, allowing it to inspect its
 * surroundings at the current location.
 */
void InspectorBot::rotateBot() {
  twist_publisher_ = this->create_publisher<turtlebot_rot>("/cmd_vel", 10);

  // bot_check_.angular.z = 0.5;

  int count = 20;

  while (count) {
    if (count <= 10) {
      bot_check_.angular.z = 1.8;
      rclcpp::spin_some(bot_rotate_node);
      twist_publisher_->publish(bot_check_);
      RCLCPP_INFO(this->get_logger(), "Looking around..");
      count--;
      rclcpp::sleep_for(500ms);
    } else {
      bot_check_.angular.z = -1.8;
      rclcpp::spin_some(bot_rotate_node);
      twist_publisher_->publish(bot_check_);
      RCLCPP_INFO(this->get_logger(), "Looking around..");
      count--;
      rclcpp::sleep_for(500ms);
    }
  }
}

/**
 * @brief Continues the inspection process after reaching a goal.
 * 
 * This method is used to navigate the robot back to the base station
 * after completing an inspection at a goal location.
 */
void InspectorBot::continueInspection() {
  std::shared_ptr<rclcpp::Node> odom_node_1 =
      rclcpp::Node::make_shared("odom_node_1");

  auto odom_subscriber = odom_node_1->create_subscription<odom_pub>(
      "odom", 10, std::bind(&InspectorBot::continueInspectionCallback,
      this, _1));

  turtlebot_pose goal_pose_1;

  goal_pose_1.header.frame_id = "map";
  goal_pose_1.header.stamp.sec = 0;
  goal_pose_1.header.stamp.nanosec = 0;
  goal_pose_1.pose.position.x = 0;
  goal_pose_1.pose.position.y = 0;
  goal_pose_1.pose.position.z = 0;
  goal_pose_1.pose.orientation.x = 0;
  goal_pose_1.pose.orientation.y = 0;
  goal_pose_1.pose.orientation.z = 0;
  goal_pose_1.pose.orientation.w = 1;

  while (!pose_flag) {
    rclcpp::spin_some(odom_node_1);
    pose_publisher_->publish(goal_pose_1);
    RCLCPP_INFO(this->get_logger(), "Going back to base station.");
    rclcpp::sleep_for(500ms);
  }
}

/**
 * @brief Callback function for processing odometry messages during inspection.
 * 
 * @param odom_msg_i Shared pointer to the odometry message.
 */
void InspectorBot::inspectionCallback(const odom_pub::SharedPtr odom_msg_i) {
  if ((std::abs(static_cast<int>(odom_msg_i->pose.pose.position.x - goal_x_)) <
       0.5) &&
      (std::abs(static_cast<int>(odom_msg_i->pose.pose.position.y - goal_y_)) ==
       0.0)) {
    pose_flag = true;
    RCLCPP_INFO(this->get_logger(), "Now, Inspecting the location!");
  }
}

/**
 * @brief Callback function for processing odometry messages when returning to base.
 * 
 * @param odom_msg_r Shared pointer to the odometry message.
 */
void InspectorBot::continueInspectionCallback(
    const odom_pub::SharedPtr odom_msg_r) {
  if ((std::abs(static_cast<int>(odom_msg_r->pose.pose.position.x)) == 0) &&
      (std::abs(static_cast<int>(odom_msg_r->pose.pose.position.y)) == 0)) {
    pose_flag = true;

    RCLCPP_INFO(this->get_logger(),
                "Reached the base station. Inspection finished!");
  }
}
