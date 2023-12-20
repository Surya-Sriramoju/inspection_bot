// Copyright 2023 Tarun Trilokesh

/**
 * @file InspectorBot.cpp
 * @author Tarun Trilokesh
 * @author Sai Surya Sriramoju
 * @date 12/12/2023
 * @version 1.0
 * 
 * @brief InspectorBot class implementation.
 *
 * Implementation of the InspectorBot class, which handles autonomous navigation
 * and inspection tasks.
 */

#include "InspectorBot.hpp"


InspectorBot::InspectorBot() : Node("inspector_bot") {
    current_position.position.x = 0.0;
    next_position.position.x = 0.0;
}

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

void InspectorBot::setLoc(float x, float y) {
  goal_x_ = x;
  goal_y_ = y;
}

float InspectorBot::getLocx() { return goal_x_; }

float InspectorBot::getLocy() { return goal_y_; }

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

void InspectorBot::inspectionCallback(const odom_pub::SharedPtr odom_msg_i) {
  if ((std::abs(static_cast<int>(odom_msg_i->pose.pose.position.x - goal_x_)) <
       0.5) &&
      (std::abs(static_cast<int>(odom_msg_i->pose.pose.position.y - goal_y_)) ==
       0.0)) {
    pose_flag = true;
    RCLCPP_INFO(this->get_logger(), "Now, Inspecting the location!");
  }
}


void InspectorBot::continueInspectionCallback(
    const odom_pub::SharedPtr odom_msg_r) {
  if ((std::abs(static_cast<int>(odom_msg_r->pose.pose.position.x)) == 0) &&
      (std::abs(static_cast<int>(odom_msg_r->pose.pose.position.y)) == 0)) {
    pose_flag = true;

    RCLCPP_INFO(this->get_logger(),
                "Reached the base station. Inspection finished!");
  }
}
