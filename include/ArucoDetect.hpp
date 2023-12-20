/**
 * @file InspectorBot.hpp
 * @author Tarun Trilokesh
 * @author Sai Surya Sriramoju
 * @date 12/18/2023
 * @version 1.0
 * 
 * @brief InspectorBot class declaration.
 * @copyright Copyright (c) 2023
 * Declaration of the InspectorBot class, which handles autonomous navigation
 * and inspection tasks.
 */


#pragma once

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "opencv2/opencv.hpp"
#include "opencv2/aruco.hpp"
#include "cv_bridge/cv_bridge.h"

class ArUcoDetectionNode : public rclcpp::Node {
 public:
  ArUcoDetectionNode();

 private:
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr image_msg);

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscriber_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr
  marker_publisher_;

  cv::Ptr<cv::aruco::Dictionary> dictionary_;
  double marker_length_;
  cv::Mat camera_matrix_;
  cv::Mat distortion_coefficients_;
};


