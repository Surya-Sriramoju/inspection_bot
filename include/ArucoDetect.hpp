// Copyright 2023 Tarun Trilokesh

/**
 * @file ArucoDetect.hpp
 * @author Tarun Trilokesh
 * @author Sai Surya Sriramoju
 * @date 12/20/2023
 * @version 2.0
 * 
 * @brief Declaration of the ArucoDetect class.
 * 
 * The ArucoDetect class handles autonomous navigation and inspection tasks,
 * particularly focusing on ArUco marker detection using ROS2 and OpenCV.
 */

#pragma once

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "opencv2/opencv.hpp"
#include "opencv2/aruco.hpp"
#include "cv_bridge/cv_bridge.h"

/**
 * @class ArUcoDetectionNode
 * @brief The ArUcoDetectionNode class is a ROS2 node for detecting ArUco markers.
 * 
 * This class subscribes to image data, detects ArUco markers in the images,
 * and publishes the pose of detected markers. It uses OpenCV and cv_bridge
 * for image processing and marker detection.
 */
class ArUcoDetectionNode : public rclcpp::Node {
 public:
  /**
   * @brief Construct a new ArUco Detection Node object.
   * 
   * Initializes the ROS2 node, sets up the image subscriber and marker publisher,
   * and configures the ArUco marker detection parameters.
   */
  ArUcoDetectionNode();

 private:
  /**
   * @brief Callback function for image data.
   * 
   * This function is called when a new image message is received. It performs
   * ArUco marker detection on the image and publishes the pose of any detected markers.
   * 
   * @param image_msg The received image message.
   */
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr image_msg);

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscriber_;
  rclcpp::Publisher<geometry_msgs::msg::
                              PoseStamped>::SharedPtr marker_publisher_;

  cv::Ptr<cv::aruco::Dictionary> dictionary_;
  double marker_length_;
  cv::Mat camera_matrix_;
  cv::Mat distortion_coefficients_;
};
