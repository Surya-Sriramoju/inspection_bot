// Copyright 2023 Sai Surya Sriramoju

/**
 * @file ArucoDetect.cpp
 * @author Sai Surya Sriramoju
 * @brief Implementation of the ArUcoDetectionNode class.
 * @date 12/20/2023
 * @version 1.0
 * 
 * This file contains the implementation of the ArUcoDetectionNode class, which
 * is responsible for detecting ArUco markers in images and publishing their poses.
 */

#include "ArucoDetect.hpp"

/**
 * @brief Construct a new ArUco Detection Node object.
 * 
 * Initializes the ROS2 node, sets up the image subscriber and marker publisher,
 * and configures the ArUco marker detection parameters.
 */
ArUcoDetectionNode::ArUcoDetectionNode() : Node("aruco_detection_node") {
  // Subscribe to image data
  image_subscriber_ = create_subscription<sensor_msgs::msg::Image>(
      "/camera/image_raw", 10, std::bind(&ArUcoDetectionNode::imageCallback,
      this, std::placeholders::_1));

  // Publisher for marker pose
  marker_publisher_ = create_publisher<geometry_msgs::msg::PoseStamped>
  ("/aruco_marker_pose", 10);

  // ArUco marker dictionary and detection parameters
  dictionary_ = cv::aruco::getPredefinedDictionary
  (cv::aruco::DICT_ARUCO_ORIGINAL);
  marker_length_ = 0.1;
  // Camera intrinsic parameters
  camera_matrix_ = (cv::Mat1d(3, 3)
  << 1696.802685832259, 0.0, 960.5, 0.0, 1696.802685832259, 540.5, 0, 0, 1);
  distortion_coefficients_ = (cv::Mat1d(1, 5) << 0.0, 0.0, 0.0, 0.0, 0.0);
}

/**
 * @brief Callback function for image data.
 * 
 * This function is called when a new image message is received. It performs
 * ArUco marker detection on the image and publishes the pose of any detected markers.
 * 
 * @param image_msg The received image message.
 */
void ArUcoDetectionNode::imageCallback
  (const sensor_msgs::msg::Image::SharedPtr image_msg) {
  try {
    // Convert ROS image message to OpenCV image
    cv::Mat cv_image = cv_bridge::toCvCopy
    (image_msg, sensor_msgs::image_encodings::BGR8)->image;

    // Marker detection
    std::vector<int> marker_ids;
    std::vector<std::vector<cv::Point2f>> marker_corners, rejectedCandidates;
    cv::Ptr<cv::aruco::DetectorParameters>
    detectorParams = cv::aruco::DetectorParameters::create();
    cv::aruco::detectMarkers
    (cv_image, dictionary_, marker_corners, marker_ids,
    detectorParams, rejectedCandidates);

    RCLCPP_INFO(get_logger(), "size: %ld", marker_ids.size());

    // Pose estimation for each detected marker
    for (size_t i = 0; i < marker_ids.size(); ++i) {
      cv::Vec3d rvec, tvec;
      cv::aruco::estimatePoseSingleMarkers(marker_corners[i],
      marker_length_, camera_matrix_, distortion_coefficients_, rvec, tvec);

      RCLCPP_INFO(get_logger(),
      "Detected ArUco marker %d at position (%f, %f, %f)",
      marker_ids[i], tvec[0], tvec[1], tvec[2]);

      // Publish marker pose
      geometry_msgs::msg::PoseStamped marker_pose;
      marker_pose.header = image_msg->header;
      marker_pose.pose.position.x = tvec[0];
      marker_pose.pose.position.y = tvec[1];
      marker_pose.pose.position.z = tvec[2];

      // Convert rotation vector to quaternion
      cv::Mat rotation_matrix;
      cv::Rodrigues(rvec, rotation_matrix);
      cv::Mat quaternion;
      cv::decomposeProjectionMatrix
      (rotation_matrix, cv::noArray(), cv::noArray(),
      cv::noArray(), quaternion);
      marker_pose.pose.orientation.x = quaternion.at<double>(0);
      marker_pose.pose.orientation.y = quaternion.at<double>(1);
      marker_pose.pose.orientation.z = quaternion.at<double>(2);
      marker_pose.pose.orientation.w = quaternion.at<double>(3);

      marker_publisher_->publish(marker_pose);
    }
  }
  catch (cv_bridge::Exception &e) {
    RCLCPP_ERROR(get_logger(), "Error converting image message: %s", e.what());
  }
}
