/**
 * @file ArucoDetect.cpp
 * @author Sai Surya Sriramoju
 * @brief 
 * @version 0.1
 * @date 2023-12-20
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include "ArucoDetect.hpp"

ArUcoDetectionNode::ArUcoDetectionNode() : Node("aruco_detection_node") {
  image_subscriber_ = create_subscription<sensor_msgs::msg::Image>(
      "/camera/image_raw", 10, std::bind(&ArUcoDetectionNode::imageCallback,
      this, std::placeholders::_1));

  marker_publisher_ = create_publisher<geometry_msgs::msg::PoseStamped>
  ("/aruco_marker_pose", 10);

  dictionary_ = cv::aruco::getPredefinedDictionary
  (cv::aruco::DICT_ARUCO_ORIGINAL);
  marker_length_ = 0.1;
  camera_matrix_ = (cv::Mat1d(3, 3)
  << 1696.802685832259, 0.0, 960.5, 0.0, 1696.802685832259, 540.5, 0, 0, 1);
  distortion_coefficients_ = (cv::Mat1d(1, 5) << 0.0, 0.0, 0.0, 0.0, 0.0);
}

void ArUcoDetectionNode::imageCallback
  (const sensor_msgs::msg::Image::SharedPtr image_msg) {
  try {
    cv::Mat cv_image = cv_bridge::toCvCopy
    (image_msg, sensor_msgs::image_encodings::BGR8)->image;
    std::vector<int> marker_ids;
    std::vector<std::vector<cv::Point2f>> marker_corners, rejectedCandidates;
    cv::Ptr<cv::aruco::DetectorParameters>
    detectorParams = cv::aruco::DetectorParameters::create();
    cv::aruco::detectMarkers
    (cv_image, dictionary_, marker_corners, marker_ids,
    detectorParams, rejectedCandidates);

    RCLCPP_INFO(get_logger(), "size: %ld", marker_ids.size());

    for (size_t i = 0; i < marker_ids.size(); ++i) {
      cv::Vec3d rvec, tvec;
      cv::aruco::estimatePoseSingleMarkers(marker_corners[i],
      marker_length_, camera_matrix_, distortion_coefficients_, rvec, tvec);

      RCLCPP_INFO(get_logger(),
      "Detected ArUco marker %d at position (%f, %f, %f)",
      marker_ids[i], tvec[0], tvec[1], tvec[2]);

      geometry_msgs::msg::PoseStamped marker_pose;
      marker_pose.header = image_msg->header;
      marker_pose.pose.position.x = tvec[0];
      marker_pose.pose.position.y = tvec[1];
      marker_pose.pose.position.z = tvec[2];

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
