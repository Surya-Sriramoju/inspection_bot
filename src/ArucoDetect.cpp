/**
 * @file ArucoDetect.cpp
 * @author Sai Surya Sriramoju
 * @brief Subscribes to the aruco marker
 * @version 0.1
 * @date 2023-12-18
 * 
 * @copyright Copyright (c) 2023
 * 
 */

 #include "ArucoDetect.hpp"

 ArucoDetect::ArucoDetect() : Node("aruco_detect") {
  auto aruco_detection_ = this->create_subscription<ARUCO_TYPE>(
      "/aruco_markers", 10, std::bind(&ArucoDetect::arucoCallback, this, _1));
}

void ArucoDetect::arucoCallback(const ARUCO_TYPE::SharedPtr aruco_msg) {
  RCLCPP_INFO_STREAM(this->get_logger(),
                     "Detect danger at:" << aruco_msg->position.x);
}