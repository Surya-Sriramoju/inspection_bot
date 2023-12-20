 /**
 * @file ArucoMain.cpp
 * @author Sai Surya Sriramoju
 * @brief main file for launching aruco detect
 * @version 0.1
 * @date 2023-12-18
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include "ArucoDetect.hpp"

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ArUcoDetectionNode>());
  rclcpp::shutdown();
}
