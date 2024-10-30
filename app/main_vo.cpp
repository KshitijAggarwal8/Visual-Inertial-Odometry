/**
 * @file main.cpp
 * @author Apoorv Thapliyal
 * @brief C++ source file for main function
 * @version 0.1
 * @date 2024-10-22
 *
 * @copyright Copyright (c) 2024
 *
 */

#include <cmath>
#include <iomanip>
#include <iostream>

#include "data_loader.hpp"
#include "inertial_odometry.hpp"
#include "visual_odometry.hpp"

int main() {
  // Set precision for displaying floating point values
  std::cout << std::fixed << std::setprecision(6);

  // Create DataLoader object
  dl::DataLoader data_loader("indoor_forward_9_davis_with_gt");

  // Create VisualOdometry object
  vo::VisualOdometry visual_odometry(Eigen::Matrix4d::Identity());

  int counter = 0;

  // Display VO data
  while (true) {
    // Get Image data
    auto image_data = data_loader.get_image_data();

    // Check if end of file is reached
    if (std::get<0>(image_data) == -1.0) break;

    // Get data timestamp
    double timestamp = std::get<0>(image_data);

    // If timestamp is less than start time, continue
    if (timestamp < data_loader.start_gt_time) continue;
    // If timestamp is greater than finish time, break
    else if (timestamp > data_loader.finish_gt_time)
      break;

    // Extract Image data
    cv::Mat image = std::get<1>(image_data);
    std::string image_path = std::get<2>(image_data);

    // Update pose using Visual Odometry
    visual_odometry.update_pose(image);

    // Get VO pose
    Eigen::Matrix4d vo_pose = visual_odometry.get_pose();

    // Extract orientation from pose
    Eigen::Matrix3d R = vo_pose.block<3, 3>(0, 0);

    // Convert rotation matrix to Euler angles
    Eigen::Vector3d vo_euler_angles = R.eulerAngles(0, 1, 2);

    vo_euler_angles = vo_euler_angles * 180 / M_PI;

    // Display VO data
    std::cout << "x: " << vo_pose(0, 3) << std::endl;
    std::cout << "y: " << vo_pose(1, 3) << std::endl;
    std::cout << "z: " << vo_pose(2, 3) << std::endl;
    std::cout << "roll: " << vo_euler_angles(0) << std::endl;
    std::cout << "pitch: " << vo_euler_angles(1) << std::endl;
    std::cout << "yaw: " << vo_euler_angles(2) << std::endl;
    std::cout << "\n";

    counter++;
  }

  std::cout << "Total Images: " << counter << std::endl;

  return 0;
}