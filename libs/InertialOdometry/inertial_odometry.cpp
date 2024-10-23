
/**
 * @file inertial_odometry.cpp
 * @author Kshitij Aggarwal
 * @brief C++ source file for Inertial Odometry class
 * @version 0.1
 * @date 2024-10-16
 *
 * @copyright Copyright (c) 2024
 *
 */

#include "inertial_odometry.hpp"

#include <eigen3/Eigen/src/Core/Matrix.h>

#include <vector>

Eigen::MatrixXf io::InertialOdometry::preintegrate_IMU_data(
    const std::vector<Eigen::Vector3f>& imu_data, const Eigen::Vector3f& bias) {
  // Implementation for IMU preintegration from on accelerometer and gyroscope
  // data
  Eigen::MatrixXf preintegrated_data;
  return preintegrated_data;
}

Eigen::Vector3f io::InertialOdometry::correct_bias(
    const Eigen::Vector3f& bias) {
  // Implementation of the bias correction logic
  Eigen::Vector3f corrected_bias = bias;  // Placeholder
  return corrected_bias;
}

Eigen::Matrix4f io::InertialOdometry::estimate_motion(
    const Eigen::MatrixXf& preintegrated_data) {
  // Implementation of the motion estimation from on preintegrated IMU data
  Eigen::Matrix4f motion = Eigen::Matrix4f::Identity();
  return motion;
}

void io::InertialOdometry::update_pose() {
  // Updateing the current pose (io_pose) based on the estimated motion
}
