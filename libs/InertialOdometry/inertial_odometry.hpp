/**
 * @file inertial_odometry.hpp
 * @author Kshitij Aggarwal
 * @brief C++ header file for inertial Odometry class
 * @version 0.1
 * @date 2024-10-16
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <vector>

/**
 * @brief Inertial Odometry namespace
 *
 */
namespace io {

/**
 * @brief Inertial Odometry class
 *
 */
class InertialOdometry {
 private:
  /**
   * @brief Vector to store accelerometer data [ax, ay, az]
   *
   */
  Eigen::Vector3d accelerometer_data;

  /**
   * @brief Vector to store gyroscope data [wx, wy, wz]
   *
   */
  Eigen::Vector3d gyroscope_data;

  /**
   * @brief Transformation matrix of the current pose
   *
   */
  Eigen::Matrix4d io_pose;

  /**
   * @brief Sampling time for the IMU data
   *
   */
  float dt = 0.001;

 public:
  /**
   * @brief Construct a new Inertial Odometry object
   *
   * @param initial_pose
   */
  InertialOdometry(Eigen::Matrix4d initial_pose);

  /**
   * @brief Destroy the Inertial Odometry object
   *
   */
  ~InertialOdometry();

  /**
   * @brief Function to integrate the IMU data
   *
   * @param a: accelerometer data in the imu frame
   * @param w: gyroscope data in the imu frame
   */
  void update_pose(Eigen::Vector3d a, Eigen::Vector3d w);

  /**
   * @brief Function to calculate the rate of change of rotation matrix using
   * rodrigues formula
   *
   * @param w: angular velocity vector
   * @return Eigen::Matrix3d: rate of change of rotation matrix
   */
  Eigen::Matrix3d rodrigues_formula(Eigen::Vector3d w);

  /**
   * @brief Get the pose
   *
   * @param pose
   */
  Eigen::Matrix4d get_pose();
};

}  // namespace io