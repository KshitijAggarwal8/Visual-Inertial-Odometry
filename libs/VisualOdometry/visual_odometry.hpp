/**
 * @file visual_odometry.hpp
 * @author Apoorv Thapliyal
 * @brief C++ header file for Visual Odometry class
 * @version 0.1
 * @date 2024-10-16
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once

// #include <eigen3/Eigen/src/Core/Matrix.h>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <vector>

#include "opencv2/core/mat.hpp"
#include "opencv2/features2d.hpp"

namespace vo {

/**
 * @brief Visual Odometry class
 *
 */
class VisualOdometry {
 private:
  /**
   * @brief Current image keypoints
   *
   */
  std::vector<cv::KeyPoint> kp_curr;

  /**
   * @brief Current image descriptors
   *
   */
  cv::Mat des_curr;

  /**
   * @brief Previous image keypoints
   *
   */
  std::vector<cv::KeyPoint> kp_prev;

  /**
   * @brief Previous image descriptors
   *
   */
  cv::Mat des_prev;

  /**
   * @brief Create ORB detector
   *
   */
  cv::Ptr<cv::ORB> orb_descriptor = cv::ORB::create();

  /**
   * @brief Create FLANN matcher
   *
   */
  cv::FlannBasedMatcher flann_matcher;

  /**
   * @brief Camera intrinsics matrix
   *
   */
  cv::Mat camera_intrinsics;

  /**
   * @brief Camera distortion coefficients
   *
   */
  cv::Mat distortion_coefficients;

  /**
   * @brief Image width
   *
   */
  int image_width;

  /**
   * @brief Image height
   *
   */
  int image_height;

  /**
   * @brief Camera matrix
   *
   */
  cv::Mat new_camera_matrix;

  /**
   * @brief Initial pose
   *
   */
  Eigen::Matrix4d vo_pose;

 public:
  /**
   * @brief Construct a new Visual Odometry object
   *
   */
  VisualOdometry(Eigen::Matrix4d initial_pose);

  /**
   * @brief Destroy the Visual Odometry object
   *
   */
  ~VisualOdometry();

  /**
   * @brief Function to update the pose using visual odometry
   *
   */
  void update_pose(cv::Mat image);

  /**
   * @brief Function to return the current pose
   *
   */
  Eigen::Matrix4d get_pose();
};

}  // namespace vo