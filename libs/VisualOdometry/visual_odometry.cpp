/**
 * @file visual_odometry.cpp
 * @author Apoorv Thapliyal
 * @brief C++ source file for Visual Odometry class
 * @version 0.1
 * @date 2024-10-16
 *
 * @copyright Copyright (c) 2024
 *
 */

#include "visual_odometry.hpp"

/**
 * @brief Construct a new vo::Visual Odometry::Visual Odometry object
 *
 * @param initial_pose
 */
vo::VisualOdometry::VisualOdometry(Eigen::Matrix4d initial_pose) {
  // Set initial pose
  vo_pose = initial_pose;

  // Initialize camera intrinsics (Taken from dataset)
  // [172.98992850734132, 0, 163.33639726024606],
  // [0, 172.98303181090185, 134.99537889030861],
  // [0, 0, 1]
  camera_intrinsics =
      (cv::Mat_<double>(3, 3) << 172.98992850734132, 0, 163.33639726024606, 0,
       172.98303181090185, 134.99537889030861, 0, 0, 1);

  // Initialize distortion coefficients (Taken from dataset)
  // [-0.027576733308582076, -0.006593578674675004, 0.0008566938165177085,
  // -0.00030899587045247486]
  distortion_coefficients =
      (cv::Mat_<double>(1, 4) << -0.027576733308582076, -0.006593578674675004,
       0.0008566938165177085, -0.00030899587045247486);

  // Initialize image width and height
  image_width = 346;
  image_height = 260;

  // Initialize optimal camera matrix
  new_camera_matrix =
      cv::getOptimalNewCameraMatrix(camera_intrinsics, distortion_coefficients,
                                    cv::Size(image_width, image_height), 1,
                                    cv::Size(image_width, image_height), 0);
}

/**
 * @brief Destroy the vo::Visual Odometry::Visual Odometry object
 *
 */
vo::VisualOdometry::~VisualOdometry() {
  // Destructor
}

/**
 * @brief Function to return the current pose
 *
 */
Eigen::Matrix4d vo::VisualOdometry::get_pose() { return vo_pose; }

/**
 * @brief Function to update the pose using visual odometry
 *
 * @param image
 */
void vo::VisualOdometry::update_pose(cv::Mat image) {
  // Undistort the image
  cv::Mat undistorted_image;
  cv::undistort(image, undistorted_image, camera_intrinsics,
                distortion_coefficients, new_camera_matrix);

  // Get keypoints and descriptors for the current image
  orb_descriptor->detectAndCompute(undistorted_image, cv::noArray(), kp_curr,
                                   des_curr);

  if (kp_prev.size() == 0) {
    kp_prev = kp_curr;
    des_prev = des_curr;
    return;
  }

  // Ensure descriptors are CV_32F before using FLANN matcher
  if (des_prev.type() != CV_32F) des_prev.convertTo(des_prev, CV_32F);
  if (des_curr.type() != CV_32F) des_curr.convertTo(des_curr, CV_32F);

  // Prepare a vector to hold matches for each descriptor
  std::vector<std::vector<cv::DMatch>> matches;

  // Perform KNN matching
  flann_matcher.knnMatch(des_prev, des_curr, matches, 2);

  // Find good matches using Lowe's ratio test
  std::vector<cv::DMatch> good_matches;
  for (int i = 0; i < matches.size(); i++) {
    if (matches[i][0].distance < 0.78 * matches[i][1].distance)
      good_matches.push_back(matches[i][0]);
  }

  // Get matched keypoints
  std::vector<cv::Point2f> matched_kp_prev, matched_kp_curr;
  for (int i = 0; i < good_matches.size(); i++) {
    matched_kp_prev.push_back(kp_prev[good_matches[i].queryIdx].pt);
    matched_kp_curr.push_back(kp_curr[good_matches[i].trainIdx].pt);
  }

  // Calculate essential matrix
  cv::Mat E = cv::findEssentialMat(matched_kp_curr, matched_kp_prev,
                                   camera_intrinsics, cv::RANSAC, 0.999, 1.0);

  // Recover pose from essential matrix
  cv::Mat R, t;
  cv::recoverPose(E, matched_kp_curr, matched_kp_prev, camera_intrinsics, R, t);

  // Convert rotation matrix to Eigen matrix
  Eigen::Matrix3d R_eigen;
  Eigen::Vector3d t_eigen;

  for (int i = 0; i < 3; i++) {
    t_eigen(i) = t.at<double>(i);
    for (int j = 0; j < 3; j++) R_eigen(i, j) = R.at<double>(i, j);
  }

  // Make a homogeneous transformation matrix
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  T.block<3, 3>(0, 0) = R_eigen;
  T.block<3, 1>(0, 3) = t_eigen;

  // Update the pose
  vo_pose = vo_pose * T;

  // Update previous keypoints and descriptors
  kp_prev = kp_curr;
  des_prev = des_curr;

  return;
}
