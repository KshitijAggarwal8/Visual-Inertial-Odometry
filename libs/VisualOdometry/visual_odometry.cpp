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

std::vector<cv::KeyPoint> vo::VisualOdometry::compute_keypoints(cv::Mat frame) {
    // Placeholder for the keypoints

    return keypoints;
}

std::vector<cv::DMatch> vo::VisualOdometry::compute_matches(std::vector<cv::KeyPoint> prev_keypoints, std::vector<cv::KeyPoint> curr_keypoints) {
    // Placeholder for the matches

    return matches;
}

Eigen::Matrix4f vo::VisualOdometry::estimate_pose(std::vector<cv::DMatch> matches,
                                      Eigen::Matrix3f camera_intrinsics) {
    // Placeholder for the pose estimation

    return Eigen::Matrix4f::Identity();
}

Eigen::Matrix4f vo::VisualOdometry::track_motion(cv::Mat prev_frame, cv::Mat curr_frame){
    // Placeholder for the motion tracking

    return Eigen::Matrix4f::Identity();
}

