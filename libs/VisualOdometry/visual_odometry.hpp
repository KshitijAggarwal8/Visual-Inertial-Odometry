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

#pragma oncea

#include <iostream>
#include <eigen3/Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <vector>


namespace vo {

/**
 * @brief Visual Odometry class
 * 
 */
class VisualOdometry {

    private: 
    /**
     * @brief Camera intrinsics matrix
     * 
     */
    Eigen::Matrix3f camera_intrinsics;
    
    /**
     * @brief Previous frame
     * 
     */
    cv::Mat prev_frame;
    
    /**
     * @brief Current frame
     * 
     */
    cv::Mat curr_frame;
    
    /**
     * @brief Keypoints in the frame
     * 
     */
    std::vector<cv::KeyPoint> keypoints;

    /**
     * @brief Matches between the previous and current frame
     * 
     */
    std::vector<cv::DMatch> matches;

    public:

        /**
         * @brief Visual Odometry final pose
         * 
         */
        Eigen::Matrix4f vo_pose;

        /**
         * @brief Construct a new Visual Odometry object
         * 
         * @param camera_intrinsics 
         */
        VisualOdometry(Eigen::Matrix3f camera_intrinsics);

        /**
         * @brief Destroy the Visual Odometry object
         * 
         */

        ~VisualOdometry();

        /**
         * @brief Compute keypoints in the frame
         * 
         * @param frame 
         * @return std::vector<cv::KeyPoint> 
         */
        std::vector<cv::KeyPoint> compute_keypoints(cv::Mat frame);

        /**
         * @brief Match keypoints between the previous and current frame
         * 
         * @param prev_keypoints 
         * @param curr_keypoints 
         * @return std::vector<cv::DMatch> 
         */
        std::vector<cv::DMatch> compute_matches(std::vector<cv::KeyPoint> prev_keypoints, std::vector<cv::KeyPoint> curr_keypoints);

        /**
         * @brief Estimate the pose of the camera
         * 
         * @param matches 
         * @param camera_intrinsics 
         * @return Eigen::Matrix4f 
         */
        Eigen::Matrix4f estimate_pose(std::vector<cv::DMatch> matches,
                                      Eigen::Matrix3f camera_intrinsics);
        /**
         * @brief Track the motion of the camera
         * 
         * @param prev_frame 
         * @param curr_frame 
         * @return Eigen::Matrix4f 
         */
        Eigen::Matrix4f track_motion(cv::Mat prev_frame, cv::Mat curr_frame);

};


}