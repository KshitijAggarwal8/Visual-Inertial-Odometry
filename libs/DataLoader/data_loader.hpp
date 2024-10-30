/**
 * @file data_loader.hpp
 * @author Apoorv Thapliyal
 * @brief C++ header file for DataLoader class
 * @version 0.1
 * @date 2024-10-22
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#pragma once

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/src/Core/Matrix.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/mat.hpp>

/**
 * @brief Namespace for DataLoader class
 * 
 */
namespace dl{

/**
 * @brief DataLoader class for loading data from dataset
 * 
 */
class DataLoader {

private:

    /**
    * @brief File stream object for IMU data
    * 
    */
    std::ifstream imu_file;

    /**
     * @brief File stream object for ground truth data
     * 
     */
    std::ifstream gt_file;

    /**
     * @brief File stream object for image data
     * 
     */
    std::ifstream image_file;

    /**
     * @brief String to store the dataset location
     * 
     */
    std::string dataset_path;

    /**
     * @brief Track which image is being read
     * 
     */
     size_t current_image_index;

     /**
      * @brief Flag to track if we've found our first valid image
      * 
      */
    bool first_image_found;

    /**
     * @brief Store first valid timestamp
     * 
     */
    double first_valid_timestamp;

public:

    /**
     * @brief Ground truth position vectors
     * 
     */
    std::vector<long double> x_gt, y_gt, z_gt;

    /**
     * @brief Ground truth orientation vectors
     * 
     */
    std::vector<long double> qx_gt, qy_gt, qz_gt, qw_gt;

    /**
     * @brief Start time of ground truth data
     * 
     */
    double start_gt_time;

    /**
     * @brief Finish time of ground truth data
     * 
     */
    double finish_gt_time;

    /**
     * @brief Construct a new DataLoader object
     * 
     * @param dataset_location Path to the dataset
     */
    DataLoader(const std::string& dataset_location);

    /**
     * @brief Destroy the DataLoader object
     * 
     */
    ~DataLoader();

    /**
     * @brief Function to get the IMU data from the dataset
     * 
     * @return std::tuple<double, Eigen::Vector3d, Eigen::Vector3d> Tuple containing timestamp, angular velocity and linear acceleration
     */
    std::tuple<long double, Eigen::Vector3d, Eigen::Vector3d> get_imu_data();

    /**
     * @brief Function to parse the ground truth data
     * 
     */
    void parse_gt_data();

    /**
     * @brief Function to get the image data from the dataset
     * 
     */
     std::tuple<double, cv::Mat, std::string> get_image_data();
};

};
