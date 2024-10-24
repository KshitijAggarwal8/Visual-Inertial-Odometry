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
     * @brief Get the IMU data from the dataset
     * 
     * @return std::tuple<double, Eigen::Vector3d, Eigen::Vector3d> Tuple containing timestamp, angular velocity and linear acceleration
     */
    std::tuple<long double, Eigen::Vector3d, Eigen::Vector3d> get_imu_data();

    /**
     * @brief Function to parse the ground truth data
     * 
     */
    void parse_gt_data();
};

};
