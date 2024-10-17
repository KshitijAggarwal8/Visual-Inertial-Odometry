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

#ifndef INERTIAL_ODOMETRY_HPP
#define INERTIAL_ODOMETRY_HPP

#pragma once

#include <iostream>
#include <eigen3/Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <Eigen/Core> 
#include <vector>


namespace vo {

/**
 * @brief Inertial Odometry class
 * 
 */
class InertialOdometry {

private:
    /**
     * @brief Accelerometer data
     * 
     */
    std::vector<Eigen::Vector3f> accelerometer_data;

    /**
     * @brief Gyroscope data
     * 
     */
    std::vector<Eigen::Vector3f> gyroscope_data;

    /**
     * @brief IMU bias
     * 
     */
    Eigen::Vector3f bias;

    /**
     * @brief State vector
     * 
     */
    Eigen::VectorXf state_vector;

public:
    /**
     * @brief Inertial Odometry final pose
     * 
     */
    Eigen::Matrix4f io_pose;

    /**
     * @brief Construct a new Inertial Odometry object
     * 
     * @param bias Initial bias for the IMU data
     */
    InertialOdometry(const Eigen::Vector3f& bias);

    /**
     * @brief Destroy the Inertial Odometry object
     * 
     */
    ~InertialOdometry();

    /**
     * @brief Preintegrate IMU data for motion estimation
     * 
     * @param imu_data The IMU data (acceleration and gyro)
     * @param bias The bias to be applied to the IMU data
     * @return Eigen::MatrixXf Preintegrated data
     */
    Eigen::MatrixXf preintegrate_IMU_data(const std::vector<Eigen::Vector3f>& imu_data, 
                                          const Eigen::Vector3f& bias);

    /**
     * @brief Correct the IMU bias
     * 
     * @param bias The current IMU bias
     * @return Eigen::Vector3f Corrected bias
     */
    Eigen::Vector3f correct_bias(const Eigen::Vector3f& bias);

    /**
     * @brief Estimate motion based on preintegrated IMU data
     * 
     * @param preintegrated_data The preintegrated IMU data
     * @return Eigen::Matrix4f Estimated motion
     */
    Eigen::Matrix4f estimate_motion(const Eigen::MatrixXf& preintegrated_data);

    /**
     * @brief Update the current pose of the system
     * 
     */
    void update_pose();
};

#endif // INERTIAL_ODOMETRY_HPP
}