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
#include <iostream>

/**
 * @brief Function to implement rodrigues formula for rotation matrix calculation
 * 
 * @param w 
 * @return Eigen::Matrix3d
 */
Eigen::Matrix3d io::InertialOdometry::rodrigues_formula(Eigen::Vector3d w){

    // Calculate angle of rotation within dt
    double angle = w.norm() * dt;

    if (angle == 0)
        return Eigen::Matrix3d::Identity();

    // Normalize the angular velocity vector to get the axis of rotation
    Eigen::Vector3d n_hat = w.normalized();

    // Calculate the skew symmetric matrix
    Eigen::Matrix3d K;
    K << 0,       -n_hat(2), n_hat(1),
         n_hat(2), 0,       -n_hat(0),
        -n_hat(1), n_hat(0), 0;
    
    Eigen::Matrix3d R = Eigen::Matrix3d::Identity() + sin(angle) * K + (1 - cos(angle)) * K * K;

    return R;
}

/**
 * @brief Function to return the current pose
 * 
 * @return Eigen::Matrix3d 
 */
Eigen::Matrix4d io::InertialOdometry::get_pose(){
    return io_pose;
}

/**
 * @brief Function to update the pose using accelerometer and gyroscope data
 * 
 * @param a 
 * @param w 
 */
void io::InertialOdometry::update_pose(Eigen::Vector3d a, Eigen::Vector3d w){

    // Convert IMU data to local frame
    Eigen::Matrix3d R = io_pose.block<3,3>(0,0);
    // std::cout << "rotation matrix: \n" << R << std::endl;
    a = R * a;
    w = R * w;

    // std::cout << "angular velocity in local frame: " << w.transpose() << std::endl;

    // Calculate rate of change of rotation matrix using rodrigues formula
    Eigen::Matrix3d r_dot = rodrigues_formula(w);

    // std::cout << "R_dot: \n" << r_dot << std::endl;

    // Update the rotation matrix io_pose directly
    io_pose.block<3, 3>(0, 0) = r_dot * io_pose.block<3, 3>(0, 0);
}

/**
 * @brief Construct a new io:: Inertial Odometry:: Inertial Odometry object
 * 
 */
io::InertialOdometry::InertialOdometry(Eigen::Matrix4d initial_pose){
    io_pose = initial_pose;
}

/**
 * @brief Destroy the io:: Inertial Odometry:: Inertial Odometry object
 * 
 */
io::InertialOdometry::~InertialOdometry(){
}


