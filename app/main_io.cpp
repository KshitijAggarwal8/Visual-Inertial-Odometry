/**
 * @file main.cpp
 * @author Apoorv Thapliyal
 * @brief C++ source file for main function
 * @version 0.1
 * @date 2024-10-22
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include "data_loader.hpp"  
#include "inertial_odometry.hpp"
#include "visual_odometry.hpp"
#include <cmath>
#include <iomanip>
#include <iostream>

 
int main()
{
    // Set precision for displaying floating point values
    std::cout << std::fixed << std::setprecision(6);

    // Create DataLoader object
    dl::DataLoader data_loader("indoor_forward_9_davis_with_gt");

    // Create InertialOdometry object
    io::InertialOdometry IO(Eigen::Matrix4d::Identity());

    int counter = 0;

    // Display IMU data
    while (true)
    { 
        // Get IMU data
        auto imu_data = data_loader.get_imu_data();
        
        // Check if end of file is reached
        if (std::get<0>(imu_data) == -1.0)
            break;
        
        // Extract IMU data
        double timestamp = std::get<0>(imu_data);

        // If timestamp is less than start time, continue
        if (timestamp < data_loader.start_gt_time)
            continue;
        // If timestamp is greater than finish time, break
        else if (timestamp > data_loader.finish_gt_time)
            break;
        
        // Extract IMU data
        Eigen::Vector3d angular_velocity = std::get<1>(imu_data);
        Eigen::Vector3d linear_acceleration = std::get<2>(imu_data); 

        // Update IMU pose
        IO.update_pose(linear_acceleration, angular_velocity);

        // Get IMU pose
        Eigen::Matrix4d io_pose = IO.get_pose();

        // Extract orientation from pose
        Eigen::Matrix3d io_rotation_matrix = io_pose.block<3,3>(0,0);

        // std::cout << "rotation: \n" << rotation << std::endl;
        // std::cout << "\n";

        // Convert to quaternion
        Eigen::Quaterniond q(io_rotation_matrix);

        // Convert to Euler angles 'xyz' between -pi to pi
        Eigen::Vector3d io_euler_angles = io_rotation_matrix.eulerAngles(0, 1, 2);
        // Convert to degrees
        io_euler_angles = io_euler_angles * 180 / M_PI;

        // std::cout << "roll: " << io_euler_angles(0) << std::endl;
        // std::cout << "pitch: " << io_euler_angles(1) << std::endl;
        // std::cout << "yaw: " << io_euler_angles(2) << std::endl;

        // Display IMU data
        std::cout << "x_gt: " << io_pose(0,3) << std::endl;
        std::cout << "y_gt: " << io_pose(1,3) << std::endl;
        std::cout << "z_gt: " << io_pose(2,3) << std::endl;
        std::cout << "qx_gt: " << q.x() << std::endl;
        std::cout << "qy_gt: " << q.y() << std::endl;
        std::cout << "qz_gt: " << q.z() << std::endl;
        std::cout << "qw_gt: " << q.w() << std::endl;
        std::cout << "\n";

        counter++;
    }    

    return 0;
}