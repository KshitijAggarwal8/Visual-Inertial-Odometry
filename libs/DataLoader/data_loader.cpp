/**
 * @file data_loader.cpp
 * @author Apoorv Thapliyal
 * @brief C++ source file for DataLoader class
 * @version 0.1
 * @date 2024-10-22
 *
 * @copyright Copyright (c) 2024
 *
 */

#include "data_loader.hpp"

/**
 * @brief Construct a new dl::Data Loader::Data Loader object
 *
 * @param dataset_location
 */
dl::DataLoader::DataLoader(const std::string& dataset_location) {
  // Save the dataset path
  dataset_path = dataset_location;

  // Initialize the image index
  current_image_index = 0;

  // Initialize the flag to track if we've found our first valid image
  first_image_found = false;

  // Initialize the first valid timestamp
  first_valid_timestamp = 0.0;

  // Open the IMU file
  std::string imu_file_path = dataset_location + "/imu.txt";
  imu_file.open(imu_file_path);
  if (!imu_file.is_open()) {
    std::cerr << "Error opening file: " << imu_file_path << std::endl;
  }
  // Skip the header line
  std::string header;
  std::getline(imu_file, header);

  // Open the ground truth file
  std::string gt_file_path = dataset_location + "/groundtruth.txt";
  gt_file.open(gt_file_path);
  if (!gt_file.is_open()) {
    std::cerr << "Error opening file: " << gt_file_path << std::endl;
  }
  // Skip the header line
  std::getline(gt_file, header);

  // Add this in your constructor along with the other file openings:
  std::string image_file_path = dataset_location + "/images.txt";
  image_file.open(image_file_path);
  if (!image_file.is_open()) {
    std::cerr << "Error opening file: " << image_file_path << std::endl;
  }
  // Skip the header line if needed
  std::getline(image_file, header);

  // Parse the ground truth data
  parse_gt_data();
}

/**
 * @brief Destroy the dl::Data Loader::Data Loader object
 *
 */
dl::DataLoader::~DataLoader() {
  if (imu_file.is_open()) imu_file.close();

  if (gt_file.is_open()) gt_file.close();

  if (image_file.is_open()) image_file.close();
}

/**
 * @brief Function to get IMU data from the dataset
 *
 * @return std::tuple<double, Eigen::Vector3d, Eigen::Vector3d>
 */
std::tuple<long double, Eigen::Vector3d, Eigen::Vector3d>
dl::DataLoader::get_imu_data() {
  if (imu_file.eof())
    return {-1.0, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};

  std::string line;
  if (std::getline(imu_file, line)) {
    std::istringstream iss(line);
    long double id;  // To capture the ID value but not use it
    long double timestamp;
    long double ang_vel_x, ang_vel_y, ang_vel_z;
    long double lin_acc_x, lin_acc_y, lin_acc_z;

    // Read ID and timestamp, then the rest of the values
    if (iss >> id >> timestamp >> ang_vel_x >> ang_vel_y >> ang_vel_z >>
        lin_acc_x >> lin_acc_y >> lin_acc_z) {
      Eigen::Vector3d angular_velocity(ang_vel_x, ang_vel_y, ang_vel_z);
      Eigen::Vector3d linear_acceleration(lin_acc_x, lin_acc_y, lin_acc_z);
      return {timestamp, angular_velocity, linear_acceleration};
    }
  }

  return {-1.0, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};
}

/**
 * @brief Function to parse the ground truth data
 *
 */
void dl::DataLoader::parse_gt_data() {
  std::string line;
  bool first_line = true;

  while (std::getline(gt_file, line)) {
    if (line[0] == '#') continue;  // Skip comments

    std::istringstream iss(line);
    long double timestamp, tx, ty, tz, qx, qy, qz, qw;

    if (iss >> timestamp >> tx >> ty >> tz >> qx >> qy >> qz >> qw) {
      // Store the values in respective vectors
      if (first_line) {
        start_gt_time = timestamp;
        first_line = false;
      }
      finish_gt_time = timestamp;

      x_gt.push_back(tx);
      y_gt.push_back(ty);
      z_gt.push_back(tz);
      qx_gt.push_back(qx);
      qy_gt.push_back(qy);
      qz_gt.push_back(qz);
      qw_gt.push_back(qw);
    }
  }

  // Shift x, y, z to start at origin
  long double x_offset = x_gt[0];
  long double y_offset = y_gt[0];
  long double z_offset = z_gt[0];

  for (size_t i = 0; i < x_gt.size(); ++i) {
    x_gt[i] -= x_offset;
    y_gt[i] -= y_offset;
    z_gt[i] -= z_offset;
  }

  // Update quaternion to un-rotate
  Eigen::Quaterniond initial_quaternion_orientation(qw_gt[0], qx_gt[0],
                                                    qy_gt[0], qz_gt[0]);

  // Make the quaternion a rotation matrix
  Eigen::Matrix3d initial_rotation_matrix =
      initial_quaternion_orientation.toRotationMatrix();

  // Un-rotate the x, y, z values
  for (size_t i = 0; i < x_gt.size(); ++i) {
    Eigen::Vector3d point(x_gt[i], y_gt[i], z_gt[i]);
    Eigen::Vector3d unrotated_point =
        initial_rotation_matrix.transpose() * point;

    x_gt[i] = unrotated_point.x();
    y_gt[i] = unrotated_point.z();
    z_gt[i] = -unrotated_point.y();
  }

  // Un-rotate the quaternion values
  for (size_t i = 0; i < qx_gt.size(); ++i) {
    // Convert quaternion to rotation matrix
    Eigen::Quaterniond orientation(qw_gt[i], qx_gt[i], qy_gt[i], qz_gt[i]);
    Eigen::Matrix3d rotation_matrix = orientation.toRotationMatrix();

    // Un-rotate the quaternion rotation matrix
    Eigen::Matrix3d unrotated_orientation =
        initial_rotation_matrix.transpose() * rotation_matrix;

    // Convert the un-rotated rotation matrix back to quaternion
    Eigen::Quaterniond unrotated_quaternion(unrotated_orientation);

    qx_gt[i] = unrotated_quaternion.x();
    qy_gt[i] = unrotated_quaternion.y();
    qz_gt[i] = unrotated_quaternion.z();
    qw_gt[i] = unrotated_quaternion.w();
  }

  gt_file.close();
}

/**
 * @brief Function to get the image data from the dataset
 *
 */
std::tuple<double, cv::Mat, std::string> dl::DataLoader::get_image_data() {
  std::string line;
  if (std::getline(image_file, line)) {
    std::istringstream iss(line);
    int id;
    double timestamp;
    std::string image_path;

    // If parsing fails, return an empty image
    if (!(iss >> id >> timestamp >> image_path)) {
      return std::make_tuple(-1.0, cv::Mat(), image_path);
    }

    // If this is the first valid image, store the timestamp
    std::string full_image_path = dataset_path + "/" + image_path;
    cv::Mat image = cv::imread(full_image_path, cv::IMREAD_COLOR);
    return std::make_tuple(timestamp, image, image_path);
  }

  // No more images to read
  return std::make_tuple(-1.0, cv::Mat(), "none");
}
