/**
 * @file test.cpp
 * @author Apoorv Thapliyal
 * @brief C++ test file for DataLoader, InertialOdometry, and VisualOdometry
 * classes
 * @version 0.1
 * @date 2024-10-23
 *
 * @copyright Copyright (c) 2024
 *
 */

#include <gtest/gtest.h>

#include "data_loader.hpp"
#include "gmock/gmock.h"
#include "inertial_odometry.hpp"
#include "visual_odometry.hpp"

/**
 * @brief Test fixture for Inertial Odometry class
 *
 */
class InertialOdometryTests : public ::testing::Test {
 protected:
  void SetUp() override {
    test_inertial_odometry =
        new io::InertialOdometry(Eigen::Matrix4d::Identity());
  };

  void TearDown() override { delete test_inertial_odometry; };

  io::InertialOdometry* test_inertial_odometry;
};

/**
 * @brief Construct a test for rodrigues formula
 *
 */
TEST_F(InertialOdometryTests, TestRodriguesFormula) {
  Eigen::Vector3d w(0.0, 0.0, 0.0);
  Eigen::Matrix3d R = test_inertial_odometry->rodrigues_formula(w);

  Eigen::Matrix3d expected_R = Eigen::Matrix3d::Identity();

  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      EXPECT_NEAR(R(i, j), expected_R(i, j), 1e-9);
    }
  }
}

/**
 * @brief Construct a test for rodrigues formula
 *
 */
TEST_F(InertialOdometryTests, TestRodriguesFormula2) {
  Eigen::Vector3d w(0.000465131, 0.506078, -0.640403);
  Eigen::Matrix3d R = test_inertial_odometry->rodrigues_formula(w);

  Eigen::Matrix3d expected_R;
  expected_R << 1, 0.000640403, 0.000506078, -0.000640403, 1, -6.27178e-07,
      -0.000506078, 3.03084e-07, 1;

  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      EXPECT_NEAR(R(i, j), expected_R(i, j), 1e-5);
    }
  }
}

/**
 * @brief Construct a test for get_pose function
 *
 */
TEST_F(InertialOdometryTests, TestGetPose) {
  Eigen::Matrix4d pose = test_inertial_odometry->get_pose();
  Eigen::Matrix4d expected_pose = Eigen::Matrix4d::Identity();

  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      EXPECT_NEAR(pose(i, j), expected_pose(i, j), 1e-9);
    }
  }
}

/**
 * @brief Construct a test to check if the pose is updated correctly
 *
 */
TEST_F(InertialOdometryTests, TestUpdatePose) {
  Eigen::Vector3d a(0.0, 0.0, 0.0);
  Eigen::Vector3d w(0.0, 0.0, 0.0);
  test_inertial_odometry->update_pose(a, w);

  Eigen::Matrix4d pose = test_inertial_odometry->get_pose();
  Eigen::Matrix4d expected_pose = Eigen::Matrix4d::Identity();

  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      EXPECT_NEAR(pose(i, j), expected_pose(i, j), 1e-9);
    }
  }
}

/**
 * @brief Construct a test for the constructor
 *
 */
TEST_F(InertialOdometryTests, TestConstructor) {
  Eigen::Matrix4d pose = test_inertial_odometry->get_pose();
  Eigen::Matrix4d expected_pose = Eigen::Matrix4d::Identity();

  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      EXPECT_NEAR(pose(i, j), expected_pose(i, j), 1e-9);
    }
  }
}

/**
 * @brief Test fixture for DataLoader class
 *
 */
class DataLoaderTests : public ::testing::Test {
 protected:
  void SetUp() override {
    test_data_loader =
        new dl::DataLoader("../../indoor_forward_9_davis_with_gt");
  };

  void TearDown() override { delete test_data_loader; };

  dl::DataLoader* test_data_loader;
};

/**
 * @brief Construct a test for get_imu_data function
 *
 */
TEST_F(DataLoaderTests, TestGetImuData) {
  auto imu_data = test_data_loader->get_imu_data();
  // long double timestamp = 1540822817.202034711838;
  // Eigen::Vector3d angular_velocity{-0.059654804257, -0.003195793048,
  // -0.019174759849}; Eigen::Vector3d linear_acceleration{0.670605468750,
  // -9.819580078125, 1.125659179687};
  // // auto imu_data = {}
  double timestamp = std::get<0>(imu_data);
  Eigen::Vector3d angular_velocity = std::get<1>(imu_data);
  Eigen::Vector3d linear_acceleration = std::get<2>(imu_data);

  long double threshold = 1e-5;
  EXPECT_NEAR(timestamp, 1540822817.202034711838, threshold);
  EXPECT_NEAR(angular_velocity(0), -0.059654804257, threshold);
  EXPECT_NEAR(angular_velocity(1), -0.003195793048, threshold);
  EXPECT_NEAR(angular_velocity(2), -0.019174759849, threshold);
  EXPECT_NEAR(linear_acceleration(0), 0.670605468750, threshold);
  EXPECT_NEAR(linear_acceleration(1), -9.819580078125, threshold);
  EXPECT_NEAR(linear_acceleration(2), 1.125659179687, threshold);
}

/**
 * @brief Construct a test for get_image_data function
 *
 */
TEST_F(DataLoaderTests, TestGetImageData) {
  auto image_data = test_data_loader->get_image_data();
  double timestamp = std::get<0>(image_data);
  cv::Mat image = std::get<1>(image_data);
  std::string image_path = std::get<2>(image_data);

  long double threshold = 1e-2;
  EXPECT_NEAR(timestamp, 1540822817.214196681976, threshold);
  EXPECT_EQ(image.rows, 260);
  EXPECT_EQ(image.cols, 346);
  EXPECT_EQ(image.channels(), 3);
  EXPECT_EQ(image_path, "img/image_0_0.png");
}

/**
 * @brief Test fixture for Visual Odometry class
 *
 */
class VisualOdometryTests : public ::testing::Test {
 protected:
  void SetUp() override {
    test_visual_odometry = new vo::VisualOdometry(Eigen::Matrix4d::Identity());
  };

  void TearDown() override { delete test_visual_odometry; };

  vo::VisualOdometry* test_visual_odometry;
};

/**
 * @brief Construct a test for get_pose function
 *
 */

TEST_F(VisualOdometryTests, TestGetPose) {
  Eigen::Matrix4d pose = test_visual_odometry->get_pose();
  Eigen::Matrix4d expected_pose = Eigen::Matrix4d::Identity();

  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      EXPECT_NEAR(pose(i, j), expected_pose(i, j), 1e-9);
    }
  }
}

/**
 * @brief Construct a test for update_pose function
 *
 */
TEST_F(VisualOdometryTests, TestUpdatePose) {
  cv::Mat image =
      cv::imread("../../indoor_forward_9_davis_with_gt/img/image_0_1101.png");
  test_visual_odometry->update_pose(image);

  image =
      cv::imread("../../indoor_forward_9_davis_with_gt/img/image_0_1102.png");
  test_visual_odometry->update_pose(image);

  Eigen::Matrix4d pose = test_visual_odometry->get_pose();

  // Fill in the expected pose
  Eigen::Matrix4d expected_pose = Eigen::Matrix4d::Identity();
  expected_pose(0, 3) = 0.577350;
  expected_pose(1, 3) = -0.577350;
  expected_pose(2, 3) = 0.577350;

  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      EXPECT_NEAR(pose(i, j), expected_pose(i, j), 1e-3);
    }
  }
}
