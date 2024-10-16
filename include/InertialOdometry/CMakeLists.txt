add_library(InertialOdometry
  # list of cpp source files:
  inertial_odometry.cpp
  )

target_include_directories(InertialOdometry PUBLIC
  # list of directories:
  .
  )