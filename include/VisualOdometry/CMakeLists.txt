add_library(VisualOdometry
  # list of cpp source files:
  visual_odometry.cpp
  )

target_include_directories(VisualOdometry PUBLIC
  # list of directories:
  .
  )