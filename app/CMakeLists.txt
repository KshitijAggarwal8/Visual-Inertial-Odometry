add_executable(app_io
    main_io.cpp)

add_executable(app_vo
    main_vo.cpp)

# Any dependent libraires needed to build this target.
target_link_libraries(app_io PUBLIC
  # list of libraries
    DataLoader
    InertialOdometry
    VisualOdometry
  )

# Any dependent libraires needed to build this target.
target_link_libraries(app_vo PUBLIC
  # list of libraries
    DataLoader
    InertialOdometry
    VisualOdometry
  )