add_executable(app
    main.cpp)

# Any dependent libraires needed to build this target.
target_link_libraries(app PUBLIC
  # list of libraries
    DataLoader
    InertialOdometry
    VisualOdometry
  )