add_library(DataLoader
  # list of cpp source files:
  data_loader.cpp
  )

target_include_directories(DataLoader PUBLIC
  # list of directories:

  .
  )

target_link_libraries(DataLoader ${OpenCV_LIBS})  # Link OpenCV libraries