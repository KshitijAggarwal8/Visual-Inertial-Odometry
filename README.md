![CICD Workflow status](https://github.com/KshitijAggarwal8/Visual-Inertial-Odometry/actions/workflows/run-unit-test-and-upload-codecov.yml/badge.svg)

[![codecov](https://codecov.io/gh/KshitijAggarwal8/Visual-Inertial-Odometry/branch/main/graph/badge.svg)](https://codecov.io/gh/KshitijAggarwal8/Visual-Inertial-Odometry)

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)]

# Visual Inertial Odometry
Visual Inertial Odometry (VIO) estimates a device's position and orientation by fusing data from a camera and an Inertial Measurement Unit (IMU). The IMU provides acceleration and angular velocity, while the camera captures visual features for movement tracking. This combination enhances motion tracking accuracy, addressing sensor limitations like IMU drift and visual ambiguity in low light. We fuse the processed measurements using the **Levenberg-Marquardt optimization algorithm**, minimizing residuals between VO-derived and IMU-integrated positions. This approach corrects IMU drift with reliable visual information and mitigates visual errors from lighting or motion blur, resulting in robust position and orientation estimates in challenging conditions.

## Phase 0
Phase 0 involves the proposal for the project, along with information about the methodology that would be used for software development in the project. </br>
All documents pertaining to the project can be found under `/doc`. </br>
Initial Product Backlog link: https://tinyurl.com/bdfh7sdd </br>
A short video providing a brief overview of the project is provided here: https://www.youtube.com/watch?v=4DxbtCh8VpY


## Build 
1. **Generate Build Files**:
```bash
cmake -S . -B build
```

2. **Compile the Project**:
```bash
cmake --build build
```

## Generate Docs
1. **Build the target**:
```bash
cmake --build ./build --target docs
```
2. **View the docs**:
```bash
open ./docs/html/index.html
```
## Collaborators
Apoorv Thapliyal - 190907268 </br>
Kshitij Aggarwal - 119211618

## References
- https://github.com/HKUST-Aerial-Robotics/VINS-Mono
- https://fpv.ifi.uzh.ch/datasets/
- https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=8421746
- https://rpg.ifi.uzh.ch/docs/VO_Part_I_Scaramuzza.pdf

