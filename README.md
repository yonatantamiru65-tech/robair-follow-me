# RobAIR Follow-Me: DATMO source snapshot

ROS 2 / C++ academic implementation of detection and tracking of moving objects (DATMO) for human-following behaviour using 2D LiDAR.

## Included files

- `datmo.cpp`: DATMO implementation.
- `datmo.hpp`: node declarations, interfaces and configuration constants.
- `project_robair (1).pdf`: implementation and test report.

This is a source snapshot, not a complete standalone ROS package. The supplied archive does not include `package.xml`, `CMakeLists.txt`, launch files or the complete RobAIR workspace. Integrate it with the original course/platform workspace rather than expecting `colcon build` to work on this directory alone.

## Technical context

The header references ROS 2 `rclcpp`, `sensor_msgs`, `visualization_msgs`, `geometry_msgs`, `std_msgs` and `nav_msgs`. Platform configuration and the robot interface must come from the original workspace. No live robot tests or build validation were performed during repository preparation.

## Reported evaluation

The report documents nine detection tests, five tracking tests and three additional failure investigations: target switching during occlusion, stale predictions after a sharp turn, and merged leg clusters. These counts describe test cases, not a measured accuracy percentage or an assertion that every case passed without limitations.

## Contributors and attribution

The report credits Leo Gabelli, Thomas Darnault and Yonatan Tamiru Asfaw. This repository should preserve that shared attribution and any original course/platform code ownership. No new software license has been assigned.
