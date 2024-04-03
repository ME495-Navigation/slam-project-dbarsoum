# ME495 Sensing, Navigation and Machine Learning For Robotics
* Demiana Barsoum
* Winter 2024
* Northwestern University

# Package List
This repository consists of several ROS packages
- nuturtle_description - ROS cmake pkg that includes the robot description of the turtlebot. (includes urdf files, basic debugging, testing, and visualization code)
- turtlelib - contains library (and tests) for geometry and se2d primitives
- nusim - ros2 cmake pkg to simulate and visualize the robot, obstacles, and arena in rviz2
- nuturtle_control - ros2 cmake pkg that contains the controller for the turtlebot to move in a circle
- nuslam - ros2 cmake pkg that contains the SLAM algorithm to estimate the robot's pose and the landmarks' positions

Each package has its own README.md file that explains the package and how to run it with videos and images.

## SLAM (EKF Algorithm)

![Screenshot from 2024-03-09 18-07-44](https://github.com/ME495-Navigation/slam-project-dbarsoum/assets/117933155/97bcfbda-45a5-4b94-b33e-2c2182141fe2)

The green (simulated robot) follows the real robot (red) trajectory.

An Extented Kalman Filter (EKF) is used to estimate the robot's pose and the landmarks' positions.
