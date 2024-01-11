# ME495 Sensing, Navigation and Machine Learning For Robotics
* Demiana Barsoum
* Winter 2024
# Package List
This repository consists of several ROS packages
- nuturtle_description - ROS cmake pkg that includes the robot description of the turtlebot. (includes urdf files, basic debugging, testing, and visualization code)
- 


# Nuturtle  Description
URDF files for Nuturtle <Name Your Robot>
* `ros2 launch nuturtle_description load_one.launch.py color:=<color>` to see the robot in rviz.
* `ros2 launch nuturtle_description load_all.launch.xml` to see four copies of the robot in rviz.
![](images/rviz.png)
* The rqt_graph when all four robots are visualized (Nodes Only, Hide Debug) is:
![](images/rqt_graph.svg)
# Launch File Details
* `<Command To Show Arguments of load_one.launch.py>`
  `<Output of the Above Command>`
* `<Command To Show Arguments of load_all.launch.py>`
  `<Output of the Above Command>`


  cmake_minimum_required(VERSION 3.10)
project(turtlelib)
# tells cmake to create an executable using specified source files
add_executable(turtlelib src/geometry.cpp)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

target_include_directories(turtlelib PUBLIC
    ${PROJECT_SOURCE_DIR}/include
    ${PROJECT_SOURCE_DIR}/src
)


