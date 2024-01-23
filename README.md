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

