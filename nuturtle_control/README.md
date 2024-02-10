# Nuturtle Control
1. A brief description of the package.
* `nuturtle_control.cpp` - node that creates the controller for the turtlebot to move in a circle.

## Launch File Details
To view the robot moving in a circle:
* `ros2 launch nuturtle_control start_robot.launch.xml cmd_src:=circle robot:=nusim use_rviz:=true` to see the robot moving in a circle.

![Screenshot from 2024-02-10 02-34-20](https://github.com/ME495-Navigation/slam-project-dbarsoum/assets/117933155/13a506ba-ead0-4859-9d31-32c2199f76d3)


## Node Details
* `circle.cpp` - node that creates the controller for the turtlebot to move in a circle.
* `odometry.cpp` - node that subscribes to the odometry topic and publishes the pose of the robot.
* `turtle_control.cpp` - node that creates the controller for the turtlebot to move in a circle.

## Services
* Give the robot a velocity and radius to move in a circle using the service `Control.srv`. Run `ros2 service call /control "nuturtle_control/Control" "{velocity: 0.2, radius: 0.5}"` to give the robot a velocity of 0.2 m/s and a radius of 0.5 m.
* To give the robot a initial pose, use the service `InitialPose.srv`. Run `ros2 service call /initialpose "nuturtle_control/InitialPose" "{x: 0.0, y: 0.0, theta: 0.0}"` to give the robot an initial pose of (0, 0, 0).


## Video of the robot moving in the circle


[video_circle_turtlebot](https://github.com/ME495-Navigation/slam-project-dbarsoum/assets/117933155/a2dab2b7-4285-4396-b817-40601c7e6a31)

Drift was ~7cm.
