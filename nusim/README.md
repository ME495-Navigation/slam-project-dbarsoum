# Nusim 
1. A brief description of the package.
* `nusim.cpp` - node that creates the simulation env with robot in arena with obstacles.

## Launch File Details
To view the rviz simualtion:
* `ros2 launch nusim nusim.launch.xml` to see the simulation of the robot in the arena with obstacles.

![nusim1](https://github.com/ME495-Navigation/slam-project-dbarsoum/assets/117933155/91fe1f73-a8da-42d9-80bc-8df10d21e5ff)


## Simulator Settings
* changes can be made to `basic_world.yaml` to edit the length of the arena, edit the number of obstacles shown in rviz by adding additional (x, y) locations for additional obstacle.
* to edit the rate of the timer is published, edit `rate`
* to edit the length of the arena using `arena_x_length` and `arena_y_length`
* to edit the location of the red robot, edit `x0`, `y0`, and `theta0`.
* to add x locations of obstacles to `obstacles/x`
* to add y locations of obstacles to `obstacles/y`
* to edit/add/ radius of obstacles to `obstacles/r`

## Services
* To teleport the robot run `ros2 service call /nusim_node/teleport nusim/srv/Teleport "{x: 2.0, y: 2.0, theta:0.0}"`.

image of the example coordinates:

![nusim_teleport](https://github.com/ME495-Navigation/slam-project-dbarsoum/assets/117933155/ac74a3ae-b33d-43bd-943b-39775075037e)

* to reset to the location of the red robot specified in `basic_world.yaml`, run `ros2 service call /nusim/reset std_srvs/srv/Empty "{}"`.
