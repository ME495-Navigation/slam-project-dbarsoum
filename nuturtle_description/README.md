# Nuturtle Description
URDF files for Nuturtle <Name Your Robot>
* `ros2 launch nuturtle_description load_one.launch.py color:=<color>` to see the robot in rviz.
* `ros2 launch nuturtle_description load_all.launch.xml` to see four copies of the robot in rviz.
![](images/rviz.png)
* The rqt_graph when all four robots are visualized (Nodes Only, Hide Debug) is:
![](images/rqt_graph.svg)

# Launch File Details
* `ros2 launch nuturtle_description load_one.launch.py --show-args`
  `Arguments (pass arguments as '<name>:=<value>'):

    'use_rviz':
        for launching rviz or not
        (default: 'true')

    'use_jsp':
        whether the jsp is published or not
        (default: 'true')

    'color':
        color of the robot. Valid choices are: ['purple', 'red', 'green', 'blue']
        (default: 'purple')
        `
* `ros2 launch nuturtle_description load_all.launch.xml --show-args`
  `Arguments (pass arguments as '<name>:=<value>'):

    'use_rviz':
        for launching rviz or not
        (default: 'true')

    'use_jsp':
        whether the jsp is published or not
        (default: 'true')

    'color':
        color of the robot. Valid choices are: ['purple', 'red', 'green', 'blue']
        (default: 'purple')
        `