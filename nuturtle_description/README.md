# Nuturtle Description
URDF files for Nuturtle <Name Your Robot>
* `ros2 launch nuturtle_description load_one.launch.py` to see the robot in rviz.
* `ros2 launch nuturtle_description load_all.launch.xml` to see four copies of the robot in rviz.

* Image of the four copies of the turtlebot3 in rivz at specified locations.
![rviz](https://github.com/ME495-Navigation/slam-project-dbarsoum/assets/117933155/46323dd8-fe77-4dc6-a24d-231ff3f9bd2a)

* The rqt_graph when all four robots are visualized (Nodes Only, Hide Debug) is:
![rqt_graph](https://github.com/ME495-Navigation/slam-project-dbarsoum/assets/117933155/4f041282-c8b8-4e50-af49-3e7e75586246)

# Launch File Details
* `ros2 launch nuturtle_description load_one.launch.py --show-args`
    ```
    Arguments (pass arguments as '<name>:=<value>'):

        'use_rviz':
            for launching rviz or not
            (default: 'true')

        'use_jsp':
            whether the jsp is published or not
            (default: 'true')

        'color':
            color of the robot. Valid choices are: ['purple', 'red', 'green', 'blue']
            (default: 'purple')
    ```
* `ros2 launch nuturtle_description load_all.launch.xml --show-args`
    ```
    Arguments (pass arguments as '<name>:=<value>'):

        'use_rviz':
            for launching rviz or not
            (default: 'true')

        'use_jsp':
            whether the jsp is published or not
            (default: 'true')

        'color':
            color of the robot. Valid choices are: ['purple', 'red', 'green', 'blue']
            (default: 'purple')
    ```