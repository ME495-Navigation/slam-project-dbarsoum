cmake_minimum_required(VERSION 3.8)
project(nuturtle_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# set(turtlelib_DIR "/home/demiana/Documents/slam_me495/workspaces/slam_ws/install/turtlelib/lib/cmake/turtlelib")
# set(CMAKE_PREFIX_PATH "/home/demiana/Documents/slam_me495/workspaces/slam_ws/install/turtlelib/lib")
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(turtlelib REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nuturtlebot_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
# find_package(nuturtle_control REQUIRED)

set(srv_files
"srv/InitialPose.srv"
"srv/Control.srv"
)
rosidl_generate_interfaces(${PROJECT_NAME}
${srv_files}
)

add_executable(turtle_control_node src/turtle_control.cpp)
ament_target_dependencies(
  turtle_control_node
  rclcpp
  std_msgs
  std_srvs
  geometry_msgs
  tf2
  tf2_ros
  nuturtlebot_msgs
  sensor_msgs
  turtlelib
  nav_msgs)

add_executable(odom_node src/odometry.cpp)
ament_target_dependencies(
  odom_node
  rclcpp
  std_msgs
  std_srvs
  geometry_msgs
  tf2
  tf2_ros
  nuturtlebot_msgs
  sensor_msgs
  turtlelib
  nav_msgs)

add_executable(circle_node src/circle.cpp)
ament_target_dependencies(
  circle_node
  rclcpp
  std_msgs
  std_srvs
  geometry_msgs
  tf2
  tf2_ros
  nuturtlebot_msgs
  sensor_msgs
  turtlelib
  nav_msgs)

# rosidl_get_typesuppsort_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)

rosidl_target_interfaces(odom_node ${PROJECT_NAME} "rosidl_typesupport_cpp")
# target_link_libraries(odom_node "${cpp_typesupport_target}")

rosidl_target_interfaces(circle_node ${PROJECT_NAME} "rosidl_typesupport_cpp")
# target_link_libraries(circle_node "${cpp_typesupport_target}")

rosidl_target_interfaces(turtle_control_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

target_link_libraries(turtle_control_node turtlelib::turtlelib "${cpp_typesupport_target}")
target_link_libraries(odom_node turtlelib::turtlelib "${cpp_typesupport_target}")
target_link_libraries(circle_node turtlelib::turtlelib "${cpp_typesupport_target}")


install(TARGETS
  turtle_control_node
  odom_node
  circle_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY srv launch config
  DESTINATION share/${PROJECT_NAME}/)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

if(BUILD_TESTING)
  include(CTest)

  find_package(catch_ros2 REQUIRED)
  ###############
  ## UNIT TEST ##
  ###############
  # control test
  add_executable(turtle_control_test_node
    test/turtle_control_test_node.cpp
  )
  # Linking a unit test executable with catch_ros2::catch_ros2_with_main
  # provides the default Catch2 main function to the executable.
  target_link_libraries(turtle_control_test_node
    catch_ros2::catch_ros2_with_node_main
  )
  ament_target_dependencies(turtle_control_test_node
  rclcpp
  std_msgs
  std_srvs
  geometry_msgs
  tf2
  tf2_ros
  nuturtlebot_msgs
  sensor_msgs
  turtlelib
  nav_msgs)

  target_link_libraries(turtle_control_test_node turtlelib::turtlelib "${cpp_typesupport_target}")

  install(TARGETS
    turtle_control_test_node
    DESTINATION lib/${PROJECT_NAME}
  )

  install(FILES
    test/turtle_control_test.launch.xml
    DESTINATION share/${PROJECT_NAME}
  )

  #### odom test
  add_executable(turtle_odom_test_node
    test/turtle_odom_test_node.cpp
  )

  target_link_libraries(turtle_odom_test_node
    catch_ros2::catch_ros2_with_node_main
  )

  ament_target_dependencies(turtle_odom_test_node
  rclcpp
  std_msgs
  std_srvs
  geometry_msgs
  tf2
  tf2_ros
  nuturtlebot_msgs
  sensor_msgs
  turtlelib
  nav_msgs)

  target_link_libraries(turtle_odom_test_node turtlelib::turtlelib "${cpp_typesupport_target}")
  rosidl_target_interfaces(turtle_odom_test_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

  install(TARGETS
    turtle_odom_test_node
    DESTINATION lib/${PROJECT_NAME}
  )


  install(FILES
    test/turtle_odom_test.launch.xml
    DESTINATION share/${PROJECT_NAME}
  )

  #### circle test
  add_executable(turtle_circle_test_node
    test/turtle_circle_test_node.cpp
  )

  target_link_libraries(turtle_circle_test_node
    catch_ros2::catch_ros2_with_node_main
  )

  ament_target_dependencies(turtle_circle_test_node
  rclcpp
  std_msgs
  std_srvs
  geometry_msgs
  tf2
  tf2_ros
  nuturtlebot_msgs
  sensor_msgs
  turtlelib
  nav_msgs)
  # nuturtle_control)

  target_link_libraries(turtle_circle_test_node turtlelib::turtlelib "${cpp_typesupport_target}")
  rosidl_target_interfaces(turtle_circle_test_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

  install(TARGETS
    turtle_circle_test_node
    DESTINATION lib/${PROJECT_NAME}
  )

  install(FILES
    test/turtle_circle_test.launch.xml
    DESTINATION share/${PROJECT_NAME}
  )

  ###############
  catch_ros2_add_integration_test(TurtleControl_TestXML
    LAUNCH_FILE turtle_control_test.launch.xml  # example in XML
  )

  catch_ros2_add_integration_test(Odometry_TestXML
    LAUNCH_FILE turtle_odom_test.launch.xml  # example in XML
  )

  catch_ros2_add_integration_test(Circle_TestXML
    LAUNCH_FILE turtle_circle_test.launch.xml  # example in XML
  )

endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
