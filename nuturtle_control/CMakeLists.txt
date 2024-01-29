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
# find_package(std_srvs REQUIRED)
find_package(nuturtle_description REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rosidl_default_generators REQUIRED)
# find_package(visualization_msgs REQUIRED)
find_package(nuturtlebot_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

set(srv_files
"srv/InitialPose.srv"
)
rosidl_generate_interfaces(${PROJECT_NAME}
${srv_files}
)

add_executable(turtle_control_node src/turtle_control.cpp)
ament_target_dependencies(
  turtle_control_node
  rclcpp
  std_msgs
  nuturtle_description
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
  nuturtle_description
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
  nuturtle_description
  geometry_msgs
  tf2
  tf2_ros
  nuturtlebot_msgs
  sensor_msgs
  turtlelib
  nav_msgs)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)

target_link_libraries(turtle_control_node turtlelib::turtlelib "${cpp_typesupport_target}")
target_link_libraries(odom_node turtlelib::turtlelib "${cpp_typesupport_target}")
target_link_libraries(circle_node turtlelib::turtlelib "${cpp_typesupport_target}")

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

install(TARGETS
  turtle_control_node
  odom_node
  circle_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY srv
  DESTINATION share/${PROJECT_NAME}/)


ament_package()
