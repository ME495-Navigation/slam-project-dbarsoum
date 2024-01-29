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
find_package(nuturtle_description REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nuturtlebot_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(turtle_control_node src/turtle_control.cpp)
ament_target_dependencies(turtle_control_node rclcpp std_msgs nuturtle_description geometry_msgs nuturtlebot_msgs sensor_msgs turtlelib)

target_link_libraries(turtle_control_node turtlelib::turtlelib "${cpp_typesupport_target}")

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
  DESTINATION lib/${PROJECT_NAME})

# install(DIRECTORY test
#   DESTINATION share/${PROJECT_NAME}/)

ament_package()
