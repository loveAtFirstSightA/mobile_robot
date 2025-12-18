cmake_minimum_required(VERSION 3.8)
project(amr_msgs)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(action_msgs REQUIRED)

set(msg_files
  "msg/Pose.msg"
  "msg/Velocity.msg"
  "msg/Line.msg"
  "msg/Rotate.msg"
  "msg/Path.msg"
  "msg/Paths.msg"
  "msg/Point.msg"
  "msg/Points.msg"
  "msg/AmrState.msg"
  "msg/TaskPoint.msg"
)

set(srv_files
  "srv/Sound.srv"
  "srv/Dqn.srv"
  "srv/Goal.srv"
  "srv/PurePursuit.srv"
  "srv/AStar.srv"
)

set(action_files
  "action/Patrol.action"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  ${action_files}
  DEPENDENCIES std_msgs geometry_msgs action_msgs
  ADD_LINTER_TESTS
)

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

ament_package()
