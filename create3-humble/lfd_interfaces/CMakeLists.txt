cmake_minimum_required(VERSION 3.8)
project(lfd_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(nav_2d_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(msg_files
  "msg/LfdPose.msg"
  "msg/LfdComp.msg"

)

set(srv_files
  "srv/AddTwoInts.srv"
  "srv/SetBool.srv"
  "srv/Trigger.srv"
  "srv/LfdRes.srv"
  "srv/SMCRes.srv"
  "srv/CalcCmdVel.srv"
)

set(action_files
  "action/RFIDController.action"
  "action/CommandManager.action"
  "action/Navigate.action"
  "action/Location.action"
  "action/WaypointSaver.action"
  "action/MapDrawer.action"


)

## Generate added messages and services with any dependencies listed here
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  ${action_files}
    
  DEPENDENCIES nav_2d_msgs
  DEPENDENCIES std_msgs
  DEPENDENCIES geometry_msgs
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
ament_export_dependencies(rosidl_default_runtime)

ament_package()
