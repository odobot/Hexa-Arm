cmake_minimum_required(VERSION 3.8)
project(hexaarm_moveit_servo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_PREFIX_PATH "/usr/lib/x86_64-linux-gnu/cmake/libusb-1.0" ${CMAKE_PREFIX_PATH})

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(PkgConfig REQUIRED)


pkg_check_modules(LIBUSB REQUIRED libusb-1.0)
include_directories(${LIBUSB_INCLUDE_DIRS})
link_directories(${LIBUSB_LIBRARY_DIRS})


set(THIS_PACKAGE_INCLUDE_DEPENDS
  rclcpp
  rclcpp_components
  std_msgs
  std_srvs
  sensor_msgs
  control_msgs
  moveit_msgs
  geometry_msgs
  moveit_ros_planning
)

include_directories(
  include
  ${ament_INCLUDE_DIRS}
)

add_library(hexaarm_joystick_input SHARED src/hexaarm_joystick_input.cpp)
target_include_directories(
  hexaarm_joystick_input
  PRIVATE "hexaarm_joystick_input_node_CPP_BUILD_DLL"
)
ament_target_dependencies(hexaarm_joystick_input ${THIS_PACKAGE_INCLUDE_DEPENDS})

rclcpp_components_register_node(
  hexaarm_joystick_input
    PLUGIN "hexaarm_moveit_servo::JoyToServoPub"
    EXECUTABLE hexaarm_joystick_input_node
)

install(
  TARGETS
    hexaarm_joystick_input
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  INCLUDES DESTINATION include
)

install(
  DIRECTORY
    config
    launch
    rviz
  DESTINATION share/${PROJECT_NAME}/
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
