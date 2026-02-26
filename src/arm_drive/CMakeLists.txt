cmake_minimum_required(VERSION 3.8)
project(arm_drive)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(robot_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(tf2 REQUIRED)
pkg_check_modules(LIBUSB REQUIRED libusb-1.0)


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

add_executable(arm_drive
  src/main.cpp
  src/cdc_trans.cpp
  src/arm_sernode.cpp
)

target_link_libraries(arm_drive usb-1.0)


target_include_directories(arm_drive
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)


# Find and link libusb (libusb-1.0) via pkg-config to resolve undefined references
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBUSB REQUIRED libusb-1.0)

target_include_directories(arm_drive
  PUBLIC
  ${LIBUSB_INCLUDE_DIRS}
)




ament_target_dependencies(arm_drive
  rclcpp
  geometry_msgs
  sensor_msgs
  tf2
  robot_interfaces)

  

install(TARGETS
arm_drive
DESTINATION lib/${PROJECT_NAME})

ament_package()
