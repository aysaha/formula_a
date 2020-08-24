cmake_minimum_required(VERSION 3.5)
project(drivers)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)

# libraries
include_directories(include)
add_library(xbox_controller SHARED src/xbox_controller.cpp)
ament_target_dependencies(xbox_controller rclcpp rclcpp_components)
rclcpp_components_register_nodes(xbox_controller drivers::XboxController)

# interfaces
set(MSG_FILES msg/Input.msg msg/Output.msg)
set(SRV_FILES srv/Start.srv srv/Power.srv srv/Trim.srv)
rosidl_generate_interfaces(${PROJECT_NAME} ${MSG_FILES} ${SRV_FILES})
ament_export_dependencies(rosidl_default_runtime)
rosidl_target_interfaces(xbox_controller ${PROJECT_NAME} rosidl_typesupport_cpp)

# install
install(TARGETS xbox_controller ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
