cmake_minimum_required(VERSION 3.8)
project(robot_control_system_tester)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(trajectory_msgs REQUIRED)

include_directories(include)

add_executable(start src/robot_control_commander.cpp)
ament_target_dependencies(start rclcpp trajectory_msgs)

install(
    DIRECTORY include/
    DESTINATION include/
)

install(TARGETS
  start
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
