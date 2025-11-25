cmake_minimum_required(VERSION 3.8)
project(offboard_rl)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()


if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem thread)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3)

include_directories(include)


add_executable(go_to_point src/go_to_point.cpp)
add_executable(trajectory src/trajectory.cpp)

ament_target_dependencies(go_to_point
  rclcpp
  px4_msgs
  Boost
  geometry_msgs
  Eigen3
)

ament_target_dependencies(trajectory
  rclcpp
  px4_msgs
  Boost
  geometry_msgs
  Eigen3
)


install(TARGETS
  go_to_point
  trajectory
  DESTINATION lib/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()