cmake_minimum_required(VERSION 3.8)
project(social_nav)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ros_gz_sim REQUIRED)
find_package(ros_gz_bridge REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

find_package(sensor_msgs REQUIRED)
find_package(vision_msgs REQUIRED)

add_executable(proxemics_checker src/proxemics_checker.cpp)
add_executable(human_detector src/human_detector.cpp)

ament_target_dependencies(proxemics_checker
  rclcpp
  geometry_msgs
  nav_msgs
)

ament_target_dependencies(human_detector
  rclcpp
  sensor_msgs
  geometry_msgs
  vision_msgs
)

install(TARGETS
  proxemics_checker
  human_detector
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  social_nav/yolo_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME yolo_node
)

install(DIRECTORY
  launch
  worlds
  models
  config
  maps
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
