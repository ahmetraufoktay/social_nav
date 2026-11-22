cmake_minimum_required(VERSION 3.8)
project(social_nav)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_compile_options(-g)

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ros_gz_sim REQUIRED)
find_package(ros_gz_bridge REQUIRED)

find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(image_geometry REQUIRED)

find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(human_state_msgs REQUIRED)

add_executable(proxemics_checker src/proxemics_checker.cc)
add_executable(human_detector src/human_detector.cc)
add_executable(human_state_visualizer src/human_state_visualizer.cc)

target_include_directories(human_detector PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
)

target_include_directories(proxemics_checker PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_include_directories(human_state_visualizer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(human_detector
  rclcpp::rclcpp
  ${std_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${visualization_msgs_TARGETS}
  ${vision_msgs_TARGETS}
  image_geometry::image_geometry
  cv_bridge::cv_bridge
  tf2::tf2
  tf2_ros::tf2_ros
  tf2_geometry_msgs::tf2_geometry_msgs
  ${OpenCV_LIBRARIES}
  ${human_state_msgs_TARGETS}
)

target_link_libraries(proxemics_checker
  rclcpp::rclcpp
  ${std_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${visualization_msgs_TARGETS}
  ${human_state_msgs_TARGETS}
)

target_link_libraries(human_state_visualizer
  rclcpp::rclcpp
  ${human_state_msgs_TARGETS}
  ${visualization_msgs_TARGETS}
)

install(TARGETS
  proxemics_checker
  human_detector
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS 
  human_detector 
  human_state_visualizer
  DESTINATION lib/${PROJECT_NAME})

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
  weights
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

# Python venv interpreter path for launch files
set(SOCIAL_NAV_PYTHON "${CMAKE_CURRENT_SOURCE_DIR}/.venv/bin/python3"
  CACHE FILEPATH "Python interpreter for the human detection nodes")
file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/python_interpreter" "${SOCIAL_NAV_PYTHON}\n")
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/python_interpreter"
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
