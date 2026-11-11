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
find_package(visualization_msgs REQUIRED)
find_package(image_geometry REQUIRED)

find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

find_package(cv_bridge REQUIRED)


add_executable(proxemics_checker src/proxemics_checker.cpp)
add_executable(human_detector src/human_detector.cpp)

target_include_directories(human_detector PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
)

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
  visualization_msgs
  image_geometry
  tf2
  tf2_geometry_msgs
  tf2_ros
  cv_bridge
)

target_link_libraries(human_detector
  ${OpenCV_LIBRARIES}
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
  weights
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

# The torch-based nodes (YOLO, DR-SPAAM) run with this interpreter.
# simulation.launch.py reads this file, so the venv never has to be activated by hand.
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
