cmake_minimum_required(VERSION 3.8)
project(follow_ball)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_geometry REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

find_package(OpenCV REQUIRED)

set(dependencies
  rclcpp
  sensor_msgs
  vision_msgs
  image_transport
  cv_bridge
  OpenCV
  image_geometry
  geometry_msgs
  tf2_geometry_msgs
  tf2
  tf2_ros
  visualization_msgs
)

include_directories(include)

add_library(${PROJECT_NAME} SHARED
  src/follow_ball/hsv_filter.cpp
  src/follow_ball/control_node.cpp
  src/follow_ball/obstacle_detector.cpp
)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

add_executable(hsv_filter_main src/hsv_filter_main.cpp)
ament_target_dependencies(hsv_filter_main ${dependencies})
target_link_libraries(hsv_filter_main ${PROJECT_NAME})

add_executable(obstacle_detector_main src/obstacle_detector_main.cpp)
ament_target_dependencies(obstacle_detector_main ${dependencies})
target_link_libraries(obstacle_detector_main ${PROJECT_NAME})

add_executable(control_node_main src/control_node_main.cpp)
ament_target_dependencies(control_node_main ${dependencies})
target_link_libraries(control_node_main ${PROJECT_NAME})

install(TARGETS
  ${PROJECT_NAME}
  hsv_filter_main
  obstacle_detector_main
  control_node_main
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)

  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})
ament_package()
