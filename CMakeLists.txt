cmake_minimum_required(VERSION 3.8)
project(apriltag3_ros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(image_geometry REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(apriltag REQUIRED)
find_package(apriltag3_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(generate_parameter_library REQUIRED)

# Generate parameter library from YAML
generate_parameter_library(apriltag_detector_parameters
  src/apriltag_detector_parameters.yaml
)

# AprilTagDetector composable component (shared library)
add_library(apriltag_detector_component SHARED
  src/apriltag_detector.cpp
  src/detector.cpp
)
target_include_directories(apriltag_detector_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_compile_features(apriltag_detector_component PUBLIC c_std_99 cxx_std_17)
target_link_libraries(apriltag_detector_component
  apriltag::apriltag
  apriltag_detector_parameters
  ${OpenCV_LIBS}
)
ament_target_dependencies(apriltag_detector_component
  rclcpp
  rclcpp_components
  sensor_msgs
  geometry_msgs
  image_transport
  image_geometry
  cv_bridge
  apriltag3_msgs
  tf2
  tf2_ros
)

# Register as composable component AND generate a standalone executable
rclcpp_components_register_node(apriltag_detector_component
  PLUGIN "apriltag3_ros::AprilTagDetector"
  EXECUTABLE apriltag_detector
)

install(TARGETS apriltag_detector_component apriltag_detector_parameters
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME})

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}/)

ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(
  rclcpp
  rclcpp_components
  sensor_msgs
  geometry_msgs
  image_transport
  image_geometry
  cv_bridge
  apriltag
  apriltag3_msgs
  tf2
  tf2_ros
  OpenCV
  generate_parameter_library
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
