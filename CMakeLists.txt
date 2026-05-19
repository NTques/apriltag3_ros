cmake_minimum_required(VERSION 3.8)
project(apriltag3_ros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(apriltag REQUIRED)
find_package(apriltag3_msgs REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(OpenCV 4 REQUIRED COMPONENTS core calib3d)

# --- Generated parameter library --------------------------------------------
generate_parameter_library(
  apriltag3_ros_parameters
  src/apriltag3_ros_parameters.yaml
)

# --- Component library (shared) ---------------------------------------------
# Holds the AprilTag3RosNode + helper classes. Loadable via component containers
# and also linked into the standalone executable below.
add_library(${PROJECT_NAME}_component SHARED
  src/apriltag3_ros_node.cpp
  src/detector.cpp
  src/pose_solver.cpp
)

target_include_directories(${PROJECT_NAME}_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

target_compile_features(${PROJECT_NAME}_component PUBLIC c_std_99 cxx_std_17)

target_link_libraries(${PROJECT_NAME}_component
  PUBLIC
    apriltag::apriltag
    ${OpenCV_LIBS}
  PRIVATE
    apriltag3_ros_parameters
)

ament_target_dependencies(${PROJECT_NAME}_component
  PUBLIC
    rclcpp
    rclcpp_components
    std_msgs
    sensor_msgs
    geometry_msgs
    tf2_ros
    tf2_geometry_msgs
    image_transport
    cv_bridge
    diagnostic_updater
    apriltag3_msgs
)

rclcpp_components_register_node(${PROJECT_NAME}_component
  PLUGIN "apriltag3_ros::AprilTag3RosNode"
  EXECUTABLE ${PROJECT_NAME}_container
)

# --- Standalone executable --------------------------------------------------
add_executable(${PROJECT_NAME} src/apriltag3_ros.cpp)
target_link_libraries(${PROJECT_NAME} PRIVATE ${PROJECT_NAME}_component)
ament_target_dependencies(${PROJECT_NAME} PUBLIC rclcpp)

# --- Install ----------------------------------------------------------------
install(TARGETS ${PROJECT_NAME}_component
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include/${PROJECT_NAME}
)

install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME})

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include/${PROJECT_NAME})
ament_export_libraries(${PROJECT_NAME}_component)
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(
  rclcpp rclcpp_components std_msgs sensor_msgs geometry_msgs
  tf2_ros tf2_geometry_msgs image_transport cv_bridge
  diagnostic_updater apriltag3_msgs)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
