#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "apriltag3_ros/apriltag3_ros_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto node = std::make_shared<apriltag3_ros::AprilTag3RosNode>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
