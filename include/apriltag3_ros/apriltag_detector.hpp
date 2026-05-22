#ifndef APRILTAG3_ROS__APRILTAG_DETECTOR_HPP_
#define APRILTAG3_ROS__APRILTAG_DETECTOR_HPP_

#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <image_transport/image_transport.hpp>

#include <apriltag3_msgs/msg/april_tag_detection_array.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "apriltag3_ros/apriltag_detector_parameters.hpp"
#include "apriltag3_ros/detector.hpp"

namespace apriltag3_ros
{

class AprilTagDetector : public rclcpp::Node
{
public:
  explicit AprilTagDetector(const rclcpp::NodeOptions & options);
  ~AprilTagDetector() override;

private:
  void onImage(
    const sensor_msgs::msg::Image::ConstSharedPtr & image,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr & camera_info);

  std::shared_ptr<apriltag3_ros::ParamListener> param_listener_;
  apriltag3_ros::Params params_;

  std::unique_ptr<Detector> detector_;

  image_transport::CameraSubscriber camera_sub_;
  bool info_validated_ = false;

  rclcpp::Publisher<apriltag3_msgs::msg::AprilTagDetectionArray>::SharedPtr
    detection_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

}  // namespace apriltag3_ros

#endif  // APRILTAG3_ROS__APRILTAG_DETECTOR_HPP_
