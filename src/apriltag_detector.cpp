#include "apriltag3_ros/apriltag_detector.hpp"

#include <algorithm>
#include <cmath>
#include <string>

#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <image_transport/camera_common.hpp>
#include <image_transport/image_transport.hpp>
#include <image_transport/transport_hints.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

namespace apriltag3_ros
{

namespace
{

geometry_msgs::msg::Quaternion rotationToQuaternion(
  const std::array<double, 9> & R)
{
  tf2::Matrix3x3 m(
    R[0], R[1], R[2],
    R[3], R[4], R[5],
    R[6], R[7], R[8]);
  tf2::Quaternion q;
  m.getRotation(q);
  geometry_msgs::msg::Quaternion out;
  out.x = q.x();
  out.y = q.y();
  out.z = q.z();
  out.w = q.w();
  return out;
}

apriltag3_msgs::msg::AprilTagDetection toMsg(const Detection & d)
{
  apriltag3_msgs::msg::AprilTagDetection m;
  m.group = d.group;
  m.family = d.family;
  m.id = d.id;
  m.tf_frame_id = d.tag_frame_prefix + std::to_string(d.id);
  m.hamming = d.hamming;
  m.decision_margin = d.decision_margin;
  m.size = d.size;

  m.center.x = d.center[0];
  m.center.y = d.center[1];
  m.center.z = 0.0;

  for (size_t i = 0; i < d.corners.size(); ++i) {
    m.corners[i].x = d.corners[i][0];
    m.corners[i].y = d.corners[i][1];
    m.corners[i].z = 0.0;
  }

  if (d.pose) {
    m.pose.pose.position.x = d.pose->t[0];
    m.pose.pose.position.y = d.pose->t[1];
    m.pose.pose.position.z = d.pose->t[2];
    m.pose.pose.orientation = rotationToQuaternion(d.pose->R);
    for (size_t i = 0; i < d.pose->covariance.size(); ++i) {
      m.pose.covariance[i] = d.pose->covariance[i];
    }
    m.pose_error = d.pose->err;
  } else {
    m.pose.pose.orientation.w = 1.0;
    m.pose_error = 0.0;
  }

  return m;
}

}  // namespace

AprilTagDetector::AprilTagDetector(const rclcpp::NodeOptions & options)
: rclcpp::Node("apriltag_detector", options)
{
  param_listener_ =
    std::make_shared<apriltag3_ros::ParamListener>(get_node_parameters_interface());
  params_ = param_listener_->get_params();

  detector_ = std::make_unique<Detector>(params_);

  detection_pub_ = create_publisher<apriltag3_msgs::msg::AprilTagDetectionArray>(
    "detections", rclcpp::SensorDataQoS());

  if (params_.publish_tf) {
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  }

  image_transport::TransportHints hints(this);
  camera_sub_ = image_transport::create_camera_subscription(
    this, params_.image_topic,
    std::bind(
      &AprilTagDetector::onImage, this,
      std::placeholders::_1, std::placeholders::_2),
    hints.getTransport());

  const std::string resolved_info_topic =
    image_transport::getCameraInfoTopic(camera_sub_.getTopic());

  RCLCPP_INFO(
    get_logger(),
    "AprilTagDetector ready: image='%s', camera_info='%s', groups=%zu, "
    "pose_method='%s', publish_tf=%s",
    camera_sub_.getTopic().c_str(), resolved_info_topic.c_str(),
    params_.tag_groups.size(), params_.pose_method.c_str(),
    params_.publish_tf ? "true" : "false");
}

AprilTagDetector::~AprilTagDetector() = default;

void AprilTagDetector::onImage(
  const sensor_msgs::msg::Image::ConstSharedPtr & image,
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr & camera_info)
{
  if (!info_validated_) {
    info_validated_ = true;
    // P[0]/P[5] are fx/fy of the rectified projection. Zero means the
    // camera_info is uncalibrated; pose output will be invalid.
    if (camera_info->p[0] == 0.0 || camera_info->p[5] == 0.0) {
      RCLCPP_WARN(
        get_logger(),
        "CameraInfo P matrix appears uncalibrated (fx=%.3f, fy=%.3f). "
        "Pose estimates will be invalid until a rectified camera_info arrives.",
        camera_info->p[0], camera_info->p[5]);
    }
    // A rectified stream should carry D == 0. Non-zero suggests the input
    // is the raw camera topic, not image_rect — corners will be off.
    double dmax = 0.0;
    for (double v : camera_info->d) {
      dmax = std::max(dmax, std::abs(v));
    }
    if (dmax > 1e-6) {
      RCLCPP_WARN(
        get_logger(),
        "Input camera_info has non-zero distortion (|D|_max=%.4f). "
        "AprilTagDetector expects rectified input (e.g. image_proc/rectify_node "
        "output); raw images will produce sub-pixel corner errors.",
        dmax);
    }
  }

  const auto detections = detector_->detect(image, camera_info);

  apriltag3_msgs::msg::AprilTagDetectionArray msg;
  msg.header = image->header;
  msg.detections.reserve(detections.size());
  for (const auto & d : detections) {
    msg.detections.push_back(toMsg(d));
  }
  detection_pub_->publish(msg);

  if (!tf_broadcaster_) {
    return;
  }

  std::vector<geometry_msgs::msg::TransformStamped> transforms;
  transforms.reserve(detections.size());
  for (const auto & d : detections) {
    if (!d.pose) {
      continue;
    }
    geometry_msgs::msg::TransformStamped tf;
    tf.header = image->header;
    tf.child_frame_id = d.tag_frame_prefix + std::to_string(d.id);
    tf.transform.translation.x = d.pose->t[0];
    tf.transform.translation.y = d.pose->t[1];
    tf.transform.translation.z = d.pose->t[2];
    tf.transform.rotation = rotationToQuaternion(d.pose->R);
    transforms.push_back(std::move(tf));
  }
  if (!transforms.empty()) {
    tf_broadcaster_->sendTransform(transforms);
  }
}

}  // namespace apriltag3_ros

RCLCPP_COMPONENTS_REGISTER_NODE(apriltag3_ros::AprilTagDetector)
