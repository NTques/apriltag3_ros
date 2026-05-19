#pragma once

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <image_transport/image_transport.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <diagnostic_updater/diagnostic_updater.hpp>

#include <apriltag3_msgs/msg/april_tag_detection_array.hpp>

#include "apriltag3_ros/detector.hpp"
#include "apriltag3_ros/pose_solver.hpp"
#include "apriltag3_ros/tag_config.hpp"

namespace apriltag3_ros
{

// Forward declaration of the parameter library generated type.
class ParamListener;
struct Params;

class AprilTag3RosNode : public rclcpp::Node
{
public:
  explicit AprilTag3RosNode(const rclcpp::NodeOptions & options);
  ~AprilTag3RosNode() override;

private:
  void on_image(const sensor_msgs::msg::Image::ConstSharedPtr & msg);
  void on_camera_info(const sensor_msgs::msg::CameraInfo::ConstSharedPtr & msg);

  // One-shot construction of the detector + pose solver (called from the
  // constructor). Throws on failure.
  void init_pipeline();

  // Re-apply dynamic (non-read-only) parameters to existing detector/solver.
  void refresh_dynamic_params();

  void update_diagnostics(diagnostic_updater::DiagnosticStatusWrapper & stat);

  // --- parameters / configuration ----------------------------------------
  std::unique_ptr<ParamListener> param_listener_;
  std::unique_ptr<Params> params_;

  TagConfig tag_config_;
  std::unique_ptr<Detector> detector_;
  std::unique_ptr<PoseSolver> pose_solver_;
  PoseSolver::CameraIntrinsics intrinsics_override_;
  bool use_camera_info_{true};

  // --- ROS I/O -----------------------------------------------------------
  image_transport::Subscriber image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp::Publisher<apriltag3_msgs::msg::AprilTagDetectionArray>::SharedPtr detections_pub_;
  image_transport::Publisher detections_image_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // Latest CameraInfo (used when use_camera_info is true).
  std::mutex camera_info_mutex_;
  std::optional<sensor_msgs::msg::CameraInfo> latest_camera_info_;

  // --- diagnostics + stats ----------------------------------------------
  std::unique_ptr<diagnostic_updater::Updater> diag_updater_;
  rclcpp::TimerBase::SharedPtr stats_timer_;
  std::atomic<uint64_t> frames_processed_{0};
  std::atomic<uint64_t> total_detections_{0};
  std::atomic<size_t> last_detection_count_{0};
  std::mutex stats_mutex_;
  rclcpp::Time last_frame_stamp_{0, 0, RCL_ROS_TIME};
  double measured_fps_{0.0};
  uint64_t frames_at_last_sample_{0};
  rclcpp::Time last_sample_stamp_{0, 0, RCL_ROS_TIME};
};

}  // namespace apriltag3_ros
