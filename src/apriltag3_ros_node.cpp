#include "apriltag3_ros/apriltag3_ros_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <exception>
#include <utility>

#include <cv_bridge/cv_bridge.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include "apriltag3_ros/apriltag3_ros_parameters.hpp"

namespace apriltag3_ros
{

namespace
{

rclcpp::QoS make_qos(
  const std::string & reliability,
  const std::string & history, int depth)
{
  rclcpp::QoS qos = (history == "keep_all") ?
    rclcpp::QoS(rclcpp::KeepAll()) :
    rclcpp::QoS(rclcpp::KeepLast(depth));
  if (reliability == "best_effort") {
    qos.best_effort();
  } else {
    qos.reliable();
  }
  return qos;
}

void rotation_to_quaternion(
  const std::array<double, 9> & R,
  geometry_msgs::msg::Quaternion & q)
{
  tf2::Matrix3x3 m(
    R[0], R[1], R[2],
    R[3], R[4], R[5],
    R[6], R[7], R[8]);
  tf2::Quaternion tq;
  m.getRotation(tq);
  q.x = tq.x();
  q.y = tq.y();
  q.z = tq.z();
  q.w = tq.w();
}

}  // namespace

AprilTag3RosNode::AprilTag3RosNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("apriltag3_ros", options)
{
  param_listener_ = std::make_unique<ParamListener>(get_node_parameters_interface());
  params_ = std::make_unique<Params>(param_listener_->get_params());

  init_pipeline();

  // --- QoS / topics ----------------------------------------------------
  const auto qos = make_qos(params_->qos.reliability, params_->qos.history, params_->qos.depth);

  detections_pub_ = create_publisher<apriltag3_msgs::msg::AprilTagDetectionArray>(
    params_->detections_topic, qos);

  if (params_->publish_tf) {
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  }

  camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
    params_->camera_info_topic, qos,
    std::bind(&AprilTag3RosNode::on_camera_info, this, std::placeholders::_1));

  image_transport::TransportHints hints(this, params_->image_transport);
  image_sub_ = image_transport::create_subscription(
    this, params_->image_topic,
    std::bind(&AprilTag3RosNode::on_image, this, std::placeholders::_1),
    hints.getTransport(),
    qos.get_rmw_qos_profile());

  if (params_->publish_detections_image) {
    detections_image_pub_ = image_transport::create_publisher(
      this, params_->detections_image_topic);
  }

  // --- diagnostics + stats ---------------------------------------------
  if (params_->diagnostics.enable) {
    diag_updater_ = std::make_unique<diagnostic_updater::Updater>(this);
    diag_updater_->setHardwareID(get_name());
    diag_updater_->add(
      "AprilTag3 Detector",
      std::bind(&AprilTag3RosNode::update_diagnostics, this, std::placeholders::_1));
  }

  const auto log_period =
    std::chrono::duration<double>(params_->diagnostics.log_period_sec);
  last_sample_stamp_ = now();
  stats_timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(log_period),
    [this]() {
      const auto current = now();
      const double dt = (current - last_sample_stamp_).seconds();
      const uint64_t frames_now = frames_processed_.load();
      const uint64_t delta = frames_now - frames_at_last_sample_;
      if (dt > 0.0) {
        std::lock_guard<std::mutex> lk(stats_mutex_);
        measured_fps_ = static_cast<double>(delta) / dt;
      }
      frames_at_last_sample_ = frames_now;
      last_sample_stamp_ = current;
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
        "fps=%.1f frames=%lu detections=%lu (last=%zu)",
        measured_fps_, frames_now,
        total_detections_.load(), last_detection_count_.load());
    });

  RCLCPP_INFO(get_logger(),
    "apriltag3_ros initialized | families=%zu | pose_method=%s | "
    "estimate_pose=%s | publish_tf=%s | diagnostics=%s",
    params_->families.size(),
    PoseSolver::method_name(PoseSolver::parse_method(params_->pose_method)),
    params_->estimate_pose ? "true" : "false",
    params_->publish_tf ? "true" : "false",
    params_->diagnostics.enable ? "true" : "false");
}

AprilTag3RosNode::~AprilTag3RosNode() = default;

void AprilTag3RosNode::init_pipeline()
{
  tag_config_ = make_tag_config(
    params_->tag.default_size,
    params_->tag.ids,
    params_->tag.sizes_ids,
    params_->tag.sizes_values,
    params_->tag.frames_ids,
    params_->tag.frames_values,
    params_->tf.tag_frame_prefix);

  Detector::Params dp;
  dp.threads = params_->detector.threads;
  dp.decimate = params_->detector.decimate;
  dp.blur = params_->detector.blur;
  dp.refine_edges = params_->detector.refine_edges;
  dp.sharpening = params_->detector.sharpening;
  dp.debug = params_->detector.debug;
  dp.hamming = params_->detector.hamming;
  dp.qtp_min_cluster_pixels = params_->detector.qtp.min_cluster_pixels;
  dp.qtp_max_nmaxima = params_->detector.qtp.max_nmaxima;
  dp.qtp_critical_rad = params_->detector.qtp.critical_rad;
  dp.qtp_max_line_fit_mse = params_->detector.qtp.max_line_fit_mse;
  dp.qtp_min_white_black_diff = params_->detector.qtp.min_white_black_diff;
  dp.qtp_deglitch = params_->detector.qtp.deglitch;
  detector_ = std::make_unique<Detector>(params_->families, dp);

  pose_solver_ = std::make_unique<PoseSolver>(
    PoseSolver::parse_method(params_->pose_method),
    params_->pose_iterations);

  use_camera_info_ = params_->camera.use_camera_info;
  intrinsics_override_.fx = params_->camera.fx;
  intrinsics_override_.fy = params_->camera.fy;
  intrinsics_override_.cx = params_->camera.cx;
  intrinsics_override_.cy = params_->camera.cy;
}

void AprilTag3RosNode::refresh_dynamic_params()
{
  *params_ = param_listener_->get_params();

  // detector tuning (everything except threads/hamming/families is in-place).
  Detector::Params dp;
  dp.decimate = params_->detector.decimate;
  dp.blur = params_->detector.blur;
  dp.refine_edges = params_->detector.refine_edges;
  dp.sharpening = params_->detector.sharpening;
  dp.debug = params_->detector.debug;
  dp.qtp_min_cluster_pixels = params_->detector.qtp.min_cluster_pixels;
  dp.qtp_max_nmaxima = params_->detector.qtp.max_nmaxima;
  dp.qtp_critical_rad = params_->detector.qtp.critical_rad;
  dp.qtp_max_line_fit_mse = params_->detector.qtp.max_line_fit_mse;
  dp.qtp_min_white_black_diff = params_->detector.qtp.min_white_black_diff;
  dp.qtp_deglitch = params_->detector.qtp.deglitch;
  detector_->apply_dynamic_params(dp);

  // pose solver
  pose_solver_->set_method(PoseSolver::parse_method(params_->pose_method));
  pose_solver_->set_oi_iterations(params_->pose_iterations);

  // tag config
  tag_config_ = make_tag_config(
    params_->tag.default_size,
    params_->tag.ids,
    params_->tag.sizes_ids,
    params_->tag.sizes_values,
    params_->tag.frames_ids,
    params_->tag.frames_values,
    params_->tf.tag_frame_prefix);

  // camera intrinsics override + frame
  use_camera_info_ = params_->camera.use_camera_info;
  intrinsics_override_.fx = params_->camera.fx;
  intrinsics_override_.fy = params_->camera.fy;
  intrinsics_override_.cx = params_->camera.cx;
  intrinsics_override_.cy = params_->camera.cy;

  RCLCPP_INFO(get_logger(),
    "Refreshed dynamic params | pose_method=%s | default_size=%.4f m",
    params_->pose_method.c_str(), params_->tag.default_size);
}

void AprilTag3RosNode::on_camera_info(
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr & msg)
{
  std::lock_guard<std::mutex> lk(camera_info_mutex_);
  latest_camera_info_ = *msg;
}

void AprilTag3RosNode::on_image(
  const sensor_msgs::msg::Image::ConstSharedPtr & msg)
{
  // Apply any queued dynamic parameter updates.
  if (param_listener_->is_old(*params_)) {
    try {
      refresh_dynamic_params();
    } catch (const std::exception & e) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
        "refresh_dynamic_params failed: %s", e.what());
    }
  }

  cv::Mat gray;
  try {
    auto cv_ptr = cv_bridge::toCvShare(msg, "mono8");
    gray = cv_ptr->image;
  } catch (const cv_bridge::Exception & e) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
      "cv_bridge conversion failed: %s", e.what());
    return;
  }

  // ROI crop (parameter [x, y, w, h], all-zero = full image).
  const auto & roi = params_->filter.roi;
  cv::Mat work = gray;
  int roi_x = 0;
  int roi_y = 0;
  if (roi.size() == 4 && (roi[2] > 0 && roi[3] > 0)) {
    roi_x = std::clamp<int>(static_cast<int>(roi[0]), 0, gray.cols - 1);
    roi_y = std::clamp<int>(static_cast<int>(roi[1]), 0, gray.rows - 1);
    const int w = std::min<int>(static_cast<int>(roi[2]), gray.cols - roi_x);
    const int h = std::min<int>(static_cast<int>(roi[3]), gray.rows - roi_y);
    if (w > 0 && h > 0) {
      work = gray(cv::Rect(roi_x, roi_y, w, h));
    }
  }

  // Intrinsics: prefer CameraInfo when configured, else override params.
  PoseSolver::CameraIntrinsics K = intrinsics_override_;
  std::string camera_frame = params_->tf.camera_frame.empty() ?
    msg->header.frame_id : params_->tf.camera_frame;

  if (use_camera_info_) {
    std::lock_guard<std::mutex> lk(camera_info_mutex_);
    if (!latest_camera_info_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "Waiting for camera_info on '%s'", params_->camera_info_topic.c_str());
      return;
    }
    K.fx = latest_camera_info_->k[0];
    K.fy = latest_camera_info_->k[4];
    K.cx = latest_camera_info_->k[2];
    K.cy = latest_camera_info_->k[5];
    if (camera_frame.empty()) {
      camera_frame = latest_camera_info_->header.frame_id;
    }
  }

  Detector::Detections dets = detector_->detect(work);

  apriltag3_msgs::msg::AprilTagDetectionArray out;
  out.header = msg->header;
  if (!params_->tf.camera_frame.empty()) {
    out.header.frame_id = params_->tf.camera_frame;
  }
  out.detections.reserve(dets.size());

  std::vector<geometry_msgs::msg::TransformStamped> tfs;
  tfs.reserve(dets.size());

  for (size_t i = 0; i < dets.size(); ++i) {
    const apriltag_detection_t * det = dets[i];

    if (!tag_config_.is_allowed(det->id)) {continue;}
    if (det->decision_margin < params_->filter.min_decision_margin) {continue;}
    if (det->hamming > params_->filter.max_hamming) {continue;}

    apriltag3_msgs::msg::AprilTagDetection d;
    d.id = det->id;
    d.family = det->family->name ? det->family->name : "";
    d.hamming = det->hamming;
    d.decision_margin = det->decision_margin;
    d.size = tag_config_.size_for(det->id);
    d.center.x = det->c[0] + roi_x;
    d.center.y = det->c[1] + roi_y;
    d.center.z = 0.0;
    for (int k = 0; k < 4; ++k) {
      d.corners[k].x = det->p[k][0] + roi_x;
      d.corners[k].y = det->p[k][1] + roi_y;
      d.corners[k].z = 0.0;
    }

    if (params_->estimate_pose) {
      try {
        // PoseSolver expects intrinsics matching the image fed to the detector.
        // When using a ROI, the principal point shifts.
        PoseSolver::CameraIntrinsics Kroi = K;
        Kroi.cx -= roi_x;
        Kroi.cy -= roi_y;
        const auto pose = pose_solver_->solve(det, d.size, Kroi);

        d.pose.pose.position.x = pose.t[0];
        d.pose.pose.position.y = pose.t[1];
        d.pose.pose.position.z = pose.t[2];
        rotation_to_quaternion(pose.R, d.pose.pose.orientation);
        d.pose_error = pose.error;

        if (params_->covariance.enable) {
          d.pose.covariance = PoseSolver::compute_covariance(
            pose, d.size, Kroi, params_->covariance.pixel_sigma);
        }

        if (tf_broadcaster_) {
          geometry_msgs::msg::TransformStamped tf;
          tf.header = msg->header;
          if (!camera_frame.empty()) {
            tf.header.frame_id = camera_frame;
          }
          tf.child_frame_id = tag_config_.frame_for(det->id);
          tf.transform.translation.x = pose.t[0];
          tf.transform.translation.y = pose.t[1];
          tf.transform.translation.z = pose.t[2];
          rotation_to_quaternion(pose.R, tf.transform.rotation);
          tfs.push_back(std::move(tf));
        }
      } catch (const std::exception & e) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
          "Pose estimation failed for id=%d: %s", det->id, e.what());
      }
    }

    out.detections.push_back(std::move(d));
    if (params_->filter.max_detections_per_frame > 0 &&
      static_cast<int>(out.detections.size()) >= params_->filter.max_detections_per_frame)
    {
      break;
    }
  }

  detections_pub_->publish(out);
  if (tf_broadcaster_ && !tfs.empty()) {
    tf_broadcaster_->sendTransform(tfs);
  }

  if (params_->publish_detections_image && detections_image_pub_.getNumSubscribers() > 0) {
    cv::Mat color;
    cv::cvtColor(gray, color, cv::COLOR_GRAY2BGR);
    for (const auto & d : out.detections) {
      for (int k = 0; k < 4; ++k) {
        const auto & a = d.corners[k];
        const auto & b = d.corners[(k + 1) % 4];
        cv::line(color,
          cv::Point2d(a.x, a.y), cv::Point2d(b.x, b.y),
          cv::Scalar(0, 255, 0), 2);
      }
      cv::circle(color, cv::Point2d(d.center.x, d.center.y), 4,
        cv::Scalar(0, 0, 255), -1);
      cv::putText(color, std::to_string(d.id),
        cv::Point2d(d.center.x + 6, d.center.y - 6),
        cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 255), 2);
    }
    auto img_msg = cv_bridge::CvImage(msg->header, "bgr8", color).toImageMsg();
    detections_image_pub_.publish(img_msg);
  }

  // --- stats ------------------------------------------------------------
  frames_processed_.fetch_add(1);
  total_detections_.fetch_add(out.detections.size());
  last_detection_count_.store(out.detections.size());
  {
    std::lock_guard<std::mutex> lk(stats_mutex_);
    last_frame_stamp_ = now();
  }
}

void AprilTag3RosNode::update_diagnostics(
  diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  double fps;
  rclcpp::Time last;
  {
    std::lock_guard<std::mutex> lk(stats_mutex_);
    fps = measured_fps_;
    last = last_frame_stamp_;
  }

  const auto now_t = now();
  const double age = last.nanoseconds() == 0 ?
    std::numeric_limits<double>::infinity() :
    (now_t - last).seconds();

  using Wrapper = diagnostic_updater::DiagnosticStatusWrapper;
  const double target = params_->diagnostics.target_fps;
  if (frames_processed_.load() == 0) {
    stat.summary(Wrapper::WARN, "No frames processed yet");
  } else if (age > 2.0) {
    stat.summary(Wrapper::ERROR, "Detection stale (no frames in 2s)");
  } else if (fps < target * 0.5) {
    stat.summary(Wrapper::WARN, "Detection FPS below half the target");
  } else {
    stat.summary(Wrapper::OK, "Running");
  }

  stat.add("families", static_cast<int>(params_->families.size()));
  stat.add("pose_method", params_->pose_method);
  stat.add("estimate_pose", params_->estimate_pose);
  stat.add("measured_fps", fps);
  stat.add("target_fps", target);
  stat.add("frames_processed", static_cast<uint64_t>(frames_processed_.load()));
  stat.add("total_detections", static_cast<uint64_t>(total_detections_.load()));
  stat.add("last_detection_count", static_cast<uint64_t>(last_detection_count_.load()));
  stat.add("last_frame_age_sec", age);
  stat.add("use_camera_info", use_camera_info_);
}

}  // namespace apriltag3_ros

RCLCPP_COMPONENTS_REGISTER_NODE(apriltag3_ros::AprilTag3RosNode)
