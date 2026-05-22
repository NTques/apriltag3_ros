#ifndef APRILTAG3_ROS__DETECTOR_HPP_
#define APRILTAG3_ROS__DETECTOR_HPP_

#include <array>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "apriltag3_ros/apriltag_detector_parameters.hpp"

// Forward declarations from the apriltag C library: keep this header
// independent of <apriltag/apriltag.h>.
extern "C" {
typedef struct apriltag_detector apriltag_detector_t;
typedef struct apriltag_family apriltag_family_t;
}

namespace apriltag3_ros
{

struct Pose
{
  std::array<double, 9> R{};
  std::array<double, 3> t{};

  // 6x6 row-major covariance in ROS PoseWithCovariance ordering
  // [x, y, z, rx, ry, rz]. Linearized from the corner-reprojection
  // Jacobian (J^T J)^-1 scaled by residual-based sigma^2 with a small
  // noise floor to avoid overconfidence.
  std::array<double, 36> covariance{};

  // Method-dependent error metric.
  //   ippe_square / iterative : RMS reprojection error (pixels)
  //   orthogonal_iteration    : object-space error (apriltag units)
  //   homography              : 0 (estimator returns no error)
  double err = 0.0;
};

struct Detection
{
  std::string group;
  std::string family;
  int id;
  int hamming;
  float decision_margin;
  std::array<double, 2> center;
  std::array<std::array<double, 2>, 4> corners;
  double size;

  // Resolved per-group TF child-frame prefix (group override or global default).
  std::string tag_frame_prefix;

  std::optional<Pose> pose;
};

class Detector
{
public:
  explicit Detector(const apriltag3_ros::Params & params);
  ~Detector();

  Detector(const Detector &) = delete;
  Detector & operator=(const Detector &) = delete;

  std::vector<Detection> detect(
    const sensor_msgs::msg::Image::ConstSharedPtr & image,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr & camera_info);

private:
  struct Group
  {
    std::string name;
    std::string family;
    int id_begin;
    int id_end;
    double size;
    std::string tag_frame_prefix;
  };

  const Group * findGroup(const std::string & family, int id) const;

  apriltag_detector_t * td_;
  std::vector<std::pair<std::string, apriltag_family_t *>> families_;
  std::vector<Group> groups_;

  std::string pose_method_;
  double decision_margin_min_;
};

}  // namespace apriltag3_ros

#endif  // APRILTAG3_ROS__DETECTOR_HPP_
