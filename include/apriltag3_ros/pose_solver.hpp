#pragma once

#include <array>
#include <string>

extern "C" {
#include <apriltag.h>
#include <apriltag_pose.h>
}

namespace apriltag3_ros
{

class PoseSolver
{
public:
  enum class Method
  {
    Homography,
    OrthogonalIteration,
    OpenCvIppeSquare,
  };

  struct CameraIntrinsics
  {
    double fx{0.0};
    double fy{0.0};
    double cx{0.0};
    double cy{0.0};
  };

  // Tag pose expressed in the camera optical frame.
  struct Pose
  {
    // Row-major 3x3 rotation.
    std::array<double, 9> R{};
    // 3-vector translation in meters.
    std::array<double, 3> t{};
    // Solver-reported residual (object-space for OI, reprojection-like for IPPE).
    double error{0.0};
  };

  PoseSolver(Method method, int oi_iterations);

  void set_method(Method m) {method_ = m;}
  void set_oi_iterations(int n) {oi_iterations_ = n;}
  Method method() const {return method_;}

  static Method parse_method(const std::string & name);
  static const char * method_name(Method m);

  Pose solve(
    const apriltag_detection_t * det,
    double tagsize,
    const CameraIntrinsics & K) const;

  // 6x6 covariance in row-major order, axes (x, y, z, rot_x, rot_y, rot_z).
  // For small uncertainties this is interchangeable with (x,y,z,roll,pitch,yaw).
  // Computed by linearizing the pinhole projection of the 4 tag corners around
  // the supplied pose; the result scales with pixel_sigma^2.
  static std::array<double, 36> compute_covariance(
    const Pose & pose,
    double tagsize,
    const CameraIntrinsics & K,
    double pixel_sigma);

private:
  Pose solve_homography(const apriltag_detection_info_t & info) const;
  Pose solve_orthogonal_iteration(const apriltag_detection_info_t & info) const;
  Pose solve_opencv_ippe_square(
    const apriltag_detection_t * det,
    double tagsize,
    const CameraIntrinsics & K) const;

  Method method_;
  int oi_iterations_;
};

}  // namespace apriltag3_ros
