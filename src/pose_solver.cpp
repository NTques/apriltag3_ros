#include "apriltag3_ros/pose_solver.hpp"

#include <stdexcept>
#include <vector>

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>

namespace apriltag3_ros
{

PoseSolver::PoseSolver(Method method, int oi_iterations)
: method_(method), oi_iterations_(oi_iterations) {}

PoseSolver::Method PoseSolver::parse_method(const std::string & name)
{
  if (name == "homography") {
    return Method::Homography;
  }
  if (name == "orthogonal_iteration") {
    return Method::OrthogonalIteration;
  }
  if (name == "opencv_ippe_square") {
    return Method::OpenCvIppeSquare;
  }
  throw std::invalid_argument("Unknown pose_method: " + name);
}

const char * PoseSolver::method_name(Method m)
{
  switch (m) {
    case Method::Homography: return "homography";
    case Method::OrthogonalIteration: return "orthogonal_iteration";
    case Method::OpenCvIppeSquare: return "opencv_ippe_square";
  }
  return "unknown";
}

namespace
{

PoseSolver::Pose to_pose(const apriltag_pose_t & p, double err)
{
  PoseSolver::Pose out;
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      out.R[i * 3 + j] = matd_get(p.R, i, j);
    }
    out.t[i] = matd_get(p.t, i, 0);
  }
  out.error = err;
  return out;
}

void destroy_pose(apriltag_pose_t & p)
{
  if (p.R) {matd_destroy(p.R);}
  if (p.t) {matd_destroy(p.t);}
  p.R = nullptr;
  p.t = nullptr;
}

}  // namespace

PoseSolver::Pose PoseSolver::solve(
  const apriltag_detection_t * det,
  double tagsize,
  const CameraIntrinsics & K) const
{
  if (!det) {
    throw std::invalid_argument("PoseSolver::solve: null detection");
  }
  if (tagsize <= 0.0) {
    throw std::invalid_argument("PoseSolver::solve: tagsize must be positive");
  }

  if (method_ == Method::OpenCvIppeSquare) {
    return solve_opencv_ippe_square(det, tagsize, K);
  }

  apriltag_detection_info_t info{
    const_cast<apriltag_detection_t *>(det),
    tagsize, K.fx, K.fy, K.cx, K.cy,
  };

  switch (method_) {
    case Method::Homography:
      return solve_homography(info);
    case Method::OrthogonalIteration:
      return solve_orthogonal_iteration(info);
    default:
      throw std::logic_error("PoseSolver::solve: unreachable");
  }
}

PoseSolver::Pose PoseSolver::solve_homography(const apriltag_detection_info_t & info) const
{
  apriltag_pose_t p{};
  estimate_pose_for_tag_homography(
    const_cast<apriltag_detection_info_t *>(&info), &p);
  Pose out = to_pose(p, 0.0);
  destroy_pose(p);
  return out;
}

PoseSolver::Pose PoseSolver::solve_orthogonal_iteration(
  const apriltag_detection_info_t & info) const
{
  // estimate_tag_pose internally runs OI with two initial estimates and
  // returns the lower-error pose. It uses the library's default iteration
  // count, so the oi_iterations parameter is informational only.
  apriltag_pose_t p{};
  const double err = estimate_tag_pose(
    const_cast<apriltag_detection_info_t *>(&info), &p);
  Pose out = to_pose(p, err);
  destroy_pose(p);
  (void)oi_iterations_;
  return out;
}

PoseSolver::Pose PoseSolver::solve_opencv_ippe_square(
  const apriltag_detection_t * det,
  double tagsize,
  const CameraIntrinsics & K) const
{
  // Object points in tag coordinates. Order matches AprilTag's corner order:
  // p[0..3] wrap counter-clockwise; with the AprilTag convention
  // p[0] = (-s/2, -s/2), p[1] = (s/2, -s/2), p[2] = (s/2, s/2), p[3] = (-s/2, s/2).
  // (Any consistent ordering works for IPPE_SQUARE, as long as object and
  // image points correspond.)
  const double s = tagsize / 2.0;
  std::vector<cv::Point3d> obj_pts = {
    {-s, -s, 0.0},
    {s, -s, 0.0},
    {s, s, 0.0},
    {-s, s, 0.0},
  };
  std::vector<cv::Point2d> img_pts = {
    {det->p[0][0], det->p[0][1]},
    {det->p[1][0], det->p[1][1]},
    {det->p[2][0], det->p[2][1]},
    {det->p[3][0], det->p[3][1]},
  };

  cv::Matx33d cam_K(
    K.fx, 0.0, K.cx,
    0.0, K.fy, K.cy,
    0.0, 0.0, 1.0);
  cv::Mat dist = cv::Mat::zeros(4, 1, CV_64F);

  cv::Mat rvec, tvec;
  const bool ok = cv::solvePnP(
    obj_pts, img_pts, cam_K, dist, rvec, tvec,
    /*useExtrinsicGuess=*/ false, cv::SOLVEPNP_IPPE_SQUARE);
  if (!ok) {
    throw std::runtime_error("cv::solvePnP(SOLVEPNP_IPPE_SQUARE) failed");
  }

  cv::Mat R;
  cv::Rodrigues(rvec, R);

  // Reprojection-error proxy (RMS in pixels).
  std::vector<cv::Point2d> reproj;
  cv::projectPoints(obj_pts, rvec, tvec, cam_K, dist, reproj);
  double sse = 0.0;
  for (size_t i = 0; i < reproj.size(); ++i) {
    const double dx = reproj[i].x - img_pts[i].x;
    const double dy = reproj[i].y - img_pts[i].y;
    sse += dx * dx + dy * dy;
  }
  const double rms = std::sqrt(sse / static_cast<double>(reproj.size()));

  Pose out;
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      out.R[i * 3 + j] = R.at<double>(i, j);
    }
    out.t[i] = tvec.at<double>(i, 0);
  }
  out.error = rms;
  return out;
}

std::array<double, 36> PoseSolver::compute_covariance(
  const Pose & pose,
  double tagsize,
  const CameraIntrinsics & K,
  double pixel_sigma)
{
  std::array<double, 36> out{};

  if (tagsize <= 0.0 || pixel_sigma <= 0.0) {
    return out;
  }

  const double s = tagsize / 2.0;
  const std::vector<cv::Point3d> obj_pts = {
    {-s, -s, 0.0}, {s, -s, 0.0}, {s, s, 0.0}, {-s, s, 0.0},
  };

  cv::Mat R(3, 3, CV_64F);
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      R.at<double>(i, j) = pose.R[i * 3 + j];
    }
  }
  cv::Mat rvec;
  cv::Rodrigues(R, rvec);
  cv::Mat tvec = (cv::Mat_<double>(3, 1) << pose.t[0], pose.t[1], pose.t[2]);

  cv::Matx33d Kmat(K.fx, 0.0, K.cx, 0.0, K.fy, K.cy, 0.0, 0.0, 1.0);
  cv::Mat dist = cv::Mat::zeros(4, 1, CV_64F);

  std::vector<cv::Point2d> base_proj;
  cv::projectPoints(obj_pts, rvec, tvec, Kmat, dist, base_proj);

  // 8 (4 corners x 2 px) x 6 (tx,ty,tz,rx,ry,rz) Jacobian via central diff.
  cv::Mat J(8, 6, CV_64F, cv::Scalar(0.0));
  const double eps = 1e-6;
  for (int dof = 0; dof < 6; ++dof) {
    cv::Mat r_p = rvec.clone();
    cv::Mat t_p = tvec.clone();
    cv::Mat r_m = rvec.clone();
    cv::Mat t_m = tvec.clone();
    if (dof < 3) {
      t_p.at<double>(dof, 0) += eps;
      t_m.at<double>(dof, 0) -= eps;
    } else {
      r_p.at<double>(dof - 3, 0) += eps;
      r_m.at<double>(dof - 3, 0) -= eps;
    }
    std::vector<cv::Point2d> p_plus, p_minus;
    cv::projectPoints(obj_pts, r_p, t_p, Kmat, dist, p_plus);
    cv::projectPoints(obj_pts, r_m, t_m, Kmat, dist, p_minus);
    for (int i = 0; i < 4; ++i) {
      J.at<double>(2 * i, dof) = (p_plus[i].x - p_minus[i].x) / (2.0 * eps);
      J.at<double>(2 * i + 1, dof) = (p_plus[i].y - p_minus[i].y) / (2.0 * eps);
    }
  }

  // cov = sigma_px^2 * (J^T J)^-1
  cv::Mat JtJ = J.t() * J;
  cv::Mat cov;
  if (!cv::invert(JtJ, cov, cv::DECOMP_SVD)) {
    return out;
  }
  cov *= pixel_sigma * pixel_sigma;

  for (int i = 0; i < 6; ++i) {
    for (int j = 0; j < 6; ++j) {
      out[i * 6 + j] = cov.at<double>(i, j);
    }
  }
  return out;
}

}  // namespace apriltag3_ros
