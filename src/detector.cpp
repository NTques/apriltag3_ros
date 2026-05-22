#include "apriltag3_ros/detector.hpp"

#include <cmath>
#include <stdexcept>
#include <unordered_map>

#include <cv_bridge/cv_bridge.hpp>
#include <image_geometry/pinhole_camera_model.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>

#include <apriltag/apriltag.h>
#include <apriltag/apriltag_pose.h>
#include <apriltag/common/matd.h>
#include <apriltag/common/zarray.h>

#include <apriltag/tag16h5.h>
#include <apriltag/tag25h9.h>
#include <apriltag/tag36h10.h>
#include <apriltag/tag36h11.h>
#include <apriltag/tagCircle21h7.h>
#include <apriltag/tagCircle49h12.h>
#include <apriltag/tagCustom48h12.h>
#include <apriltag/tagStandard41h12.h>
#include <apriltag/tagStandard52h13.h>

namespace apriltag3_ros
{

namespace
{

struct FamilyDef
{
  apriltag_family_t * (*create)();
  void (*destroy)(apriltag_family_t *);
};

const std::unordered_map<std::string, FamilyDef> & familyRegistry()
{
  static const std::unordered_map<std::string, FamilyDef> registry = {
    {"tag16h5",          {tag16h5_create,          tag16h5_destroy}},
    {"tag25h9",          {tag25h9_create,          tag25h9_destroy}},
    {"tag36h10",         {tag36h10_create,         tag36h10_destroy}},
    {"tag36h11",         {tag36h11_create,         tag36h11_destroy}},
    {"tagCircle21h7",    {tagCircle21h7_create,    tagCircle21h7_destroy}},
    {"tagCircle49h12",   {tagCircle49h12_create,   tagCircle49h12_destroy}},
    {"tagCustom48h12",   {tagCustom48h12_create,   tagCustom48h12_destroy}},
    {"tagStandard41h12", {tagStandard41h12_create, tagStandard41h12_destroy}},
    {"tagStandard52h13", {tagStandard52h13_create, tagStandard52h13_destroy}},
  };
  return registry;
}

std::array<double, 36> computeCovariance(
  const apriltag_detection_t & det, double tag_size,
  const std::array<double, 9> & R_arr,
  const std::array<double, 3> & t_arr,
  const cv::Mat & K, const cv::Mat & D)
{
  // Sub-pixel corner-localisation noise floor (sigma >= 0.5 px) — avoids
  // overconfident covariance when residuals happen to be near zero.
  constexpr double kSigma2Floor = 0.25;

  const cv::Mat R = (cv::Mat_<double>(3, 3) <<
      R_arr[0], R_arr[1], R_arr[2],
      R_arr[3], R_arr[4], R_arr[5],
      R_arr[6], R_arr[7], R_arr[8]);
  cv::Mat rvec;
  cv::Rodrigues(R, rvec);
  const cv::Mat tvec =
    (cv::Mat_<double>(3, 1) << t_arr[0], t_arr[1], t_arr[2]);

  const double h = tag_size / 2.0;
  // Marker frame: X right, Y up, Z out of plane. Order matches apriltag's
  // ideal-tag homography enumeration so det.p[0..3] pairs as:
  //   det.p[0] ↔ top-left, [1] top-right, [2] bottom-right, [3] bottom-left.
  const std::vector<cv::Point3d> object_pts = {
    {-h,  h, 0.0},
    { h,  h, 0.0},
    { h, -h, 0.0},
    {-h, -h, 0.0},
  };
  const std::vector<cv::Point2d> image_pts = {
    {det.p[0][0], det.p[0][1]},
    {det.p[1][0], det.p[1][1]},
    {det.p[2][0], det.p[2][1]},
    {det.p[3][0], det.p[3][1]},
  };

  std::vector<cv::Point2d> reproj;
  cv::Mat J_full;
  cv::projectPoints(object_pts, rvec, tvec, K, D, reproj, J_full);

  // First 6 columns of J_full: [d/drvec (3), d/dtvec (3)].
  const cv::Mat J = J_full(cv::Rect(0, 0, 6, J_full.rows)).clone();

  double sse = 0.0;
  for (size_t i = 0; i < image_pts.size(); ++i) {
    const double dx = reproj[i].x - image_pts[i].x;
    const double dy = reproj[i].y - image_pts[i].y;
    sse += dx * dx + dy * dy;
  }
  const int dof = std::max(1, static_cast<int>(2 * image_pts.size()) - 6);
  const double sigma2 = std::max(sse / dof, kSigma2Floor);

  cv::Mat Sigma;
  if (cv::invert(J.t() * J, Sigma, cv::DECOMP_SVD) == 0.0) {
    return {};
  }
  Sigma *= sigma2;

  // Reorder from solver layout [rx, ry, rz, x, y, z] to ROS
  // PoseWithCovariance layout [x, y, z, rx, ry, rz].
  static constexpr int kPerm[6] = {3, 4, 5, 0, 1, 2};
  std::array<double, 36> cov{};
  for (int r = 0; r < 6; ++r) {
    for (int c = 0; c < 6; ++c) {
      cov[r * 6 + c] = Sigma.at<double>(kPerm[r], kPerm[c]);
    }
  }
  return cov;
}

std::optional<Pose> solveOpenCv(
  const apriltag_detection_t & det, double tag_size, int flag,
  const cv::Mat & K, const cv::Mat & D)
{
  const double h = tag_size / 2.0;
  // Marker frame: X right, Y up, Z out of plane. Order required by
  // SOLVEPNP_IPPE_SQUARE and matches apriltag's H homography enumeration
  // so det.p[i] pairs with object_pts[i] for i in 0..3.
  const std::vector<cv::Point3d> object_pts = {
    {-h,  h, 0.0},
    { h,  h, 0.0},
    { h, -h, 0.0},
    {-h, -h, 0.0},
  };
  std::vector<cv::Point2d> image_pts = {
    {det.p[0][0], det.p[0][1]},
    {det.p[1][0], det.p[1][1]},
    {det.p[2][0], det.p[2][1]},
    {det.p[3][0], det.p[3][1]},
  };

  cv::Mat rvec, tvec;
  if (!cv::solvePnP(object_pts, image_pts, K, D, rvec, tvec, false, flag)) {
    return std::nullopt;
  }

  cv::Mat R;
  cv::Rodrigues(rvec, R);

  Pose p;
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      p.R[r * 3 + c] = R.at<double>(r, c);
    }
    p.t[r] = tvec.at<double>(r, 0);
  }

  std::vector<cv::Point2d> reproj;
  cv::projectPoints(object_pts, rvec, tvec, K, D, reproj);
  double sse = 0.0;
  for (size_t k = 0; k < object_pts.size(); ++k) {
    const double dx = reproj[k].x - image_pts[k].x;
    const double dy = reproj[k].y - image_pts[k].y;
    sse += dx * dx + dy * dy;
  }
  p.err = std::sqrt(sse / static_cast<double>(object_pts.size()));
  p.covariance = computeCovariance(det, tag_size, p.R, p.t, K, D);
  return p;
}

std::optional<Pose> solveApriltag(
  apriltag_detection_t & det, double tag_size, const std::string & method,
  double fx, double fy, double cx, double cy)
{
  apriltag_detection_info_t info{&det, tag_size, fx, fy, cx, cy};
  apriltag_pose_t pose{};
  double err = 0.0;

  if (method == "homography") {
    estimate_pose_for_tag_homography(&info, &pose);
  } else {
    err = estimate_tag_pose(&info, &pose);
  }

  if (!pose.R || !pose.t) {
    if (pose.R) {matd_destroy(pose.R);}
    if (pose.t) {matd_destroy(pose.t);}
    return std::nullopt;
  }

  Pose p;
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      p.R[r * 3 + c] = matd_get(pose.R, r, c);
    }
    p.t[r] = matd_get(pose.t, r, 0);
  }
  p.err = err;
  matd_destroy(pose.R);
  matd_destroy(pose.t);

  const cv::Mat K = (cv::Mat_<double>(3, 3) <<
      fx, 0.0, cx,
      0.0, fy, cy,
      0.0, 0.0, 1.0);
  const cv::Mat D = cv::Mat::zeros(5, 1, CV_64F);
  p.covariance = computeCovariance(det, tag_size, p.R, p.t, K, D);
  return p;
}

std::optional<Pose> solvePose(
  apriltag_detection_t & det, double tag_size, const std::string & method,
  const cv::Mat & K, const cv::Mat & D,
  double fx, double fy, double cx, double cy)
{
  if (method == "ippe_square") {
    return solveOpenCv(det, tag_size, cv::SOLVEPNP_IPPE_SQUARE, K, D);
  }
  if (method == "iterative") {
    return solveOpenCv(det, tag_size, cv::SOLVEPNP_ITERATIVE, K, D);
  }
  return solveApriltag(det, tag_size, method, fx, fy, cx, cy);
}

}  // namespace

Detector::Detector(const apriltag3_ros::Params & params)
: td_(apriltag_detector_create()),
  pose_method_(params.pose_method),
  decision_margin_min_(params.decision_margin_min)
{
  td_->nthreads = static_cast<int>(params.nthreads);
  td_->quad_decimate = static_cast<float>(params.quad_decimate);
  td_->quad_sigma = static_cast<float>(params.quad_sigma);
  td_->refine_edges = params.refine_edges;
  td_->decode_sharpening = params.decode_sharpening;

  td_->qtp.min_cluster_pixels = static_cast<int>(params.qtp.min_cluster_pixels);
  td_->qtp.max_nmaxima = static_cast<int>(params.qtp.max_nmaxima);
  td_->qtp.critical_rad = static_cast<float>(params.qtp.critical_rad);
  td_->qtp.cos_critical_rad = std::cos(static_cast<float>(params.qtp.critical_rad));
  td_->qtp.max_line_fit_mse = static_cast<float>(params.qtp.max_line_fit_mse);
  td_->qtp.min_white_black_diff = static_cast<int>(params.qtp.min_white_black_diff);
  td_->qtp.deglitch = params.qtp.deglitch ? 1 : 0;

  const int bits_corrected = static_cast<int>(params.max_hamming);

  std::unordered_map<std::string, apriltag_family_t *> created;
  groups_.reserve(params.tag_groups.size());

  for (const std::string & name : params.tag_groups) {
    auto it = params.tag_groups_map.find(name);
    if (it == params.tag_groups_map.end()) {
      continue;
    }
    const auto & g = it->second;

    if (g.id_end < g.id_begin) {
      throw std::invalid_argument(
              "Tag group '" + name + "': id_end (" + std::to_string(g.id_end) +
              ") < id_begin (" + std::to_string(g.id_begin) + ")");
    }

    // Per-group prefix overrides the global default when non-empty so
    // that groups can publish TFs under distinct namespaces.
    const std::string prefix =
      g.tag_frame_prefix.empty() ? params.tag_frame_prefix : g.tag_frame_prefix;

    groups_.push_back(Group{
          name, g.family,
          static_cast<int>(g.id_begin),
          static_cast<int>(g.id_end),
          g.size,
          prefix});

    if (created.find(g.family) == created.end()) {
      auto reg_it = familyRegistry().find(g.family);
      if (reg_it == familyRegistry().end()) {
        throw std::invalid_argument("Unknown tag family: " + g.family);
      }
      apriltag_family_t * fam = reg_it->second.create();
      apriltag_detector_add_family_bits(td_, fam, bits_corrected);
      created.emplace(g.family, fam);
      families_.emplace_back(g.family, fam);
    }
  }
}

Detector::~Detector()
{
  if (td_) {
    apriltag_detector_clear_families(td_);
    apriltag_detector_destroy(td_);
    td_ = nullptr;
  }
  for (auto & [name, fam] : families_) {
    auto reg_it = familyRegistry().find(name);
    if (reg_it != familyRegistry().end() && fam) {
      reg_it->second.destroy(fam);
    }
  }
  families_.clear();
}

const Detector::Group * Detector::findGroup(
  const std::string & family, int id) const
{
  for (const auto & g : groups_) {
    if (g.family == family && id >= g.id_begin && id <= g.id_end) {
      return &g;
    }
  }
  return nullptr;
}

std::vector<Detection> Detector::detect(
  const sensor_msgs::msg::Image::ConstSharedPtr & image,
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr & camera_info)
{
  if (!image) {
    return {};
  }

  cv_bridge::CvImageConstPtr cv_image;
  try {
    cv_image = cv_bridge::toCvShare(image, "mono8");
  } catch (const cv_bridge::Exception &) {
    return {};
  }
  if (!cv_image || cv_image->image.empty()) {
    return {};
  }

  image_u8_t im{
    static_cast<int32_t>(cv_image->image.cols),
    static_cast<int32_t>(cv_image->image.rows),
    static_cast<int32_t>(cv_image->image.step[0]),
    cv_image->image.data,
  };

  zarray_t * raw = apriltag_detector_detect(td_, &im);
  if (!raw) {
    return {};
  }

  // Input is assumed to be a rectified image (e.g. image_proc/rectify_node
  // output). Use the rectified projection matrix P from CameraInfo and treat
  // distortion as zero; raw K/D would be inconsistent with the pixel grid.
  image_geometry::PinholeCameraModel model;
  model.fromCameraInfo(*camera_info);
  const cv::Matx34d & P = model.projectionMatrix();
  const double fx = P(0, 0);
  const double fy = P(1, 1);
  const double cx = P(0, 2);
  const double cy = P(1, 2);
  const cv::Mat K = (cv::Mat_<double>(3, 3) <<
      fx, 0.0, cx,
      0.0, fy, cy,
      0.0, 0.0, 1.0);
  const cv::Mat D = cv::Mat::zeros(5, 1, CV_64F);

  std::vector<Detection> out;
  out.reserve(zarray_size(raw));

  for (int i = 0; i < zarray_size(raw); ++i) {
    apriltag_detection_t * det = nullptr;
    zarray_get(raw, i, &det);
    if (!det) {
      continue;
    }
    if (det->decision_margin < decision_margin_min_) {
      continue;
    }
    const Group * g = findGroup(det->family->name, det->id);
    if (!g) {
      continue;
    }

    Detection d{};
    d.group = g->name;
    d.family = det->family->name;
    d.id = det->id;
    d.hamming = det->hamming;
    d.decision_margin = det->decision_margin;
    d.center = {det->c[0], det->c[1]};
    for (int k = 0; k < 4; ++k) {
      d.corners[k] = {det->p[k][0], det->p[k][1]};
    }
    d.size = g->size;
    d.tag_frame_prefix = g->tag_frame_prefix;
    d.pose = solvePose(*det, g->size, pose_method_, K, D, fx, fy, cx, cy);

    out.push_back(std::move(d));
  }

  apriltag_detections_destroy(raw);
  return out;
}

}  // namespace apriltag3_ros
