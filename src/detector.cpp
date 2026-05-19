#include "apriltag3_ros/detector.hpp"

#include <stdexcept>
#include <unordered_map>
#include <utility>

extern "C" {
#include <tag16h5.h>
#include <tag25h9.h>
#include <tag36h10.h>
#include <tag36h11.h>
#include <tagCircle21h7.h>
#include <tagCircle49h12.h>
#include <tagCustom48h12.h>
#include <tagStandard41h12.h>
#include <tagStandard52h13.h>
}

namespace apriltag3_ros
{

namespace
{

struct FamilyFactory
{
  apriltag_family_t * (* create)();
  void (* destroy)(apriltag_family_t *);
};

const std::unordered_map<std::string, FamilyFactory> & family_table()
{
  static const std::unordered_map<std::string, FamilyFactory> table = {
    {"tag36h11", {tag36h11_create, tag36h11_destroy}},
    {"tag36h10", {tag36h10_create, tag36h10_destroy}},
    {"tag25h9", {tag25h9_create, tag25h9_destroy}},
    {"tag16h5", {tag16h5_create, tag16h5_destroy}},
    {"tagStandard41h12", {tagStandard41h12_create, tagStandard41h12_destroy}},
    {"tagStandard52h13", {tagStandard52h13_create, tagStandard52h13_destroy}},
    {"tagCircle21h7", {tagCircle21h7_create, tagCircle21h7_destroy}},
    {"tagCircle49h12", {tagCircle49h12_create, tagCircle49h12_destroy}},
    {"tagCustom48h12", {tagCustom48h12_create, tagCustom48h12_destroy}},
  };
  return table;
}

}  // namespace

std::vector<std::string> Detector::supported_families()
{
  std::vector<std::string> out;
  out.reserve(family_table().size());
  for (const auto & kv : family_table()) {
    out.push_back(kv.first);
  }
  return out;
}

Detector::Detector(
  const std::vector<std::string> & families,
  const Params & params)
{
  td_ = apriltag_detector_create();
  if (!td_) {
    throw std::runtime_error("apriltag_detector_create() returned null");
  }

  td_->nthreads = params.threads;
  apply_dynamic_params(params);

  const auto & table = family_table();
  for (const auto & name : families) {
    auto it = table.find(name);
    if (it == table.end()) {
      apriltag_detector_destroy(td_);
      td_ = nullptr;
      throw std::invalid_argument("Unsupported tag family: " + name);
    }
    apriltag_family_t * fam = it->second.create();
    if (!fam) {
      apriltag_detector_destroy(td_);
      td_ = nullptr;
      throw std::runtime_error("Failed to create tag family: " + name);
    }
    apriltag_detector_add_family_bits(td_, fam, params.hamming);
    families_.push_back({name, fam, it->second.destroy});
  }
}

Detector::~Detector()
{
  if (td_) {
    apriltag_detector_destroy(td_);
    td_ = nullptr;
  }
  for (auto & entry : families_) {
    if (entry.family) {
      entry.destroy(entry.family);
    }
  }
  families_.clear();
}

Detector::Detections::Detections(zarray_t * z) noexcept
: z_(z) {}

Detector::Detections::~Detections()
{
  if (z_) {
    apriltag_detections_destroy(z_);
    z_ = nullptr;
  }
}

Detector::Detections::Detections(Detections && other) noexcept
: z_(other.z_)
{
  other.z_ = nullptr;
}

Detector::Detections & Detector::Detections::operator=(Detections && other) noexcept
{
  if (this != &other) {
    if (z_) {
      apriltag_detections_destroy(z_);
    }
    z_ = other.z_;
    other.z_ = nullptr;
  }
  return *this;
}

size_t Detector::Detections::size() const noexcept
{
  return z_ ? static_cast<size_t>(zarray_size(z_)) : 0;
}

const apriltag_detection_t * Detector::Detections::operator[](size_t i) const
{
  apriltag_detection_t * det = nullptr;
  zarray_get(z_, static_cast<int>(i), &det);
  return det;
}

void Detector::apply_dynamic_params(const Params & params)
{
  if (!td_) {return;}
  td_->quad_decimate = static_cast<float>(params.decimate);
  td_->quad_sigma = static_cast<float>(params.blur);
  td_->refine_edges = params.refine_edges;
  td_->decode_sharpening = params.sharpening;
  td_->debug = params.debug;

  td_->qtp.min_cluster_pixels = params.qtp_min_cluster_pixels;
  td_->qtp.max_nmaxima = params.qtp_max_nmaxima;
  td_->qtp.critical_rad = static_cast<float>(params.qtp_critical_rad);
  td_->qtp.cos_critical_rad =
    static_cast<float>(std::cos(params.qtp_critical_rad));
  td_->qtp.max_line_fit_mse = static_cast<float>(params.qtp_max_line_fit_mse);
  td_->qtp.min_white_black_diff = params.qtp_min_white_black_diff;
  td_->qtp.deglitch = params.qtp_deglitch;
}

Detector::Detections Detector::detect(const cv::Mat & gray)
{
  if (gray.type() != CV_8UC1) {
    throw std::invalid_argument("Detector::detect expects an 8-bit single-channel image");
  }
  image_u8_t im{
    gray.cols,
    gray.rows,
    static_cast<int32_t>(gray.step),
    gray.data,
  };
  zarray_t * z = apriltag_detector_detect(td_, &im);
  return Detections(z);
}

}  // namespace apriltag3_ros
