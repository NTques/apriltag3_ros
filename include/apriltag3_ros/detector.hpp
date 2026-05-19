#pragma once

#include <memory>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

extern "C" {
#include <apriltag.h>
}

namespace apriltag3_ros
{

class Detector
{
public:
  struct Params
  {
    int threads{1};
    double decimate{2.0};
    double blur{0.0};
    bool refine_edges{true};
    double sharpening{0.25};
    bool debug{false};
    int hamming{2};

    int qtp_min_cluster_pixels{5};
    int qtp_max_nmaxima{10};
    double qtp_critical_rad{0.1745329};
    double qtp_max_line_fit_mse{10.0};
    int qtp_min_white_black_diff{5};
    bool qtp_deglitch{false};
  };

  // RAII wrapper around the zarray<apriltag_detection_t*> returned by detect.
  class Detections
  {
public:
    explicit Detections(zarray_t * z) noexcept;
    ~Detections();
    Detections(const Detections &) = delete;
    Detections & operator=(const Detections &) = delete;
    Detections(Detections && other) noexcept;
    Detections & operator=(Detections && other) noexcept;

    size_t size() const noexcept;
    const apriltag_detection_t * operator[](size_t i) const;

private:
    zarray_t * z_;
  };

  Detector(const std::vector<std::string> & families, const Params & params);
  ~Detector();

  Detector(const Detector &) = delete;
  Detector & operator=(const Detector &) = delete;

  // Returns names of family factories that are supported by this build.
  static std::vector<std::string> supported_families();

  // Run detection on an 8-bit single-channel image. The cv::Mat must outlive
  // the returned Detections object (no data is copied).
  Detections detect(const cv::Mat & gray);

  // Apply dynamic tuning fields (everything except threads/hamming/families
  // which require detector reconstruction). Safe to call between frames.
  void apply_dynamic_params(const Params & params);

private:
  apriltag_detector_t * td_{nullptr};

  struct FamilyEntry
  {
    std::string name;
    apriltag_family_t * family;
    void (* destroy)(apriltag_family_t *);
  };
  std::vector<FamilyEntry> families_;
};

}  // namespace apriltag3_ros
