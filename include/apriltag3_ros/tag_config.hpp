#pragma once

#include <algorithm>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

namespace apriltag3_ros
{

// Per-ID configuration table built from positional parameter arrays.
// id_whitelist empty => allow every detected ID.
struct TagConfig
{
  double default_size{0.0};
  std::vector<int64_t> id_whitelist;
  std::unordered_map<int64_t, double> size_overrides;
  std::unordered_map<int64_t, std::string> frame_overrides;
  std::string frame_prefix{"tag_"};

  bool is_allowed(int64_t id) const
  {
    if (id_whitelist.empty()) {
      return true;
    }
    return std::find(id_whitelist.begin(), id_whitelist.end(), id) != id_whitelist.end();
  }

  double size_for(int64_t id) const
  {
    auto it = size_overrides.find(id);
    return it != size_overrides.end() ? it->second : default_size;
  }

  std::string frame_for(int64_t id) const
  {
    auto it = frame_overrides.find(id);
    if (it != frame_overrides.end()) {
      return it->second;
    }
    return frame_prefix + std::to_string(id);
  }
};

// Build TagConfig from raw parameter arrays. Throws std::invalid_argument
// on length mismatch between paired *_ids and *_values arrays.
inline TagConfig make_tag_config(
  double default_size,
  const std::vector<int64_t> & id_whitelist,
  const std::vector<int64_t> & sizes_ids,
  const std::vector<double> & sizes_values,
  const std::vector<int64_t> & frames_ids,
  const std::vector<std::string> & frames_values,
  const std::string & frame_prefix)
{
  if (sizes_ids.size() != sizes_values.size()) {
    throw std::invalid_argument(
            "tag.sizes_ids and tag.sizes_values must have the same length");
  }
  if (frames_ids.size() != frames_values.size()) {
    throw std::invalid_argument(
            "tag.frames_ids and tag.frames_values must have the same length");
  }

  TagConfig cfg;
  cfg.default_size = default_size;
  cfg.id_whitelist = id_whitelist;
  cfg.frame_prefix = frame_prefix;

  for (size_t i = 0; i < sizes_ids.size(); ++i) {
    cfg.size_overrides[sizes_ids[i]] = sizes_values[i];
  }
  for (size_t i = 0; i < frames_ids.size(); ++i) {
    cfg.frame_overrides[frames_ids[i]] = frames_values[i];
  }
  return cfg;
}

}  // namespace apriltag3_ros
