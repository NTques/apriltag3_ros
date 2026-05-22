# apriltag3_ros

> 한국어 문서: [README.ko.md](README.ko.md).

ROS 2 wrapper around the [AprilTag 3](https://github.com/AprilRobotics/apriltag) library. Detects tags in a rectified image stream, estimates 6-DoF pose with covariance, optionally broadcasts a TF per detection — and, **most importantly, treats tag groups as first-class citizens** so a single detector can drive multiple semantic roles at once.

Ships as both a composable node (`apriltag3_ros::AprilTagDetector`) and a standalone executable (`apriltag_detector`).

## Why tag groups

Most AprilTag wrappers force one family and one tag size per detector instance. Real robots rarely fit that mold: docking markers, navigation landmarks, and object IDs typically use different families, different physical sizes, and need to be consumed by completely different downstream nodes. This package solves that with **tag groups**: named buckets that each declare their own `family`, ID range, physical `size`, and (now) TF child-frame prefix.

A single `AprilTagDetector` instance can:

- **Mix tag families.** Run `tag36h11` for landmarks and `tag25h9` for docking from the same camera — the detector loads each declared family exactly once and routes every decoded tag back to the group it belongs to.
- **Mix physical sizes.** Pose estimation needs the true tag edge length, which can differ wildly between roles (e.g. 10 cm wall markers vs. 5 cm dock fiducials). Each group owns its own `size`, so poses come out correct without per-ID exception lists.
- **Namespace TF frames per group.** Each group can set its own `tag_frame_prefix`. Landmarks publish as `tag_0, tag_1, ...` while dock markers publish as `dock_100, dock_101, ...`. Empty prefix falls back to the global default, so existing configs keep working.
- **Route detections by semantic role.** Every published `AprilTagDetection` carries the matched `group` name and the resolved `tf_frame_id`. Downstream nodes can filter by group ("only docking markers, please") without hard-coding family/ID ranges of their own.
- **Drop everything else.** A tag whose family/ID doesn't fall inside any declared group is silently ignored. Use groups as a positive allowlist; stray fiducials in the environment won't pollute your topic.

See [Tag groups](#tag-groups) for the schema and a worked example.

## Topics

| Direction | Topic         | Type                                          | Notes |
|-----------|---------------|-----------------------------------------------|-------|
| sub       | `image_rect`  | `sensor_msgs/Image`                           | Rectified input. Topic name comes from the `image_topic` parameter; remap via launch arg. |
| sub       | `camera_info` | `sensor_msgs/CameraInfo`                      | Auto-resolved as the `image_rect` sibling by `image_transport`. |
| pub       | `detections`  | `apriltag3_msgs/AprilTagDetectionArray`       | One message per input frame. |
| pub       | `/tf`         | `tf2_msgs/TFMessage`                          | One TF per detection with a valid pose, when `publish_tf: true`. |

Pose frame: detections are in the image header's `frame_id` (the camera optical frame). TF child frame is `<tag_frame_prefix><id>`, resolved per group (see below).

## Quick start

```bash
ros2 launch apriltag3_ros apriltag_detector.launch.py \
    image_topic:=/camera/image_rect \
    params_file:=/abs/path/to/your.yaml
```

Launch arguments:

- `params_file` — absolute path to a parameters YAML. Defaults to the bundled `config/apriltag_detector.yaml`.
- `image_topic` — input image topic to remap onto `image_rect`. The matching `camera_info` is the topic's sibling.
- `container_name` — composable node container name (default `apriltag_container`).
- `use_intra_process` — toggle intra-process comms inside the container (default `true`).

Input must be rectified: the node uses `CameraInfo.P` (fx, fy, cx, cy) and treats distortion as zero. Feeding `image_raw` produces a warning and sub-pixel corner errors. Pair with `image_proc/rectify_node` upstream if your driver does not already publish `image_rect`.

## Parameters

All parameters are declared via `generate_parameter_library` from `src/apriltag_detector_parameters.yaml`. See that file for full validation rules and defaults.

### I/O

| Name             | Type   | Default       | Notes |
|------------------|--------|---------------|-------|
| `image_topic`    | string | `image_rect`  | Read-only. Base topic for `image_transport`. |
| `image_transport`| string | `raw`         | Read-only. Transport plugin (`raw`, `compressed`, ...). |

### Detector

| Name                          | Type   | Default     | Notes |
|-------------------------------|--------|-------------|-------|
| `nthreads`                    | int    | `1`         | Worker threads. |
| `quad_decimate`               | double | `2.0`       | Downscale factor for quad detection; decoding stays at full res. |
| `quad_sigma`                  | double | `0.0`       | Pre-blur sigma (px). `~0.8` helps on noisy inputs. |
| `refine_edges`                | bool   | `true`      | Ignored when `quad_decimate == 1`. |
| `decode_sharpening`           | double | `0.25`      | |
| `qtp.min_cluster_pixels`      | int    | `5`         | |
| `qtp.max_nmaxima`             | int    | `10`        | |
| `qtp.critical_rad`            | double | `0.17453293`| ~10°. |
| `qtp.max_line_fit_mse`        | double | `10.0`      | |
| `qtp.min_white_black_diff`    | int    | `5`         | 0..255. |
| `qtp.deglitch`                | bool   | `false`     | |
| `max_hamming`                 | int    | `0`         | 0..2; library hard cap. |
| `decision_margin_min`         | double | `0.0`       | Drops low-confidence decodes. `0` disables. |

### Pose estimation

| Name          | Type   | Default                  | Notes |
|---------------|--------|--------------------------|-------|
| `pose_method` | string | `orthogonal_iteration`   | One of `orthogonal_iteration`, `homography`, `ippe_square`, `iterative`. |

Pose error semantics depend on the solver:

- `ippe_square` / `iterative` — RMS reprojection error (pixels).
- `orthogonal_iteration` — object-space error (apriltag units).
- `homography` — `0` (estimator returns no error).

Covariance is always populated, linearized from the corner-reprojection Jacobian `(JᵀJ)⁻¹` scaled by residual sigma² with a `σ² ≥ 0.25 px²` floor to avoid overconfidence.

### TF broadcasting

| Name               | Type   | Default | Notes |
|--------------------|--------|---------|-------|
| `publish_tf`       | bool   | `true`  | Broadcast a TF per detection with a valid pose. |
| `tag_frame_prefix` | string | `tag_`  | Global default; used when a group has no override. |

### Tag groups

Tag groups are the core organizing unit of this package (see [Why tag groups](#why-tag-groups) for the motivation). Each name in `tag_groups` (a `string_array`, read-only) must have a matching nested block declaring how to interpret tags in that bucket. Tags whose family/ID does not fall inside any declared group are dropped.

Per-group fields:

| Field              | Type   | Default     | Notes |
|--------------------|--------|-------------|-------|
| `family`           | string | `tag36h11`  | One of `tag16h5`, `tag25h9`, `tag36h10`, `tag36h11`, `tagCircle21h7`, `tagCircle49h12`, `tagCustom48h12`, `tagStandard41h12`, `tagStandard52h13`. Multiple groups may share a family. |
| `id_begin`         | int    | `0`         | First ID in the group (inclusive). |
| `id_end`           | int    | `0`         | Last ID in the group (inclusive). Must be `>= id_begin`. |
| `size`             | double | `0.05`      | Physical edge length (m). Pose accuracy depends on this matching reality. |
| `tag_frame_prefix` | string | `""`        | Per-group TF child-frame prefix. Empty → fall back to the global `tag_frame_prefix`. |

Two groups may share a `family` as long as their ID ranges don't overlap — the detector adds each family to the underlying `apriltag_detector_t` exactly once and dispatches by (family, ID).

#### Worked example: landmarks + dock markers

A robot uses 10 cm `tag36h11` markers scattered across a warehouse as navigation landmarks, and 5 cm `tag25h9` markers on its docking station. Both are seen by the same camera. Without running two detectors, the two are cleanly separated in TF, in the detection topic, and to downstream consumers.

```yaml
apriltag_detector:
  ros__parameters:
    publish_tf: true
    tag_frame_prefix: "tag_"           # global default (used by 'landmarks')

    tag_groups: ["landmarks", "dock"]

    landmarks:
      family: tag36h11
      id_begin: 0
      id_end: 99
      size: 0.10                       # 10 cm wall markers
      # tag_frame_prefix omitted → uses the global "tag_".
      # TF frames:  tag_0, tag_1, ..., tag_99
      # Detections: group="landmarks", tf_frame_id="tag_<id>"

    dock:
      family: tag25h9
      id_begin: 100
      id_end: 149
      size: 0.05                       # 5 cm dock fiducials
      tag_frame_prefix: "dock_"
      # TF frames:  dock_100, ..., dock_149
      # Detections: group="dock", tf_frame_id="dock_<id>"
```

With this config:

- The navigation stack consumes the TF tree and only listens for `tag_*` frames — it never sees dock markers.
- The docking controller subscribes to `detections`, filters `group == "dock"`, and looks up `tf_frame_id` directly in its TF lookups. No hard-coded ID ranges in client code.
- A stray `tag36h11` ID outside `0..99` — e.g. an AprilTag put up by another team or vendor in the same space — falls outside every declared group and is silently dropped.

## Messages

The `apriltag3_msgs` package defines:

- `AprilTagDetectionArray` — `header` (matches the input image) plus `AprilTagDetection[] detections`.
- `AprilTagDetection` — `group`, `family`, `id`, `tf_frame_id`, `hamming`, `decision_margin`, `size`, `center`, 4× `corners`, `pose` (PoseWithCovariance), `pose_error`.

  - `group` — name of the configured tag group the detection matched (e.g. `landmarks`, `dock`); lets downstream code route by semantic role.
  - `tf_frame_id` — resolved TF child frame id (`<prefix><id>`) using the group's prefix override or the global default. Populated even when `publish_tf` is disabled, so consumers can still address the tag.

Detections whose pose solver failed are still published in the array; only their TF is skipped. Use `pose_error` and the covariance to gate downstream consumers.

## Install & build

Tested on ROS 2 Jazzy. The package depends on a companion message package, [`apriltag3_msgs`](https://github.com/NTques/apriltag3_msgs), so clone both into the same workspace.

```bash
# 1. Clone into your workspace `src/`.
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone https://github.com/NTques/apriltag3_msgs.git
git clone https://github.com/NTques/apriltag3_ros.git

# 2. Install system / ROS dependencies (apriltag C library, image_transport, ...).
cd ~/ros2_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# 3. Build.
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to apriltag3_ros
source install/setup.bash
```

After this, `ros2 launch apriltag3_ros apriltag_detector.launch.py` should resolve. See [Quick start](#quick-start) for parameters and remaps.

Direct dependencies (handled by `rosdep` above): `rclcpp`, `rclcpp_components`, `sensor_msgs`, `geometry_msgs`, `image_transport`, `image_geometry`, `cv_bridge`, `OpenCV`, `apriltag`, `apriltag3_msgs`, `tf2`, `tf2_ros`, `generate_parameter_library`.

## License

BSD-2-Clause. See [`LICENSE`](LICENSE).
