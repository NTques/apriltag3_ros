# apriltag3_ros

> 한국어 문서입니다. English: [README.md](README.md).

[AprilTag 3](https://github.com/AprilRobotics/apriltag) 라이브러리의 ROS 2 래퍼 패키지입니다. 렉티파이된(rectified) 이미지 스트림에서 태그를 검출하고, 공분산 포함 6-DoF 자세(pose)를 추정하며, 옵션에 따라 검출된 태그마다 TF를 브로드캐스트합니다. 그리고 **이 패키지의 핵심은 "태그 그룹(tag groups)"을 일급 객체로 다룬다는 점입니다.** 하나의 디텍터 인스턴스로 여러 의미적 역할을 동시에 다룰 수 있습니다.

컴포저블 노드(`apriltag3_ros::AprilTagDetector`)와 독립 실행파일(`apriltag_detector`) 두 가지 형태로 제공됩니다.

## 왜 태그 그룹인가

대부분의 AprilTag 래퍼는 디텍터 하나당 패밀리 하나, 태그 크기 하나만 가정합니다. 그런데 실제 로봇에서는 도킹 마커, 내비게이션 랜드마크, 객체 식별용 마커가 보통 서로 다른 패밀리와 서로 다른 실제 크기를 사용하고, 완전히 다른 하위 노드가 소비합니다. 이 패키지는 그 문제를 **태그 그룹**으로 해결합니다. 그룹은 자기 자신의 `family`, ID 범위, 실제 `size`, 그리고 (이제) TF child 프레임 접두어까지 가지는 이름붙은 묶음입니다.

`AprilTagDetector` 하나로 다음이 가능합니다.

- **여러 패밀리 혼용.** 같은 카메라에서 랜드마크용 `tag36h11`과 도킹용 `tag25h9`를 동시에 처리합니다. 디텍터는 선언된 패밀리를 정확히 한 번씩만 로드하고, 디코딩된 모든 태그를 자기가 속한 그룹으로 라우팅합니다.
- **그룹별 실제 크기.** 자세 추정 정확도는 태그 한 변의 실제 길이에 민감합니다. 역할마다 크기가 크게 다르면(예: 10 cm 벽 마커 vs 5 cm 도킹 마커) ID별 예외 처리 없이 그룹의 `size`만 잘 정해 두면 됩니다.
- **그룹별 TF 네임스페이스.** 그룹마다 `tag_frame_prefix`를 따로 줄 수 있습니다. 랜드마크는 `tag_0, tag_1, ...`로, 도킹 마커는 `dock_100, dock_101, ...`로 발행됩니다. 빈 접두어이면 글로벌 기본값으로 폴백하므로 기존 설정 파일은 그대로 동작합니다.
- **의미 역할 기반 라우팅.** 발행되는 모든 `AprilTagDetection`에는 매칭된 `group` 이름과 해석된 `tf_frame_id`가 들어갑니다. 하위 노드는 자기쪽에 family/ID 범위를 하드코딩하지 않고 그룹 이름으로 필터링하면 됩니다. ("도킹 마커만 줘.")
- **나머지는 자동 폐기.** 어떤 그룹의 family/ID 범위에도 속하지 않는 태그는 조용히 무시됩니다. 그룹을 일종의 허용 목록(allowlist)으로 쓰는 셈이라, 환경에 굴러다니는 잡다한 fiducial이 토픽을 오염시키지 않습니다.

스키마와 예시는 [태그 그룹](#태그-그룹) 섹션을 참조하세요.

## 토픽

| 방향 | 토픽           | 타입                                          | 비고 |
|------|----------------|-----------------------------------------------|------|
| sub  | `image_rect`   | `sensor_msgs/Image`                           | 렉티파이된 입력. 토픽 이름은 `image_topic` 파라미터로 지정되며, 런치 인자로 리맵 가능. |
| sub  | `camera_info`  | `sensor_msgs/CameraInfo`                      | `image_rect`의 형제 토픽으로 `image_transport`가 자동 결정. |
| pub  | `detections`   | `apriltag3_msgs/AprilTagDetectionArray`       | 입력 프레임마다 1개 메시지. |
| pub  | `/tf`          | `tf2_msgs/TFMessage`                          | `publish_tf: true`일 때, 자세 추정에 성공한 검출마다 TF 1개씩. |

자세의 기준 좌표계는 입력 이미지 헤더의 `frame_id`(카메라 광학 좌표계)입니다. TF child 프레임 이름은 `<tag_frame_prefix><id>` 이며, 그룹별로 다르게 지정할 수 있습니다(아래 참조).

## 빠른 시작

```bash
ros2 launch apriltag3_ros apriltag_detector.launch.py \
    image_topic:=/camera/image_rect \
    params_file:=/abs/path/to/your.yaml
```

런치 인자:

- `params_file` — 파라미터 YAML 절대 경로. 기본값은 패키지의 `config/apriltag_detector.yaml`.
- `image_topic` — `image_rect`에 리맵할 입력 이미지 토픽. 매칭되는 `camera_info`는 이 토픽의 형제로 자동 결정됨.
- `container_name` — 컴포저블 노드 컨테이너 이름 (기본 `apriltag_container`).
- `use_intra_process` — 컨테이너 내부 intra-process 통신 사용 여부 (기본 `true`).

입력은 반드시 **렉티파이된** 이미지여야 합니다. 노드는 `CameraInfo.P`의 `fx, fy, cx, cy`를 사용하고 왜곡 계수는 0으로 간주합니다. `image_raw`를 그대로 넣으면 경고가 뜨고 서브픽셀 단위 코너 오차가 발생합니다. 드라이버가 `image_rect`를 직접 발행하지 않는다면 앞단에 `image_proc/rectify_node`를 연결하세요.

## 파라미터

모든 파라미터는 `src/apriltag_detector_parameters.yaml`로부터 `generate_parameter_library`가 선언합니다. 유효성 검증 규칙과 전체 기본값은 해당 파일을 참조하세요.

> **동적 vs. read-only.** 런타임에 `ros2 param set`으로 바꿀 수 있는 것은 `detection_rate`, `pose_method`, `decision_margin_min` 셋뿐입니다. on-set 파라미터 콜백으로 즉시 반영됩니다(이미지 수신과 무관). 나머지는 전부 **read-only**로, 생성자에서 한 번만 적용되고 `ros2 param set`은 거부됩니다. 이들은 변경하려면 디텍터/태그 패밀리, 구독, 또는 TF 브로드캐스터를 다시 만들어야 하기 때문입니다. 해당 값들은 params YAML이나 런치 인자로 지정하세요.

### I/O

| 이름             | 타입   | 기본값        | 비고 |
|------------------|--------|---------------|------|
| `image_topic`    | string | `image_rect`  | read-only. `image_transport` 베이스 토픽. |
| `image_transport`| string | `raw`         | read-only. 전송 플러그인 (`raw`, `compressed`, ...). |
| `detection_rate` | double | `0.0`         | **동적.** 검출 주기 상한(Hz). 가장 최근 프레임만 처리하도록 스로틀링하며, 실제 처리율은 `min(카메라 FPS, detection_rate)`. `0`이면 매 프레임 처리. 노드 클럭 사용(`use_sim_time` 반영). |

### 디텍터

| 이름                          | 타입   | 기본값       | 비고 |
|-------------------------------|--------|--------------|------|
| `nthreads`                    | int    | `1`          | 워커 스레드 수. |
| `quad_decimate`               | double | `2.0`        | 쿼드 검출용 다운스케일 배율. 디코딩은 원본 해상도 그대로. |
| `quad_sigma`                  | double | `0.0`        | 입력 블러 시그마(px). 노이즈가 심하면 `~0.8` 권장. |
| `refine_edges`                | bool   | `true`       | `quad_decimate == 1`일 때 무시됨. |
| `decode_sharpening`           | double | `0.25`       | |
| `qtp.min_cluster_pixels`      | int    | `5`          | |
| `qtp.max_nmaxima`             | int    | `10`         | |
| `qtp.critical_rad`            | double | `0.17453293` | 약 10°. |
| `qtp.max_line_fit_mse`        | double | `10.0`       | |
| `qtp.min_white_black_diff`    | int    | `5`          | 0..255. |
| `qtp.deglitch`                | bool   | `false`      | |
| `max_hamming`                 | int    | `0`          | 0..2 (라이브러리 한도). |
| `decision_margin_min`         | double | `0.0`        | **동적.** 낮은 신뢰도 디코드 제거. `0`이면 비활성화. |

### 자세 추정

| 이름          | 타입   | 기본값                  | 비고 |
|---------------|--------|-------------------------|------|
| `pose_method` | string | `orthogonal_iteration`  | **동적.** `orthogonal_iteration`, `homography`, `ippe_square`, `iterative` 중 하나. |

`pose_error`의 의미는 솔버에 따라 다릅니다.

- `ippe_square` / `iterative` — 재투영 RMS 오차 (픽셀).
- `orthogonal_iteration` — object-space 오차 (apriltag 단위).
- `homography` — `0` (솔버가 오차를 반환하지 않음).

공분산은 항상 채워집니다. 코너 재투영 자코비안 기반 선형화 `(JᵀJ)⁻¹`에 잔차 σ²를 곱해 계산하며, 과신을 막기 위해 `σ² ≥ 0.25 px²`의 바닥값을 적용합니다.

### TF 브로드캐스트

| 이름               | 타입   | 기본값  | 비고 |
|--------------------|--------|---------|------|
| `publish_tf`       | bool   | `true`  | 자세 추정에 성공한 검출마다 TF 발행. |
| `tag_frame_prefix` | string | `tag_`  | 글로벌 기본값. 그룹별 override가 없을 때 사용. |

### 태그 그룹

태그 그룹은 이 패키지의 핵심 구성 단위입니다 (동기는 [왜 태그 그룹인가](#왜-태그-그룹인가) 참고). `tag_groups`(읽기 전용 `string_array`)에 나열된 이름마다 동일한 이름의 중첩 블록이 있어야 하며, 이 블록이 해당 그룹의 태그를 어떻게 해석할지 선언합니다. 어떤 그룹의 family/ID 범위에도 속하지 않는 태그는 무시됩니다.

그룹별 필드:

| 필드               | 타입   | 기본값      | 비고 |
|--------------------|--------|-------------|------|
| `family`           | string | `tag36h11`  | `tag16h5`, `tag25h9`, `tag36h10`, `tag36h11`, `tagCircle21h7`, `tagCircle49h12`, `tagCustom48h12`, `tagStandard41h12`, `tagStandard52h13` 중 하나. 여러 그룹이 같은 family를 공유해도 됨. |
| `id_begin`         | int    | `0`         | 시작 ID (포함). |
| `id_end`           | int    | `0`         | 끝 ID (포함). `id_begin` 이상이어야 함. |
| `size`             | double | `0.05`      | 태그 한 변 실제 길이 (m). 자세 추정 정확도가 이 값에 직결됨. |
| `tag_frame_prefix` | string | `""`        | 그룹별 TF child 프레임 접두어. 비어있으면 글로벌 `tag_frame_prefix`로 폴백. |

두 그룹이 같은 `family`를 공유하는 것은 가능하지만, ID 범위는 겹치지 않아야 합니다. 디텍터는 각 family를 내부 `apriltag_detector_t`에 정확히 한 번만 추가하고, (family, ID) 조합으로 그룹을 디스패치합니다.

#### 실제 예시: 랜드마크 + 도킹 마커

창고 곳곳에 10 cm `tag36h11` 마커를 내비게이션 랜드마크로 부착하고, 도킹 스테이션에는 5 cm `tag25h9` 마커를 사용한다고 합시다. 둘 다 같은 카메라로 들어옵니다. 디텍터를 두 개 띄우지 않고도, TF / 검출 토픽 / 하위 소비자에 이 둘이 깔끔히 구분됩니다.

```yaml
apriltag_detector:
  ros__parameters:
    publish_tf: true
    tag_frame_prefix: "tag_"           # 글로벌 기본값 ('landmarks'가 사용)

    tag_groups: ["landmarks", "dock"]

    landmarks:
      family: tag36h11
      id_begin: 0
      id_end: 99
      size: 0.10                       # 10 cm 벽 마커
      # tag_frame_prefix 미지정 → 글로벌 "tag_" 사용.
      # TF 프레임: tag_0, tag_1, ..., tag_99
      # 검출 메시지: group="landmarks", tf_frame_id="tag_<id>"

    dock:
      family: tag25h9
      id_begin: 100
      id_end: 149
      size: 0.05                       # 5 cm 도킹 마커
      tag_frame_prefix: "dock_"
      # TF 프레임: dock_100, ..., dock_149
      # 검출 메시지: group="dock", tf_frame_id="dock_<id>"
```

이 설정의 효과:

- 내비게이션 스택은 TF 트리에서 `tag_*` 프레임만 사용 — 도킹 마커는 자동으로 시야 밖.
- 도킹 컨트롤러는 `detections` 토픽을 구독해 `group == "dock"`만 필터링하고, `tf_frame_id`로 TF 조회를 바로 수행. ID 범위를 클라이언트 코드에 하드코딩할 필요 없음.
- 환경에 잘못 굴러다니는 `tag36h11` ID 500번 같은 태그(타 사에서 부착한 다른 AprilTag)는 어떤 그룹에도 속하지 않으므로 조용히 폐기됨.

## 메시지

`apriltag3_msgs` 패키지가 다음 두 메시지를 정의합니다.

- `AprilTagDetectionArray` — 입력 이미지와 일치하는 `header` 및 `AprilTagDetection[] detections`.
- `AprilTagDetection` — `group`, `family`, `id`, `tf_frame_id`, `hamming`, `decision_margin`, `size`, `center`, `corners[4]`, `pose`(PoseWithCovariance), `pose_error`.

  - `group` — 매칭된 그룹의 이름 (예: `landmarks`, `dock`). 하위 소비자가 family/ID가 아니라 의미적 역할로 라우팅하기에 좋습니다.
  - `tf_frame_id` — 해당 검출의 TF child 프레임 ID(`<prefix><id>`). 그룹별 prefix override가 있으면 그것을, 없으면 글로벌 기본값을 사용해 계산됩니다. `publish_tf`가 꺼져 있어도 항상 채워지므로 소비자가 TF 그래프에서 태그를 그대로 조회할 수 있습니다.

자세 추정에 실패한 검출도 배열에는 포함되지만 TF는 발행되지 않습니다. 하위 소비자는 `pose_error`와 공분산을 기준으로 신뢰도 필터링을 거는 것을 권장합니다.

## 설치 및 빌드

ROS 2 Jazzy에서 테스트했습니다. 메시지 정의는 별도 패키지 [`apriltag3_msgs`](https://github.com/NTques/apriltag3_msgs)에 있으므로 같은 워크스페이스에 함께 클론하세요.

```bash
# 1. 워크스페이스의 src/에 두 저장소 클론.
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone https://github.com/NTques/apriltag3_msgs.git
git clone https://github.com/NTques/apriltag3_ros.git

# 2. 시스템/ROS 의존성 설치 (apriltag C 라이브러리, image_transport 등).
cd ~/ros2_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# 3. 빌드.
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to apriltag3_ros
source install/setup.bash
```

이후 `ros2 launch apriltag3_ros apriltag_detector.launch.py`가 동작합니다. 런치 인자와 토픽 리맵은 [빠른 시작](#빠른-시작) 섹션 참고.

직접 의존성(위 `rosdep` 명령으로 자동 해결): `rclcpp`, `rclcpp_components`, `sensor_msgs`, `geometry_msgs`, `image_transport`, `image_geometry`, `cv_bridge`, `OpenCV`, `apriltag`, `apriltag3_msgs`, `tf2`, `tf2_ros`, `generate_parameter_library`.

## 라이선스

BSD-2-Clause. [`LICENSE`](LICENSE) 파일을 참조하세요.
