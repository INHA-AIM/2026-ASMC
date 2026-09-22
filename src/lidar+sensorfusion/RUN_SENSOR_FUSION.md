# LiDAR + camera sensor fusion

> **역할:** Active Standalone Guide — 독립 nested catkin workspace의 build·runtime·검증
>
> Phase 4 통합 경계와 중복 package 제거 gate는
> [Perception 개발·통합 가이드](../../docs/packages/asmc_perception/README.md)를 따른다. 이
> workspace는 outer catkin package처럼 취급하거나 parity 확인 전에 이동하지 않는다.

This directory is a self-contained ROS1 Noetic catkin workspace nested under
the `2026-ASMC` repository. `CATKIN_IGNORE` keeps the outer workspace from
recursively building the copied packages; build this directory separately.

The authoritative ASMC v1 poses are:

- front camera: `xyz=(1.90, 0.00, 1.20)m`, `rpy=(0, 2, 0)deg`, 1280x720, HFOV 90 deg
- 3D LiDAR: `xyz=(0.80, 0.00, 1.45)m`, `rpy=(0, 0, 0)deg`
- GPS: `xyz=(0.35, 0.00, 1.25)m`

The runtime copy is in `src/Main/setting.yaml`; the simulator source is the
outer repository's `config/morai/sensors/ASMC_sensor_set_v1.json`.

## Build

```bash
cd /path/to/2026-ASMC
git pull --recurse-submodules
git submodule update --init --recursive
./scripts/build_ws.sh -j2

cd src/lidar+sensorfusion
source /opt/ros/noetic/setup.bash
source ../../devel/setup.bash
catkin_make --force-cmake -j2
```

## Run

Start the outer sensor bridge first:

```bash
cd /path/to/2026-ASMC
source /opt/ros/noetic/setup.bash
source devel/setup.bash
./scripts/udp_bridge.sh --simulator 26r1 publish_map_tf:=true
```

`publish_map_tf:=true` is an opt-in raw GPS + latest IMU transform used by
this standalone map-frame fusion candidate. Do not enable it when localization
already publishes the authoritative `map -> base_link` transform; two TF
authorities for the same edge are invalid. The ordinary team UDP bridge keeps
this option false by default.

In another terminal, start the perception node. Camera-LiDAR class fusion and
the YOLO node are enabled by default:

```bash
cd /path/to/2026-ASMC/src/lidar+sensorfusion
source /opt/ros/noetic/setup.bash
source ../../devel/setup.bash
source devel/setup.bash
./start_main.sh
```

To run LiDAR-only, launch with `enable_sensor_fusion:=false`.

Fusion outputs only `person`, `car`, or `unknown`; YOLO `truck` and `bus` are
mapped to `car`. The YOLO result carries the source image timestamp. Each
LiDAR object frame waits at most 200 ms for the nearest source-stamped YOLO
result and consumes that result exactly once, only when the timestamp offset
is at most 50 ms. The limit is not relaxed on timeout. Every 5 seconds,
`Fusion sync 5s` reports matched/dropped counts, offset median/p95/max, and
pending queue sizes. Initial classification needs 5 consistent matches, an
established class changes after 3 consistent opposite matches, and 3 unmatched
YOLO frames retain the class before the fourth resets it to `unknown`. Class
state is never permanently locked. The broad bottom-image LiDAR exclusion is
disabled by default; the targeted ego-hood YOLO false-positive filter remains.

## Camera inference GPU (CUDA)

GPU 세대별로 이미지만 분리하며 소스 코드는 동일하게 사용한다.

| image | interpreter | PyTorch | GPU inference |
|---|---|---|---|
| `asmc/ros-noetic:dev` (RTX 40) | `/usr/bin/python3` (3.8) | 2.1.2+cu118 | sensor-fusion YOLO + traffic-light ResNet18 |
| `asmc/ros-noetic:rtx50` | `/opt/yolo-venv/bin/python` (3.9) | 2.7.1+cu128 | sensor-fusion YOLO + traffic-light ResNet18 |

ROS Noetic은 Python 3.8에 고정되어 있고 해당 버전용 PyTorch cu118은 RTX 50의
`sm_120` 커널을 포함하지 않는다. 따라서 RTX 50 이미지만 기본 이미지를 상속하고
두 카메라 추론 프로세스용 Python 3.9/cu128 환경을 추가한다. LiDAR, tracking 및
나머지 ROS 노드는 두 이미지 모두 시스템 Python 3.8을 그대로 쓴다.

`Main/scripts/yolo_detection_node`는 이미지가 지정한 인터프리터로
`YOLODetection.py`를 실행한다. `traffic_light/run.sh`도 동일한 방식으로
시스템 Python 또는 RTX 50 venv를 선택한다. 사용자가 컨테이너 안에서 실행하는
`./start_main.sh`, bridge, ROS 명령은 GPU 세대와 무관하게 동일하다.

ROS message support works across the split because `rospy`, `rosgraph`,
`genpy`, `sensor_msgs` and the generated `Msg` package are pure Python and are
reached over `PYTHONPATH`. The CPython 3.8 extension modules in the ROS tree
(`cv_bridge`, `tf2_py`, `camera_calibration_parsers`) are never imported — this
node decodes `CompressedImage` with `cv2.imdecode` directly.

### First run after pulling

RTX 40:

```bash
git pull
./scripts/docker_ros_up.sh build
./scripts/docker_ros_up.sh up
```

RTX 50 (기본 이미지를 먼저 한 번 만들고 파생 이미지를 만든다):

```bash
git pull
./scripts/docker_ros_up.sh build
./scripts/docker_ros_up.sh build-rtx50
./scripts/docker_ros_up.sh up-rtx50

docker exec -it asmc-ros-noetic bash
cd /root/ws/src/lidar+sensorfusion
catkin_make -j2          # picks up the new node wrapper
./start_main.sh
```

The host must already have the NVIDIA driver and the NVIDIA Container Toolkit
installed; the compose file only forwards the GPU (`gpus: all`,
`NVIDIA_VISIBLE_DEVICES`, `NVIDIA_DRIVER_CAPABILITIES`). **Never install an
NVIDIA host driver inside the container** — the toolkit injects the host driver
libraries at runtime, and a second copy breaks the ABI match.

### Checking that it works

`start_main.sh` prints a read-only GPU line before roslaunch:

```
[GPU][OK] NVIDIA GeForce RTX 5070 (sm_120) torch=2.7.1+cu128 torch_cuda=12.8
```

The node itself then logs its preflight, which runs a real CUDA kernel and
synchronizes rather than trusting `is_available()`:

```
YOLO CUDA preflight OK:
  device=cuda:0
  torch=2.7.1+cu128
  torch_cuda=12.8
  torch.cuda.is_available=True
  torch_arch_list=sm_75,sm_80,sm_86,sm_90,sm_100,sm_120,compute_120
  device_count=1
  gpu_name=NVIDIA GeForce RTX 5070
  gpu_capability=sm_120
  CUDA kernel executed and synchronized
YOLO model ready:
  weights=/root/ws/src/lidar+sensorfusion/src/Main/YOLOWeight/yolo11m.pt
  model_device=cuda:0
  warmup_inference=... ms
```

Then, every 5 s:

```
YOLO performance (mean/median/p95 ms over 5.0s, N frames):
  decode=... preprocess=... inference=... postprocess=...
  callback=... e2e=... fps=... device=cuda:0
```

To query the environment by hand:

```bash
docker exec -it asmc-ros-noetic /opt/yolo-venv/bin/python -c \
  "import torch; print(torch.__version__, torch.version.cuda); \
   print(torch.cuda.is_available(), torch.cuda.get_device_name(0)); \
   print(torch.cuda.get_device_capability(0), torch.cuda.get_arch_list())"
```

To confirm the GPU is really doing the work, look for the node as a CUDA
compute process while it runs:

```bash
nvidia-smi                         # process list + utilization + memory
nvidia-smi pmon -c 5               # per-process compute samples
```

A healthy run shows a `python` process from `/opt/yolo-venv` in the compute
list. If YOLO is silently on the CPU instead, that process is absent and the
node burns several hundred percent CPU in `top`.

### If the GPU is not used

`start_main.sh` and the node preflight separate the three causes; work through
them in this order.

1. **Driver / passthrough** — `nvidia-smi` fails inside the container, or
   `torch.cuda.is_available()` is `False`. Check the host driver and the NVIDIA
   Container Toolkit, then that the container was started from
   `docker/ros-noetic/docker-compose.yaml` (it carries `gpus: all`). Do not
   install a driver in the container.
2. **PyTorch architecture** — `is_available()` is `True` but the kernel probe
   fails with `no kernel image is available for execution on the device`, and
   `torch_arch_list` has no entry for your GPU's capability. The image is stale:
   RTX 40은 `rebuild`, RTX 50은 `rebuild-rtx50`을 실행한다.
3. **Wrapper / environment** — the node exits with
   `YOLO 전용 Python 환경이 없습니다: /opt/yolo-venv/bin/python`. The image
   predates this change; rebuild. If the node still starts under
   `/usr/bin/python3`, the workspace was not rebuilt after the pull; run
   `catkin_make` so `devel/lib/Main/yolo_detection_node` exists.

The node never falls back to the CPU on its own. To run on the CPU
deliberately, pass both arguments explicitly:

```bash
./start_main.sh yolo_require_cuda:=false yolo_device:=cpu
```

## LiDAR-only motion decision

`MotionClassifier` keeps its decision history separately from the tracker's
current-frame velocity. Thus `velocity_mps` and `speed_kph` always remain the
latest KF result, while only `is_dynamic`, the final marker text, and the
dynamic log use the filtered decision.

- Ignore the first 3 valid observations.
- Keep the latest 5 accepted signed `(vx, vy)` samples and take each component's
  median before applying `abs`.
- Treat `|vy| >= 3 km/h` as the higher-priority lateral threat. A raw
  `|vy| >= 5 km/h` sample becomes `PENDING` immediately and two same-sign
  samples confirm `DYNAMIC`.
- Treat `|vx| >= 10 km/h` as longitudinal threat only after 5 accepted samples
  and an ENU displacement speed of at least 5 km/h.
- A velocity jump over 10 km/h does not replace the stable history until 3
  candidates have a direction cosine of at least 0.7 and adjacent candidate
  velocities are within 10 km/h.
- Preserve state/history across at most 3 missed observations; a longer miss
  resets only the classifier history. Only `miss_count == 0` objects are
  published to the typed map/final topics.

`PENDING` is deliberately serialized as `is_dynamic=true` for the safety path.
The exact state, raw velocity, decision velocity, ENU displacement check, jump,
sample counts, and age are included in the final-object log/marker diagnostics.
All thresholds are private ROS parameters prefixed with `motion_` in
`src/Main/main.launch`.

`start_main.sh` consumes `/image_jpeg/compressed` and `/velodyne_points` from
the outer bridge. It does not bind UDP 9299. For legacy direct-UDP operation,
stop the outer bridge and run `START_VELODYNE_DRIVER=true ./start_main.sh`.

## RViz

Run `rviz`, set **Fixed Frame** to `map`, and add:

| Display | Topic | Contents |
|---|---|---|
| MarkerArray | `/lidar_tracking_markers` | fitted 3D OBB, track ID, fused class, speed, motion flag, velocity arrow |
| PointCloud2 | `/lidar/debug_clustered` | one centroid per current cluster in the preprocessed LiDAR frame |
| PointCloud2 | `/lidar/debug_ransac` | non-ground points |
| PointCloud2 | `/lidar/debug_roi` | range/ROI-filtered points |
| PointCloud2 | `/velodyne_points` | raw LiDAR cloud |
| PointCloud2 | `/lidar/debug_clustered_map` | the same current-cluster centroids transformed to map ENU |
| Msg/TrackedObjectArray | `/lidar/tracked_objects_map` | numeric map ENU OBB centers/corners and absolute velocity vectors |
| MarkerArray | `/lidar/tracking_markers_map` | map ENU OBB, center, km/h text and one-second velocity vector |
| Msg/TrackedObjectArray | `/last_lidar` | drivable-area-filtered final objects in map ENU, including the fused `classification` and the raw YOLO label `yolo_class` |
| MarkerArray | `/last_lidar_markers` | final OBB, unit velocity arrow, class/YOLO label, speed/age/ENU/road ratio in `velodyne` |
| MarkerArray | `/lidar/object_boxes` | optional fused AABB and class label |
| PointCloud2 | `/lidar/object` | optional fused object-only cloud |
| Image | `/fusion/debug_image` | camera image with projected LiDAR/fusion debug overlay |

The one topic required for OBB + ID + speed visualization is
`/lidar_tracking_markers`.

Its OBB markers are rebuilt from only the objects observed in the current
LiDAR frame (`miss_count == 0`).  Every MarkerArray starts with `DELETEALL`,
so an OBB from a disappeared or predict-only track cannot remain in RViz.
Predict-only tracks may still keep their ID/velocity annotations, but they do
not publish either an OBB or the old fallback AABB.

Useful checks:

```bash
rostopic hz /image_jpeg/compressed
rostopic hz /velodyne_points
rostopic hz /Ego_topic
rostopic hz /YOLO/BBox
rostopic hz /lidar_tracking_markers
rostopic hz /fusion/debug_image
rostopic echo -n1 /lidar_tracking_markers
rosrun rqt_image_view rqt_image_view /camera/image/compressed
```

`/lidar_tracking_markers` is a visualization message, not a stable downstream
tracked-object API. Consumers can inspect it with `rostopic echo`.

The typed downstream and validation API is now `/lidar/tracked_objects_map`.
Its velocity components use m/s; `speed_kph` is display-only. To record one
validation scenario after rebuilding:

```bash
rosbag record --lz4 -O lidar_velocity_test.bag \
  /velodyne_points /gps /imu /Ego_topic /Object_topic \
  /lidar/tracked_objects_map /lidar/tracking_markers_map \
  /lidar/debug_clustered_map
```

After recording, generate the synchronized samples, summary and plots:

```bash
./analyze_velocity_bag.py lidar_velocity_test.bag
```

To lock the comparison to known IDs, add `--gt-id 101 --track-id 7`.

Outputs are written below `lidar_velocity_test_velocity_analysis/`.
