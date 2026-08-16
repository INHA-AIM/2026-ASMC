# LiDAR + camera sensor fusion

This directory is a self-contained ROS1 Noetic catkin workspace nested under
the `2026-ASMC` repository. `CATKIN_IGNORE` keeps the outer workspace from
recursively building the copied packages; build this directory separately.

The authoritative ASMC v1 poses are:

- front camera: `xyz=(1.90, 0.00, 1.20)m`, `rpy=(0, 2, 0)deg`, 1280x720, HFOV 90 deg
- 3D LiDAR: `xyz=(0.80, 0.00, 1.45)m`, `rpy=(0, 0, 0)deg`
- GPS: `xyz=(0.35, 0.00, 1.25)m`

The runtime copy is in `src/Main/setting.yaml`; the simulator source is the
outer repository's `config/ASMC_sensor_set_v1.json`.

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
./scripts/udp_bridge.sh
```

In another terminal, start fusion:

```bash
cd /path/to/2026-ASMC/src/lidar+sensorfusion
source /opt/ros/noetic/setup.bash
source ../../devel/setup.bash
source devel/setup.bash
./start_main.sh
```

`start_main.sh` consumes `/image_jpeg/compressed` and `/velodyne_points` from
the outer bridge. It does not bind UDP 9299. For legacy direct-UDP operation,
stop the outer bridge and run `START_VELODYNE_DRIVER=true ./start_main.sh`.

## RViz

Run `rviz`, set **Fixed Frame** to `base_link`, and add:

| Display | Topic | Contents |
|---|---|---|
| MarkerArray | `/lidar_tracking_markers` | fitted 3D OBB, track ID, fused class, speed, motion flag, velocity arrow |
| PointCloud2 | `/lidar/debug_clustered` | points retained by clustering |
| PointCloud2 | `/lidar/debug_ransac` | non-ground points |
| PointCloud2 | `/lidar/debug_roi` | range/ROI-filtered points |
| PointCloud2 | `/velodyne_points` | raw LiDAR cloud |
| MarkerArray | `/lidar/object_boxes` | optional fused AABB and class label |
| PointCloud2 | `/lidar/object` | optional fused object-only cloud |
| Image | `/fusion/debug_image` | camera image with projected LiDAR/fusion debug overlay |

The one topic required for OBB + ID + speed visualization is
`/lidar_tracking_markers`.

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
tracked-object API. Consumers can inspect it with `rostopic echo`; add a typed
tracked-object message before using this output as a planning interface.
