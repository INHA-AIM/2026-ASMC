# morai_sensor_bridge

MORAI Sensor UDP를 ROS1 camera, PointCloud2, GPS와 IMU message로 변환하는 현재 bridge다.

- 구현·frame·timestamp·검증: [bridge package-group guide](../../docs/packages/asmc_bridge/README.md)
- 현재 센서 포트와 실행: [운영 가이드](../../docs/operations/README.md)
- 센서 규정과 v2 기준: [대회 환경](../../docs/competition/README.md)

```bash
roslaunch morai_sensor_bridge morai_sensor_bridge.launch
```

기본 runtime은 RGB 4대, VLP16, GPS와 IMU를 열며 개발 편의를 위해 Semantic camera와 Instance
LiDAR optional port도 연다. optional stream은 v2에서 idle이지만, 본선 allowlist를 만족하는
별도 launch/profile은 Phase 3에서 구현해야 한다.

`timestamp_mode=receive`가 기본이다. `publish_map_tf=false`를 유지해 localization의 authoritative
`map→base_link`와 충돌하지 않게 한다. `base_link→velodyne` static TF는 기본 발행한다.

```bash
rostopic echo /morai_sensor_bridge/diagnostics -n1
```
