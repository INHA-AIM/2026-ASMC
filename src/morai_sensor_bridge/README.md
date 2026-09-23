# morai_sensor_bridge

MORAI Sensor UDP를 ROS1 camera, PointCloud2, GPS와 IMU message로 변환하는 현재 bridge다.

- 구현·frame·timestamp·검증: [bridge package-group guide](../../docs/packages/asmc_bridge/README.md)
- 현재 센서 포트와 실행: [운영 가이드](../../docs/operations/README.md)
- 센서 규정과 v2 기준: [대회 환경](../../docs/competition/README.md)

```bash
roslaunch morai_sensor_bridge morai_sensor_bridge.launch
```

기본 `competition` profile은 RGB 4대, VLP16, GPS와 IMU만 연다. Semantic camera와
Instance LiDAR는 `./scripts/udp_bridge.sh --streams development`에서만 열리며 LAN2LAN과
결합할 수 없다.

`timestamp_mode=receive`가 기본이다. `publish_map_tf=false`를 유지해 localization의 authoritative
`map→base_link`와 충돌하지 않게 한다. `base_link→velodyne` static TF는 기본 발행한다.

```bash
rostopic echo /morai_sensor_bridge/diagnostics -n1
```
