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

VLP16은 기본 `lidar_frame_mode=azimuth`로 물리 회전 경계마다 PointCloud2를 만든다. 이전
고정 packet-count 방식은 v3에서 출력률을 약 8 Hz로 낮춰 보이게 했다. 2026-09-23 local
실측에서는 보정 뒤 raw 완성 scan이 약 12 Hz다. 팀 요구는 v3 설정과 손재호 소비 loop가
15 Hz인 것이므로 이 도착률은 허용하며, bridge는 중복 cloud를 재발행하지 않는다.
`packet_count`는 과거 재현이 필요한 경우에만 명시한다.

```bash
rostopic echo /morai_sensor_bridge/diagnostics -n1
```
