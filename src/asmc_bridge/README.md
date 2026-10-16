# asmc_bridge

MORAI Network UDP, Sensor UDP와 GPS/IMU 기반 `/vehicle/state`를 소유하는 단일 ROS1
패키지다. P3-F에서 `morai_udp_bridge`, `morai_sensor_bridge`, `vehicle_state`,
`integration_launch`의 구현을 이 패키지로 물리 통합했다. node와 diagnostics 이름은 소비자
호환을 위해 그대로 유지한다.

```bash
./scripts/udp_bridge.sh
# 같은 canonical launch를 직접 실행할 수도 있다.
roslaunch asmc_bridge bridge.launch
```

기본값은 `morai_udp + local + competition`이며 `/vehicle/state`까지 발행한다. simulator
release는 runtime 인자가 아니라 MORAI UI에 import한 network JSON schema로만 구분한다.
`local`과 `lan2lan`은 수신 포트가 같고 Ctrl 목적지 IP만 다르므로 현장에서만
`--transport lan2lan`을 명시한다.

일반 실행용 공개 진입점은 `bridge.launch` 하나다. 구성요소 단독 시험이나 다른 launch에서의
선택적 include를 위해 다음 component launch를 같은 패키지 안에 둔다.

- `morai_udp_bridge.launch`: Ego/Competition/Collision, `/ctrl_cmd`
- `morai_sensor_bridge.launch`: RGB/LiDAR/GPS/IMU
- `vehicle_state.launch`: GPS/IMU→`asmc_msgs/VehicleState`
- `vehicle_state_ego_dev.launch`: 개발 전용 EgoStatus state source

이전 `integration_launch`에서 옮겨 온 중간 조합 파일 `morai_udp_bridges.launch`는 제거했다.
`bridge.launch`가 Network/Sensor component와 기본 VehicleState를 직접 include하므로 전체 실행에
추가 조합 계층이 필요하지 않다.

Object/Semantic/Instance oracle은 `./scripts/udp_bridge.sh --gt`로만 열 수 있고
LAN2LAN과 함께 사용할 수 없다. `/vehicle/state` authoritative publisher는 하나만 실행한다.
`--streams`와 `ASMC_STREAMS`는 제거했다. 인자 없는 실행은 일반 입력만 받고, `--gt`는
내부 `development` 프로필을 선택한다. 내부 `competition/development` 이름은 MORAI 버전 구분이 아니다.
GT 센서셋의 Intensity·Instance LiDAR 모두15Hz이며, 변경된 JSON을 MORAI UI에서 다시 로드해야 한다.
Bridge의 기본 `lidar_rotation_rate=15`는 조립 참고값이고 실제 센서 송신 주기를 바꾸지 않는다.

v3는 MORAI 저장 설정과 손재호 소비 loop가 15 Hz이며 완성 scan 약 12 Hz 입력은 허용한다.
bridge는 azimuth wrap으로 실제 회전을 조립하고 중복 cloud를 만들지 않는다. MolitComp
Competition Vehicle Status는 type 787 `9797→9798`이고, type 258 `7603→7502`는
TrafficLight oracle이라 production bridge에서 소비하지 않는다.

참가팀 local에서는 type 787 packet이 나오지 않아도 정상이다. decoder와
`/Competition_topic` publisher는 유지하며, 주최 현장판을 검사할 때만 다음처럼 필수화한다.

```bash
./scripts/udp_bridge.sh --transport lan2lan --require-competition-status
./scripts/docker_ros_up.sh verify-field 30 molitcomp03
```

기본 diagnostics는 미수신 Competition 입력을 `optional; no data`로 표시한다.
