# P3-D canonical bridge façade baseline

> 기준일: 2026-09-23
>
> 기계 판독 정본: [`config/bridge/bridge_profiles_v1.json`](../../../config/bridge/bridge_profiles_v1.json)

## canonical 진입점

```bash
./scripts/udp_bridge.sh
# 동일 launch
roslaunch asmc_bridge bridge.launch
```

`asmc_bridge/bridge.launch`는 다음을 한 번에 기동한다.

- `morai_udp_bridge`: Ego/Competition/Collision, Ctrl
- `morai_sensor_bridge`: RGB 4대, VLP16, GPS, IMU
- `vehicle_state`: GPS/IMU→`asmc_msgs/VehicleState`

기본은 `morai_udp + local + competition`, `enable_vehicle_state=true`다. 기존 stack이
vehicle-state publisher를 소유하는 일시적 호환 실행에서만 `enable_vehicle_state:=false`를
사용한다. `/vehicle/state` publisher를 두 개 동시에 실행하지 않는다.

## 보존 경계

P3-D는 composition façade이며 구현 node의 물리 이동이 아니다. 다음을 rollback·parity
기준으로 유지한다.

- `integration_launch/morai_udp_bridges.launch`
- `morai_udp_bridge`
- `morai_sensor_bridge`
- `vehicle_state`

하위 package 삭제는 packet fixture, raw/result bag replay, consumer build, local MORAI live 수신,
Competition packet capture와 LAN2LAN 리허설 후에만 검토한다.

## 회귀 gate

- façade와 호환 launch의 Network/Sensor node 목록·parameter parity
- façade에만 GPS/IMU vehicle-state node가 기본 추가됨
- competition profile에서 privileged publisher가 없음
- `/vehicle/state` 50 Hz와 `valid=true` 실측
- 실제 local MORAI topic rate·diagnostics 확인
