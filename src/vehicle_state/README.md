# vehicle_state

MORAI GPS/IMU를 팀 내부 `asmc_msgs/VehicleState`로 정규화해 `/vehicle/state`에 발행한다.

- 상태 추정 규칙과 검증: [bridge package-group guide](../../docs/packages/asmc_bridge/README.md#4-팀-내부-vehiclestate)
- message 호환 규칙: [asmc_msgs guide](../asmc_msgs/README.md)

본선 후보 GPS/IMU source:

```bash
roslaunch vehicle_state vehicle_state.launch
```

개발 전용 Ego status source:

```bash
roslaunch vehicle_state vehicle_state_ego_dev.launch
```

두 launch를 동시에 실행하지 않는다. 둘 다 `/vehicle/state`를 publish하며 Ego source는 본선용
GPS/IMU fallback이 아니다. GPS/IMU 출력은 `map` frame, 50 Hz이고 입력이 없거나 stale이면
`valid=false`를 유지한다.
