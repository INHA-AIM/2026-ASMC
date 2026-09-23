# Bridge profile contract

정본은 [`bridge_profiles_v1.json`](./bridge_profiles_v1.json)이다. P3-A에서 network JSON의
simulator release, runtime protocol, transport, sensor set을 서로 다른 축으로 고정했고,
P3-B에서 Competition Vehicle Status decoder와 fixture 계약을 추가했다.
P3-C에서 `competition`/`development` stream policy와 v2/v3 sensor transport 파생 계약을
추가했다.
P3-D에서 `asmc_bridge/bridge.launch`를 canonical façade로 추가해 Network, Sensor,
GPS/IMU `/vehicle/state`를 한 번에 기동한다. 기존 패키지는 rollback 기준으로 보존한다.
P3-E local 실측에서 v3 LiDAR 프레이밍을 물리 회전 기준으로 고쳤고, 설정·소비 15 Hz와
허용되는 실제 완성 scan 약 12 Hz를 구분했다. UDP `7502`의 `#TrafficLight$` header도
Competition Vehicle Status와 분리해 기록했다.

- runtime protocol은 `morai_udp` 하나다. 26.R1.H3와 25.S4.MolitComp03 때문에 bridge code를
  분기하지 않는다.
- MORAI network export schema는 release마다 다르므로 network template은 둘로 유지한다.
- `local`과 `lan2lan`은 포트·Ctrl mode가 같고 IP만 다르다.
- bridge 기본값은 `morai_udp + local + competition`이고 센서 회귀 기준은 v2다. simulator release는 bridge
  실행 인자가 아니라 MORAI UI에 import할 JSON을 고를 때만 사용한다.
- Ctrl은 지금부터 `9093/9094`, mode 2가 기본이다. `9111/9112`는 archive 재현에만 쓴다.
- Ego Vehicle Status는 release별 PUBSUB 값만 다르고 포트는 `9088/9099`로 통일한다.
- Competition Vehicle Status는 MolitComp03의 `787 / 9797→9798` 항목이다. decoder와
  synthetic fixture는 `#MoraiInfo$` 계약을 검증하지만 competition mode live packet은 아직
  발생하지 않아 `/Competition_topic` 배포 gate가 남아 있다.
- `258 / 7603→7502`는 live `#TrafficLight$`인 simulator oracle이며 Competition Vehicle
  Status로 해석하거나 canonical bridge에서 소비하지 않는다.
- type 258과 GetTrafficLight query는 별개다. 조사한 두 release와 주최 export에는
  MoraiTLServiceProvider type 778이 없으므로 query를 필수 계약으로 두지 않는다.
- v3는 UI 저장값과 손재호 코드의 소비 loop가 15 Hz다. local raw UDP 완성 scan 약 12 Hz는
  요구상 허용하며, bridge가 중복 cloud를 만들어 15 Hz로 부풀리지 않는다.
- 기본 `competition`은 Object/Semantic/Instance publisher를 생성하지 않는다. 이 입력은
  local `development`에서만 활성화하며 LAN2LAN과의 결합은 거부한다.
- `sensors/generated/` v2/v3 파생본은 UDP IP만 transport에 맞게 바꾸며 GT LAN2LAN은
  생성하지 않는다.

```bash
python3 tools/audit/validate_bridge_profiles.py
python3 tools/bridge/render_morai_sensor_profiles.py --check
```

현재 `competition_vehicle_status`는 정적 decoder·fixture·ROS publish 회귀 기준까지만
적용됐다. local live 불일치는
[P3-E 기록](../../docs/records/baselines/bridge-p3e-local-live-v1.md)에 남겼다. generated JSON과
synthetic test의 성공을 현장 승인으로 해석하지 않는다.
