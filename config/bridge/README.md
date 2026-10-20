# Bridge profile contract

정본은 [`bridge_profiles_v1.json`](./bridge_profiles_v1.json)이다. P3-A에서 network JSON의
simulator release, runtime protocol, transport, sensor set을 서로 다른 축으로 고정했고,
P3-B에서 Competition Vehicle Status decoder와 fixture 계약을 추가했다.
P3-C에서 `competition`/`development` stream policy와 v2/v3 sensor transport 파생 계약을
추가했다.
P3-D에서 `asmc_bridge/bridge.launch`를 canonical façade로 추가해 Network, Sensor,
GPS/IMU `/vehicle/state`를 한 번에 기동한다.
P3-E local 실측에서 v3 LiDAR 프레이밍을 물리 회전 기준으로 고쳤고, 설정·소비 15 Hz와
허용되는 실제 완성 scan 약 12 Hz를 구분했다. UDP `7502`의 `#TrafficLight$` header도
Competition Vehicle Status와 분리해 기록했다.
P3-F에서 local sensor/state, Ctrl closed loop와 bag replay parity를 근거로 Network/Sensor/
VehicleState와 component launch를 `asmc_bridge` 한 패키지로 물리 통합했다.
P3-G에서 참가팀 local과 주최 현장 수락 조건을 분리해 Competition Vehicle Status는 local에서
선택, `--require-competition-status`를 준 현장 gate에서만 필수로 고정했다.
P3 closeout 정리에서 중간 `morai_udp_bridges.launch`를 제거하고 canonical `bridge.launch`가
Network/Sensor component와 VehicleState를 직접 조합하도록 단순화했다. 일반 사용자는
`bridge.launch`만 실행하고 component launch는 단위 시험과 선택적 include에만 사용한다.

- runtime protocol은 `morai_udp` 하나다. 26.R1.H3와 25.S4.MolitComp03 때문에 bridge code를
  분기하지 않는다.
- MORAI network export schema는 release마다 다르므로 network template은 둘로 유지한다.
- `local`과 `lan2lan`은 포트·Ctrl mode가 같고 IP만 다르다.
- bridge 기본값은 `morai_udp + local + competition + v3`다. v3가 설정·소비 15 Hz 본선본이고
  v2 10 Hz는 비교·회귀용이다. simulator release는 bridge 실행 인자가 아니라 MORAI UI에
  import할 JSON을 고를 때만 사용한다.
- Ctrl은 지금부터 `9093/9094`, mode 2가 기본이다. `9111/9112`는 archive 재현에만 쓴다.
- Ego Vehicle Status는 release별 PUBSUB 값만 다르고 포트는 `9088/9099`로 통일한다.
- Competition Vehicle Status는 MolitComp03의 `787 / 9797→9798` 항목이다. 참가팀 local
  검증에서는 선택 사항이며 decoder와 topic은 그대로 연다. 주최 현장 수락에서만
  `--require-competition-status`로 필수화하며 live packet gate는 아직 남아 있다.
  Local 선택 입력의 관측된 `#DetailEgoInfo$`·105B body는 정상 envelope일 때만1회 INFO와
  `UNSUPPORTED` 진단으로 분리한다. 정상 decode/발행으로 세지 않으며, 미지·손상 형식과 필수
  gate/LAN2LAN의 불일치는 계속 오류다.
- `258 / 7603→7502`는 live `#TrafficLight$`인 simulator oracle이며 Competition Vehicle
  Status로 해석하거나 canonical bridge에서 소비하지 않는다.
- type 258과 GetTrafficLight query는 별개다. 조사한 두 release와 주최 export에는
  MoraiTLServiceProvider type 778이 없으므로 query를 필수 계약으로 두지 않는다.
- v3는 UI 저장값과 손재호 코드의 소비 loop가 15 Hz다. local raw UDP 완성 scan 약 12 Hz는
  요구상 허용하며, bridge가 중복 cloud를 만들어 15 Hz로 부풀리지 않는다.
- 기본 `competition`은 Object/Semantic/Instance publisher를 생성하지 않는다. 이 입력은
  `./scripts/udp_bridge.sh --gt`에서만 추가한다. 내부 이름은 `development`를 유지하며
  LAN2LAN과의 결합은 거부한다. 이전 `--streams`/`ASMC_STREAMS`는 제거했다.
- GT 센서셋은 Intensity·Instance LiDAR 모두15Hz다. 이전10Hz SHA는 계약에 보존하며
  현재 파일의 주기·frequencyIndex 외 설정은 바꾸지 않았다. MORAI UI 재로드 후 적용된다.
- `sensors/generated/` v2/v3 파생본은 UDP IP만 transport에 맞게 바꾸며 GT LAN2LAN은
  생성하지 않는다.

```bash
python3 tools/audit/validate_bridge_profiles.py
python3 tools/bridge/render_morai_sensor_profiles.py --check
```

현재 `competition_vehicle_status`는 정적 decoder·fixture·ROS publish 회귀 기준까지만
적용됐다. local live 불일치는
[P3-E 기록](../../docs/records/bridge/bridge-p3e-local-live-v1.md), 물리 통합은
[P3-F 기록](../../docs/records/bridge/bridge-p3f-consolidated-v1.md), local/field 요구 분리는
[P3-G 기록](../../docs/records/bridge/bridge-p3g-local-closeout-v1.md)에 남겼다. generated JSON과
synthetic test의 성공을 현장 승인으로 해석하지 않는다.

**채점 채널 확인:** 팀 generated LAN JSON은 비알고리즘 채널을 loopback으로 옮기는
정책이다. 주최 원본의 type258/770/772/259 목적지와 달라 채점 프로그램까지 연결됐다고
보장하지 않는다. [실제 대조](../../docs/records/bridge/2026-10-20-lan-channel-review.md)와
[두 PC 준비](../../docs/operations/lan2lan-rehearsal.md)를 확인하며, 주최 PC의 전체 network
설정을 팀 생성본으로 덮어쓰지 않는다. 알고리즘 oracle 차단과 별도 judge 소비는 구분한다.
