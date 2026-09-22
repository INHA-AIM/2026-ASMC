# Bridge profile contract

정본은 [`bridge_profiles_v1.json`](./bridge_profiles_v1.json)이다. P3-A에서 network JSON의
simulator release, runtime protocol, transport, sensor set을 서로 다른 축으로 고정했고,
P3-B에서 Competition Vehicle Status decoder와 fixture 계약을 추가했다.

- runtime protocol은 `morai_udp` 하나다. 26.R1.H3와 25.S4.MolitComp03 때문에 bridge code를
  분기하지 않는다.
- MORAI network export schema는 release마다 다르므로 network template은 둘로 유지한다.
- `local`과 `lan2lan`은 포트·Ctrl mode가 같고 IP만 다르다.
- bridge 기본값은 `morai_udp + local`이고 센서 회귀 기준은 v2다. simulator release는 bridge
  실행 인자가 아니라 MORAI UI에 import할 JSON을 고를 때만 사용한다.
- Ctrl은 지금부터 `9093/9094`, mode 2가 기본이다. `9111/9112`는 archive 재현에만 쓴다.
- Ego Vehicle Status는 release별 PUBSUB 값만 다르고 포트는 `9088/9099`로 통일한다.
- Competition Vehicle Status `7603/7502`는 별도 채널이며 `/Competition_topic`으로
  발행한다. 정적 바이너리 계약과 synthetic fixture는 검증했지만 대회 시뮬레이터
  live packet capture는 배포 승인 gate로 남아 있다.
- v3의 15 Hz는 UI 저장 값이 확인된 후보지만, 30초 실측 14~16 Hz와 손재호 tracking 비교 전에는
  본선 승인으로 간주하지 않는다.

```bash
python3 tools/audit/validate_bridge_profiles.py
```

현재 `competition_vehicle_status`는 정적 decoder·fixture·ROS publish 회귀 기준까지 적용됐다.
competition JSON이 생성되고 synthetic UDP test가 통과했다는 사실만으로 현장 bridge가
승인됐다고 판단하지 않고 live capture와 실제 두 PC 검증을 별도로 남긴다.
