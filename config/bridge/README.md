# Bridge profile contract

정본은 [`bridge_profiles_v1.json`](./bridge_profiles_v1.json)이다. P3-A는 network JSON의
simulator release, runtime protocol, transport, sensor set을 서로 다른 축으로 고정한다.

- runtime protocol은 `morai_udp` 하나다. 26.R1.H3와 25.S4.MolitComp03 때문에 bridge code를
  분기하지 않는다.
- MORAI network export schema는 release마다 다르므로 network template은 둘로 유지한다.
- `local`과 `lan2lan`은 포트·Ctrl mode가 같고 IP만 다르다.
- bridge 기본값은 `morai_udp + local`이고 센서 회귀 기준은 v2다. simulator release는 bridge
  실행 인자가 아니라 MORAI UI에 import할 JSON을 고를 때만 사용한다.
- Ctrl은 지금부터 `9093/9094`, mode 2가 기본이다. `9111/9112`는 archive 재현에만 쓴다.
- Ego Vehicle Status는 release별 PUBSUB 값만 다르고 포트는 `9088/9099`로 통일한다.
- Competition Vehicle Status `7603/7502`는 별도 채널이며 decoder가 추가되면 항상 함께 연다.
- v3의 15 Hz는 UI 저장 값이 확인된 후보지만, 30초 실측 14~16 Hz와 손재호 tracking 비교 전에는
  본선 승인으로 간주하지 않는다.

```bash
python3 tools/audit/validate_bridge_profiles.py
```

현재 `competition_vehicle_status`는 network 계약만 동결됐고 decoder 전환은 다음 P3 작업이다.
따라서 competition JSON이 생성된다는 사실만으로 현장 bridge가 완성됐다고 판단하지 않는다.
