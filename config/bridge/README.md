# Bridge profile contract

정본은 [`bridge_profiles_v1.json`](./bridge_profiles_v1.json)이다. P3-A는 simulator release,
runtime protocol, transport, sensor set을 서로 다른 축으로 고정한다.

- runtime protocol은 `morai_udp` 하나다. 26.R1.H3와 25.S4.MolitComp03 때문에 bridge code를
  분기하지 않는다.
- MORAI network export schema는 release마다 다르므로 network template은 둘로 유지한다.
- `dev_local`과 `competition_lan`은 포트·Ctrl mode가 같고 IP만 다르다.
- 기본값은 `26r1 + dev_local + v2`다.
- Ctrl은 지금부터 `9093/9094`, mode 2가 기본이다. `9111/9112`는 archive 재현에만 쓴다.
- v3의 15 Hz는 UI 저장 값이 확인된 후보지만, 30초 실측 14~16 Hz와 손재호 tracking 비교 전에는
  본선 승인으로 간주하지 않는다.

```bash
python3 tools/audit/validate_bridge_profiles.py
```

현재 `competition_vehicle_status`는 network 계약만 동결됐고 decoder 전환은 다음 P3 작업이다.
따라서 competition JSON이 생성된다는 사실만으로 현장 bridge가 완성됐다고 판단하지 않는다.
