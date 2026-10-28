# 플래너 공통 HD map — v7

모든 Frenet 모드와 고주로 참조선은 main의 공식 v7 벡터·격자 제품을 사용한다.
`avoid_hd_map`이라는 설정 이름은 호환성을 위해 유지한다.

- 정본: `src/asmc_hd_map/config/kcity2025_compact_drivable_map.json`
- build: `kcity2025_reviewed_high_speed_adjacent_drivable_v7`
- `drivable.json`: 정본과 바이트까지 같은 벡터 제품.
- `drivable_grid.rle`: 공식 NPZ의 모든 free cell을 그대로 내보낸 0.2 m 격자.
- `lines.json`: 기존 검토 완료 도색·정지선 자료를 유지한다.

```bash
python3 tools/planning/sync_hd_map.py --check
python3 tools/planning/sync_hd_map.py
```

공식 제품은 수정하지 않는다. 고주로 차로 참조선의 polygon reader는 원본의
정확한 A-B-A 왕복 선분만 제거하고, 같은 정점에서 닫힌 내부 loop는 hole로
보존한 뒤 Boost validity를 다시 검사한다. 좌표 이동·buffer·면적 기반 틈
메우기는 하지 않으며 일반 자기 교차는 거부한다. 실제 v7 제품과 역순 로딩,
좁은 내부 비주행 영역, 외부 점과 차체 포함 검사는 회귀 테스트에서 확인한다.

영역 확대는 이전과 같은 후보 선택이나 폐루프 주행을 보장하지 않는다.
차체 충돌·곡률·가감속·조향 제한과 각 모드의 도로 제약은 별도로 유지한다.
고정 입력 테스트 이후 MORAI/MPC 폐루프 주행으로 확인해야 한다.
