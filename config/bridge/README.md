# Bridge profile contract

정본은 [bridge_profiles_v1.json](bridge_profiles_v1.json)이다. 기존 v1 wire 키는 호환용으로
유지한다. [MORAI 설정 정본](../morai/README.md)·[파일 목록](../morai/catalog.json)에서
버전과 Local/LAN2LAN을 고른다. 실제 역할은 `verified_status_bindings.molitcomp03`의 UI publisher·endpoint·실측을 따른다.

| 현재 MolitComp03 역할 | UI publisher / wire | 팀 ROS·LAN |
|---|---|---|
| Competition | CompetitionInfoPublisher, type295, 9088→9099, `#MoraiInfo$` 181B | `/Ego_topic`, LAN 필수 |
| 별도 Ego (팀 미사용) | MoraiInfoPublisher, type770, 910→911 | 팀 입력에서 제외 |
| Detail | type787, 9797→9798, `#DetailEgoInfo$` 138B | 로컬 진단만, LAN·수락에서 제외 |

`channels.ego_vehicle_status`는9099 wire 키이고, `channels.competition_vehicle_status`는9798의
과거 호환 이름이다. 키·ROS 메시지 이름이나 header 하나만으로 허용 입력을 판정하지 않는다.
[UI·실측 정정](../../docs/records/bridge/2026-10-23-competition-channel-audit.md)과
[이번 적용 기록](../../docs/records/bridge/2026-10-27-competition-9099-correction.md)을 함께 본다.

- `morai_udp + local + competition + v3` 기본 구성과 canonical `asmc_bridge/bridge.launch`를 유지한다.
- 26.R1.H3와25.S4.MolitComp03은 같은 decoder를 사용하지만 MORAI import schema는 별도다.
  이번 publisher 허용 판단은 MolitComp03에서 확인한 것이며26.R1로 일반화하지 않는다.
- `--require-competition-status`, `competition_status_required`, `verify-field`는9099의
  `/Ego_topic`을 검사한다. 정상9099만으로 통과할 수 있고9798만으로 통과할 수 없다.
- `ego_port=9099`는 기존 인자다. `competition_port=9798`, `enable_competition_status`는
  호환 로컬 진단 인자로 남으며 LAN에서는 수신·발행하지 않는다.
- local 일반·`--gt` 수집과 기존 decoder 단위·부호 변환은 유지한다. GT는 local 전용이다.
- v3의 설정·소비 LiDAR15Hz와 실제 완성 scan 약12Hz를 구분하며 중복 cloud를 만들지 않는다.
- JSON `egoCtrlMode=3`은 Auto, UDP payload `CtrlMode=2`도 별도 enum의 Auto다.
  controller 선택은 `PUBSUB_TYPE=771`이다. Ctrl9093/9094와 steering 설정을 유지한다.
- TrafficLight/Object oracle과 Semantic/Instance GT를 팀 LAN 운영 입력에 추가하지 않는다.

```bash
python3 tools/bridge/render_morai_release_presets.py --check
python3 tools/audit/validate_bridge_profiles.py
```

계약의 P3-E/P3-G 관측과674개 시험 수는 당시 동결 이력이다. 새 검증 결과로 재발행하지 않는다.
과거9798 해석은 위 정정으로 대체하고 원본 기록·template·fixture는 보존한다.

팀 generated LAN preset은 팀 알고리즘 경계다. 별도 주최 채점/녹화 프로그램의 endpoint를
포함한 전체 현장 설정을 이 파일로 덮어쓰지 않는다. 9798의 채점 소비 여부는 미확인이며,
팀 bridge의 비구독과 주최 송신 경로 보존은 별개의 계약이다.
[두 PC·채점 경계](../../docs/operations/lan2lan-rehearsal.md)를 따른다.
