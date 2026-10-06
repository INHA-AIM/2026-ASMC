# 규정집 v1.2 영향과 파트별 확인 요청

> **문서 역할:** Active Handoff — 규정집 v1.2(2026-09-30)가 각 파트 runtime에 요구하는 확인 항목
> **Handoff 허브:** [README.md](../README.md)
> **규정 요약:** [대회 환경·규정·공개 자산](../../competition/README.md)

규정 요약은 대회 문서가 소유한다. 이 문서는 v1.2 문구와 현재 `main` 코드를 대조해 각 owner가
결정할 항목만 남긴다. 1~5절 확인일은 2026-10-05이고, 줄 번호는 그 시점의 `main` 기준이다.
6절은 2026-10-06 전달된 신호 채점 후속 공지를 main `c971d6b`와 대조한 문서 인계다.

## 1. 판정 전용 채널 — Ego Vehicle Status

v1.2 2-4: Ego Detail Status, Ego Vehicle Status, Get TrafficLight Status는 판정 전용이며 본선에서
참가팀이 사용할 수 없다. 허용 네트워크 외 사용은 실격이다.

| 항목 | 현재 상태 | 요청 |
|---|---|---|
| bridge 수신 | `morai_udp_bridge.py`가 9099의 Ego Vehicle Status를 `/Ego_topic`으로 발행한다. competition stream profile도 `ego_vehicle_status`를 입력으로 둔다. | 26r1과 공용이므로 수신은 유지한다. runtime 노드의 소비 0개를 본선 gate로 확인한다 (bridge @kante2) |
| LiDAR `Main` | `src/lidar+sensorfusion/src/Main/main.launch:37-39`가 `/Competition_topic`을 `/Ego_topic`으로 remap한다. Ego 속도·조향각이 tracking 절대속도와 `is_dynamic` 판정에 들어간다. | **v1.2 위반 소비다.** 9798 Competition Vehicle Status live acceptance 뒤 remap을 제거하고 `/Competition_topic`을 직접 읽는다 (LiDAR @sonshiny) |
| opt-in 경로 | `state_source:=ego_dev`, `asmc_hd_map` `local_pose_source:=ego_status`, calibration·수집 도구 | 개발 전용으로만 사용한다. 본선 launch 인자에 넣지 않는다 |

Competition Vehicle Status에는 `pos_x/y/z`, `vel_y/z`, `accel_x/y/z`, 타이어 횡력·side slip·
cornering stiffness가 없다. 위치는 GPS/IMU에서만 얻는다.

## 2. 제어 명령 — cmd type 1, ctrl mode 2

v1.2 2-8: Ego Ctrl Cmd는 `cmd type 1`(accel, brake)과 `ctrl mode 2`만 허용한다.

| 송신 노드 | 현재 | 요청 |
|---|---|---|
| `mpc_controller` | `longlCmdType=1` | 유지 |
| `jamming_lane_boundary_adapter` `camera_bbox_lane_change_node` | `src/jamming_lane_boundary_adapter/src/camera_bbox_lane_change_node.cpp:481`이 `enable_control:=true`일 때 `longlCmdType=2`(내장 속도 제어, km/h)를 보낸다. 기본값은 `enable_control=false`다. | 본선에서 이 노드가 `/ctrl_cmd`를 내면 안 된다. 쓰려면 accel/brake 출력으로 바꾼다 (planning) |
| bridge | `ctrl_mode=2`를 고정하지만 `longlCmdType`은 그대로 통과시킨다. | 본선 profile에서 type 1이 아닌 명령을 막을지 결정한다 (bridge) |

## 3. 속도 — 60 km/h와 고주로 예외

공식 규칙은 전 구간 60 km/h이고, `A2256W000411` 시작부터 `A2256W000153` 끝까지는 상한이 없다.
판정은 Ego Vehicle Status 속도로 하며 초과 즉시 15초, 3초 지속마다 15초가 추가된다.

- behavior planner는 MGeo link `max_speed`를 목표 속도로 쓴다. 60 구간의 `A202`·`A315`는 70이다.
- 지금 60 km/h를 넘지 않는 이유는 Frenet `config/params.yaml:110`의 `target_vel: 13.0` m/s뿐이다.
  이 값을 16.67 m/s 이상으로 올리면 A202·A315에서 19.4 m/s를 지령한다.
- `asmc_hd_map` metadata의 `route_speed_limits`가 link별 `effective_limit_kph = min(MGeo, 공식 상한)`을
  제공한다. 미션 구간 15개도 모두 판제 적용 가능 상태다(공식 파생 4, 팀 기준값 `authority=team_final` 11 — 연동하며 문제가 보이면 HD맵 쪽에서 고친다). planning이 이 값을 상한으로 쓰고 안전 여유를 둘지 결정한다 (planning @yuntae12-sudo
  @yangseojun).

## 4. 객체 입력

`src/behavior_planner/launch/autonomy_no_bridges_v2.launch:35-37`의 `morai_objects_adapter`는
`/Object_topic`(MORAI Object Info)을 입력으로 쓴다. competition stream profile은 `/Object_topic`을
`forbidden_topics`에 둔다. 본선 객체 입력 경로(LiDAR tracked objects 등)를 확인한다 (planning).

## 5. 체크포인트와 HD맵

- 체크포인트는 START/END와 1~14, 총 15개다. 2026-08-03 공지의 11번(`A411` 끝)이 삭제됐다.
- 경로를 바꿔도 체크포인트는 순서대로 반경 3 m 안을 지나야 한다. 불연속 통과는 주행 불능이다.
- `/hd_map/global/checkpoints`와 `kcity2025_competition_metadata.yaml`이 15개를 제공한다.
  체크포인트 진행을 추적하는 consumer가 생기면 16개를 가정하지 않는다.

## 6. 신호 채점 후속 공지 — 2026-10-06 반영

규정 원문·변경 전후·출처 식별은 [대회 요약](../../competition/README.md#51-교차로-신호준수-후속-공지--2026-10-06-반영)이
소유한다. 현재 기준은 **신호 미준수 30초**, **판정 대상 교차로 4개소 모두 미준수하면 해당 차시 실패**다.
동일 교차로는 체크포인트 복귀 후 재통과를 포함해 1회만 집계한다. 적색 무감속 통과만을 별도 차시
실패 조건으로 두던 이전 신호 채점 문구는 대체됐다. 신호 준수·일반적인 미션 이행 의무는 유지된다.

| 대상 | 확인된 상태 / 요청 |
|---|---|
| HD맵 | `src/asmc_hd_map/config/kcity2025_competition_metadata_source.yaml`의 `traffic_light_zones.rule_payload`에 `violation_penalty_s: 15`, `red_without_deceleration: run_failure`가 남아 있다. source → 생성 metadata → 발행 payload와 관련 회귀검사를 함께 갱신해야 한다. 이번 PR은 문서만 변경한다. |
| 교차로 식별 | 현재 HD맵 신호 경계 5개와 공식 판정 교차로 4개소를 일대일로 가정하지 않는다. 3-1 공식 신호등 구간과 대조해 대응을 확인한 뒤 판정 교차로별 식별·중복 집계 계약을 정한다. 텍스트만으로 특정 경계를 삭제하지 않는다. |
| 판제·평가 | 이전 패널티/실패 조건을 소비하는 경로가 있는지 감사한다. 총 이벤트 4회가 아니라 서로 다른 판정 교차로 4개소인지 확인한다. 체크포인트 복귀는 같은 차시의 중복 집계 정보를 초기화하는 사유가 아니며, 다음 주행 차시는 별도 평가한다. |
| 카메라·운영 | 초록/좌회전 및 앞바퀴의 정지선 통과 시점 기준은 유지한다. 참가팀 금지 입력인 Get TrafficLight Status를 새 규칙 구현을 위해 연결하지 않는다. |

후속 시험은 같은 교차로 반복 미준수(복귀 후 포함), 서로 다른 1~3개소 미준수, 서로 다른 4개소
모두 미준수를 구분한다. 중복 이벤트의 시간 패널티 적용은 공식 판정과 확인하며 임의로 누적 규칙을
만들지 않는다. 이는 확인할 시험 항목이며 아직 runtime 구현·검증을 완료했다는 뜻이 아니다.

## 7. 완료 조건

- [ ] 본선 launch에서 `/Ego_topic` 구독 노드 0개 (`rosnode info`로 확인)
- [ ] `/ctrl_cmd` packet capture에서 `longlCmdType=1`, `ctrl_mode=2`만 관측
- [ ] 60 구간 최대 지령 속도 ≤ 60 km/h, 고주로 구간 별도 정책 확인
- [ ] 본선 profile에서 `/Object_topic` 없이 객체 입력이 들어옴
- [ ] HD맵 source·생성물·발행 payload의 신호 패널티 30초 및 이전 실패 규칙 제거 확인
- [ ] 공식 판정 교차로 4개소와 HD맵 신호 경계의 대응 확인
- [ ] 동일 교차로 반복·체크포인트 복귀와 서로 다른 4개소 위반을 구분하는 소비·평가 시험
