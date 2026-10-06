# K-City 2025 competition metadata v1

> **상태:** Frozen Metadata — P1-C
> **원본 설정:**
> [`kcity2025_competition_metadata_source.yaml`](../../../src/asmc_hd_map/config/kcity2025_competition_metadata_source.yaml)
> **생성 bundle:**
> [`kcity2025_competition_metadata.yaml`](../../../src/asmc_hd_map/config/kcity2025_competition_metadata.yaml)
> **토픽 계약:** [HD Map topic contract v1](../hd-map/topic-v1.md)

## 1. 정본과 파생값

체크포인트 위치·원본 yaw·3 m 반경은 주최 측 JSON을 정본으로 삼는다. 파일은 저장소로 다시
복사하지 않고 `$ASMC_DATA`에서 읽으며 SHA-256을 source config에 고정한다. 생성 bundle에는
필요한 값과 원본 SHA만 담는다.

| 구분 | 권위 | 처리 |
|---|---|---|
| checkpoint x/y/z, yaw, 3 m | official | 값을 변경하지 않고 복제 |
| quaternion | derived_official | 공식 yaw degree로부터 Z축 회전 생성 |
| route/link station | derived_official | 공식 path와 검토된 39-link route에 XY 투영 |
| 기본 60 kph | official | 전역 규칙으로 기록 |
| 출발 5%, 고주로 link corridor | derived_official | 공식 규칙을 route station으로 계산 |
| 신호등 상시 인지·GPS 음영·장애물·회전교차로·합류·주변 차량 | team_final | 팀 질의응답·현장 판단 또는 공식 그림의 팀 link 해석. 2026-10-05 팀 기준값, 판제 적용 가능·수정 가능 |
| 신호등 경계 5개 | team_final | stage14의 traffic-light ID가 있는 검수 정지선에서 파생, 2026-10-05 팀 기준값 |
| 링크별 유효 제한속도 | derived_official | MGeo `max_speed`를 공식 속도 구간 상한으로 자른 값 |

GPS 음영 후보는 전역경로 `A126` 구간이며, HD맵 주행면에는 그 옆 반대방향 링크 `A131`을
회피 가능 차로로 함께 제공한다. `gps_shadow_candidate.rule_payload_json`의
`usable_opposite_lane_link_ids=[A2256W000131]`과
`drivable_area_bridge_id=gps_shadow_A126_A131`이 이 관계를 전달한다. 반대차로는 사용 가능 영역이지
주행 의무가 아니다.

입력 다섯 개의 SHA가 source config와 다르면 `--write`도 실패한다. 주최 파일이나 route/map이
바뀌면 변경을 자동 수용하지 않고 source config의 SHA와 파생 결과를 함께 리뷰해야 한다.

## 2. 체크포인트 결과

규정집 v1.2(2026-09-30) 기준 START/END와 1~14, 총 15개다. 2026-08-03 공지의 체크포인트
11(`A411` 끝, `(66.885, 89.198)`)이 삭제됐고 12~15가 위치 변경 없이 11~14로 재번호됐다.
보존한 이전 파일은 `$ASMC_DATA/R_KR_PG_K-city_2025/2026_molit_comp_checkpoints_20260803_16pt.json`이다.

route 투영 오차와 선택된 link 투영 오차는 모두 `0.0 m`다. 공유 노드에서는 두 link가 같은
좌표를 가지므로 P1-A에서 검토한 link ownership을 명시적으로 고정했다.

| CP | route s (m) | link | link s (m) |
|---:|---:|---|---:|
| START/END | 0.00 | `A2256W000748` | 0.00 |
| 1 | 72.63 | `A2256W000748` | 72.63 |
| 2 | 186.29 | `A2256W000236` | 26.28 |
| 3 | 237.42 | `A2256W000215` | 28.96 |
| 4 | 346.74 | `A2256W000222` | 30.84 |
| 5 | 471.09 | `A2256W000205` | 0.00 |
| 6 | 580.87 | `A2256W000318` | 30.25 |
| 7 | 635.11 | `A2256W000083` | 25.24 |
| 8 | 858.50 | `A2256W000304` | 223.39 |
| 9 | 930.60 | `A2256W000866` | 15.76 |
| 10 | 1118.74 | `A2256W000846` | 134.03 |
| 11 | 1593.59 | `A2256W000445` | 75.34 |
| 12 | 1741.72 | `A2256W000153` | 148.13 |
| 13 | 1859.64 | `A2256W000446` | 78.08 |
| 14 | 1996.73 | `A2256W000126` | 118.99 |

폐곡선의 첫 점과 마지막 점은 같지만 `START_END.route_station_m=0`으로 고정한다. finish 의미는
`is_finish=true`로 따로 보존한다. 전체 길이는 `2184.611723336 m`, 출발 성공 5% station은
`109.230586167 m`다.

## 3. 미션 권위와 소비 제한

생성된 15개 zone은 `derived_official` 4개, `team_final` 11개다. 신호등 인지는 `A236` 시작부터
`A308` 종료까지만 필요한 별도 activation policy로 둔다. 이 policy는 공간 미션이 아니며
`runtime_activation_hint=enable_within_zone_only`, `disable_outside_zone=true`를 전달하므로 후속
runtime에서 구간 밖 신호등 노드를 중지할 수 있다. 실제 정지 위치는 이 policy 안의
traffic-light ID 기준 경계 5개로 분리하며 같은 신호에 속한 인접 차로 정지선은 하나로 묶었다.

속도 정보는 우선순위가 필요한 “전 구간 60 + 예외” 중첩 표현 대신 다음 유효 구간 세 개로
분할한다. 세 구간은 gap과 overlap 없이 전역경로를 정확히 한 번 덮어야 하며 validator가 이를
검사한다.

| 유효 구간 | station | 링크 | 대회 최고속도 |
|---|---:|---|---:|
| 고주로 이전 | `0.00–1118.74 m` | `A748` 시작→`A846` 종료 | 60 km/h |
| 고주로 | `1118.74–1741.72 m` | `A411` 시작→`A153` 종료 | 별도 상한 없음 |
| 고주로 이후 | `1741.72–2184.61 m` | `A451` 시작→`A751` 종료 | 60 km/h |

주최 MGeo의 link `max_speed`는 이 구간과 맞지 않는다. 60 km/h 구간의 `A202`·`A315`는 70,
고주로 예외 구간의 `A411`·`A420`·`A408`·`A445`는 120, `A153`은 50이다. 판제는 원본 link 속성
대신 `route_speed_limits`를 읽는다. route link 39개마다 `mgeo_max_speed_kph`,
`competition_cap_kph`(60 또는 `null`)와 `effective_limit_kph = min(MGeo, 상한)`을 담고, validator가
순서·station·소속 구간·min 규칙을 검사한다. 60 km/h 상한 때문에 MGeo보다 낮아진 link는 A202·A315
두 개다. 주최 MGeo 원본은 수정하지 않는다.

`A866→A054→A846`은 회전교차로 종료 후 고주로 진입 전의 일반 전역경로 추종 구간이다. 기존
`highway_merge_candidate`는 삭제했으며 A054·A846을 포함하는 별도 주행 미션은 없다.

장애물 구간은 주최 측 경로 그림처럼 교차로와 그 앞뒤 접근·복귀 거리를 포함한다(2026-10-06). 링크 중간에서
시작·끝나는 경계는 `link_fraction_span` bounds로 표현한다.

| 구간 | 미션 구간 (route station) | 장애물이 놓이는 링크 (`obstacle_expected_link_ids`) |
|---|---|---|
| 정적 장애물 | `A213` 중간 → `A220` 끝, 276.7–365.8 m | `A222` 교차로 |
| 동적 장애물 | `A202` 중간 → `A315` 30%, 396.8–503.2 m | `A728→A599→A207→A205` |

판제는 미션 구간 전체에서 장애물 대응을 준비하고, 장애물 자체는 교차로 링크에 있다고 본다. 시나리오 생성기도
`obstacle_expected_link_ids`와 같은 링크에만 스폰한다(`kcity2025_mission_scenario_source.yaml`). 두 경계는 주최 측
수치 공지가 아니라 팀 질의응답과 경로 그림에 따른 팀 기준값(`team_final`)이며, 필요하면 source를 고쳐 다시 만든다.

규정집 v1.2의 판정 사실은 각 zone의 `rule_payload_json`에 기록한다. 속도는 Ego Vehicle Status
속도로 판정하며 초과 즉시 15초, 3초 지속마다 15초가 추가된다. 신호 경계 5개는 앞바퀴가 정지선을
넘는 순간의 신호(초록 또는 좌회전)로 판정하고, 위반은 15초, 빨간불에 감속 없이 통과하면 해당 차시
실패다. GPS 음영은 GPS 전체 blackout이고 차로 준수 패널티가 없으며, 구간 안의 랜덤 미션은 장애물
회피뿐이다. 장애물·끼어들기는 CollisionData로 충돌마다 15초다.

`team_final`은 주최 수치가 없어 팀이 검수한 **현재 기준값**이다. 판제가 적용할 수 있다는 뜻이며 고정값이라는
뜻은 아니다. 판제 연동이나 주행 검증에서 문제가 나오면 source의 bounds·payload를 고치고 아래 수정 절차로
다시 만든다. builder는 `rule_payload_json`에 `authority=team_final`을 자동으로 넣고
`planning_enforcement_allowed=true`로 둔다. source의 `review`(검수일·결정 등)는 선택 사항이며, 있으면
payload에 그대로 복사한다. 동결된 `MissionZone` 메시지에는 team-final enum이 없으므로 publisher는 이 zone을
`STATUS_DERIVED_OFFICIAL`로 발행한다. 근거 수준을 구분해야 하는 consumer는 payload의 `authority`를 읽는다.

### 구간·속도 수정 절차

1. `kcity2025_competition_metadata_source.yaml`의 zone `bounds`, `rule_payload`, `traffic_light_zones`를 고친다.
   체크포인트 원본이 바뀌면 `input_sha256.official_checkpoints`도 같이 고친다.
2. `python3 src/asmc_hd_map/scripts/rebuild_hd_map_artifacts.py`를 한 번 실행한다. metadata, P2-A audit의
   metadata pin, audit, compact product, runtime pin, 문서 이전 manifest의 기록 SHA까지 순서대로 갱신한다.
3. `git diff`로 바뀐 zone과 station을 검토하고 `asmc_hd_map` 테스트를 돌린다. 현재 값을 그대로 적어 둔
   테스트(예: 정적 장애물 `A222`)는 의도한 변경이면 같이 고친다.
4. `--check`로 다시 실행하면 아무것도 쓰지 않고 stale 단계만 보고한다.

`provisional` zone은 지금 없지만 규칙은 유지한다. `planning_enforcement_allowed=false`여야 하며, 이 값이
true이거나 status가 비어 있으면 validator가 실패한다.

## 4. 재생성과 검증

```bash
python3 src/asmc_hd_map/scripts/build_competition_metadata.py --check
PYTHONPATH=src/asmc_hd_map/src \
  python3 -m unittest src/asmc_hd_map/test/test_competition_metadata.py
```

`--write`는 source config를 검토해 변경한 경우에만 사용한다. 출력은 timestamp를 넣지 않으며
같은 입력에서 byte-for-byte 동일하다.

P1-A runtime baseline v1은 content commit `73c0a8c`의 repository snapshot이며 현재 metadata
검증 명령에 포함하지 않는다. 현재 HEAD 전체를 다시 동결하려면 v1을 덮어쓰지 않고 새 baseline
version을 만든다.

## 5. 아직 하지 않는 일

P1-C 자체는 metadata 생성·검증까지만 담당한다. 후속 P1-D가 이 frozen bundle을 읽어
`CompetitionCheckpointArray`/`MissionZoneArray`와 heading·3 m 원을 read-only로 발행한다.
P1-D도 차선과 주행가능영역 geometry를 변경하지 않으며 실행 규약은
[HD Map runtime·visualization v1](../hd-map/runtime-v1.md)을 따른다.
