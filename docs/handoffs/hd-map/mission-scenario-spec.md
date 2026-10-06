# 미션 구간 시나리오 spec — 시나리오 생성기·판제 인수

> **문서 역할:** Active Handoff — HD맵 미션 구간을 시나리오 자동 생성기와 판제 검증에 넘기는 방법
> **Handoff 허브:** [README.md](../README.md)
> **원본:** [`kcity2025_mission_scenario_source.yaml`](../../../src/asmc_hd_map/config/kcity2025_mission_scenario_source.yaml),
> [competition metadata](../../contracts/competition/kcity2025-metadata-v1.md)
> **코드:** [`asmc_hd_map/mission_scenario.py`](../../../src/asmc_hd_map/src/asmc_hd_map/mission_scenario.py)

시나리오 생성기는 HD맵 미션 구간마다 조건에 맞게 장애물·NPC를 랜덤 스폰한다. 그다음 플래너로 지역경로를
그려 보고, 타당한 시나리오만 주행 검증에 쓴다. HD맵은 구간 정의와 스폰 후보 pose까지 제공한다. 스폰 실행,
플래너 판정, 승인 기준은 생성기와 판제가 소유한다.

## 1. 아직 바뀔 수 있다

판제가 HD맵 토픽을 실제로 소비해 본 적이 아직 없다. 구간 경계(`team_final`)와 actor 링크는 팀의 현재 기준값일
뿐이다. 생성기나 주행 검증에서 맞지 않는 곳이 보이면 다음을 고친다.

- 구간 경계·payload: `kcity2025_competition_metadata_source.yaml`을 고친 뒤
  `python3 src/asmc_hd_map/scripts/rebuild_hd_map_artifacts.py`를 실행한다.
- 구간별 회피 차로·actor 링크: `kcity2025_mission_scenario_source.yaml`만 고친다. spec은 실행할 때마다
  다시 만들어지므로 pin이나 재생성 단계가 없다.

## 2. 사용법

ROS 없이 Python으로 쓴다. `numpy`, `PyYAML`, `shapely`가 필요하며 `asmc/ros-noetic:dev` 이미지에 있다.

```python
import random
from pathlib import Path
from asmc_hd_map.mission_scenario import MissionScenarioSpec   # PYTHONPATH=src/asmc_hd_map/src

spec = MissionScenarioSpec.load(Path("/root/ws"))
print(spec.warnings)                       # 참조 오류는 예외가 아니라 경고 목록으로 온다
zone = spec.zone("roundabout_candidate")
pose = spec.sample_actor_pose("roundabout_candidate", "npc_vehicle", random.Random(0),
                              longitudinal_margin_m=3.0, lateral_jitter_m=0.3)
# {'x':..., 'y':..., 'z':..., 'yaw_deg':..., 'link_id':..., 'in_compact_drivable':..., 'route_station_m':...}
spec.route_pose(900.0, lateral_offset_m=1.5)   # 전역경로 station과 좌(+)/우(-) 오프셋 → pose
spec.route_station(x, y)                       # 임의 위치 → 전역경로 station, 횡오차
spec.is_drivable(x, y)                         # compact 주행가능 grid (0.2 m)
```

CLI:

```bash
python3 src/asmc_hd_map/scripts/sample_mission_scenarios.py               # 구간·role 목록
python3 src/asmc_hd_map/scripts/sample_mission_scenarios.py \
  --zone roundabout_candidate --role npc_vehicle --n 10 --seed 0 --jitter 0.3
python3 src/asmc_hd_map/scripts/export_mission_scenario_spec.py --out /tmp/spec.json
```

export는 비-Python 소비자용 snapshot이다. git에 넣지 않으며 기본 위치는 `$ASMC_DATA/hd_map_exports/`다.

## 3. spec 필드

| 필드 | 의미 |
|---|---|
| `route_station_start_m`·`end_m`, `route_link_ids` | metadata zone과 같은 전역경로 구간 |
| `status`, `authority` | `derived_official` 또는 `team_final`(팀 기준값) |
| `speed_limits[]` | 구간 route link별 `effective_limit_kph`와 `competition_cap_kph`(`null`이면 상한 없음) |
| `rule` | 규정 v1.2 판정 사실 (`judged_by`, 패널티, 랜덤 미션 종류 등) |
| `maneuver_lanes[]` | 회피·충돌에 쓰이는 compact lane. `direction_vs_route`: `same`/`opposite`/`crossing` |
| `actor_sources[]` | role(`static_obstacle`, `dynamic_obstacle`, `npc_vehicle`)별 스폰 후보 MGeo 링크. centerline xyz, 길이, MGeo 최고속도, from/to node |
| `surface` | 구간 route 링크, maneuver 링크, `surface_adjacent_to_route`·`surface_link_ids`로 더한 차로에 속한 DrivableArea의 union 외곽선과 면적 (주변차량 구간은 고속도로 같은 방향 차로 전체) |

pose 규약은 다음과 같다.
- `map` frame이다. `yaw_deg = atan2(dy, dx)`로, MORAI 체크포인트 yaw와 같은 규약이다.
- `z`는 MGeo 링크 높이다. compact centerline은 z가 0이라 쓰지 않는다.
- `in_compact_drivable=false`는 compact 제품 밖 링크라는 뜻이다(예: 회전교차로 A133·A147). 그 링크가 주행
  불가하다는 뜻은 아니다.

## 4. 현재 구간별 기본값

| zone | 회피·충돌 차로 | actor 후보 |
|---|---|---|
| `static_obstacle_candidate` (A222 교차로) | 정적 그룹 7개 (A221·A746 반대방향 포함) | static_obstacle: A222 |
| `dynamic_obstacle_candidate` (A728→A205) | 동적 그룹 8개 | dynamic_obstacle: 교차 비경로 링크 7개 |
| `gps_shadow_candidate` (A126) | A131 (반대 차로) | static_obstacle: A126, npc_vehicle(선택): A131 |
| `roundabout_candidate` (A148→A151) | A134 | npc_vehicle: A147→A134, A133→A134 (노드 0728에서 ego A146→A151과 충돌) |
| `surrounding_vehicle_candidate` (A411→A153) | — | npc_vehicle: route 좌측 인접 차로 10개 자동 수집 |
| 신호·속도·출발 구간 | — | 없음 (신호는 판정 프로그램이 제어) |

## 5. 생성기·판제 쪽 결정 (HD맵 범위 밖)

- **스폰:** MORAI gRPC로 스폰한다. `tools/aim_scenario_runner`의 `NpcVehicleManager`·`RoutePositionHelper`를
  2025 맵 링크(`A2256W*`)로 옮겨 쓰면 된다. 현재 러너는 legacy KATRI 맵 기준이다.
- **승인 기준(제안):**
  - 스폰 pose가 겹치지 않고, 정적 장애물은 `in_compact_drivable=true`다.
  - 플래너가 구간을 통과하는 지역경로를 찾는다.
  - 그 경로가 장애물과 충돌하지 않고 실선·중앙선 규칙을 지킨다. 음영 구간은 차로 규칙 예외다.
  - 구간 `speed_limits` 안에서 통과한다.
- **플래너 검증 연결:** `frenet_planner_core`는 ROS 없이 C++ 테스트에서 구동된다(`tests/command_avoid_target_test.cpp`
  패턴). 오프라인 판정 하네스를 만들거나, ROS stack과 gRPC 스폰으로 폐루프를 돌린다. 하네스는 planning이 소유한다.
- **런타임 현재 구간:** 주행 중 "지금 어느 구간인지"는 아직 HD맵이 내보내지 않는다. GPS 음영 dead-reckoning을
  포함한 `MissionContext`는 후속 PR에서 추가한다.

## 6. 시각화

`roslaunch asmc_hd_map competition_map_debug.launch`의 `/hd_map/debug/markers`에서 구간 종류별 namespace로
켜고 끈다.

| namespace | 내용 |
|---|---|
| `mission_<type>` | 구간의 전역경로 span (종류별 색) |
| `mission_surface_<type>` | 구간 면적 외곽선 |
| `mission_actors_<type>` | actor 후보 링크와 진행 방향 화살표 (`debug_marker_detail:=full`에서만) |
