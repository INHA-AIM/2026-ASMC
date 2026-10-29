# 세 재밍 방식의 책임과 파라미터

2026-10-30 비교 통합 브랜치 기준. LOCAL 완료 checkpoint `5f9e1cfa`의 후보
우선순위와 근거리 관측 기반 방향 비용을 선별했다. 원래 작업트리와 그 실험의
차폭·복구 설정은 변경하지 않았다. 실험 후보를 전체 주행 수락으로 해석하지 않는다.

## 실행 책임

```text
공통 bridge/camera/LiDAR 입력
  ├─ NORMAL: 기존 Behavior/Frenet/MPC
  └─ A448부터 선택한 한 구현
       ├─ camera: jamming_lane_boundary_adapter
       ├─ local: 관측 노면/family/OBB 경로 → 공통 MPC
       └─ localization: anchor/DR + map Frenet 회피 → 공통 MPC
공통 ControlSelector → /ctrl_cmd 한 발행자
```

선택 인자는 `jamming_planner:=camera|local|localization`이다. 공통 A448 접근 목표는25km/h,
camera 기본 목표는20km/h다. LOCAL/localization은 순항15·상한20km/h를 공유한다.
상한20은 항상20으로 주행한다는 뜻이 아니다. 순항 목표·곡률·조향 속도·관측 거리·제동
가능 거리·native 후보의 속도 상한 중 작은 값이 실제 계획 속도를 결정한다.
별도 동일 속도 비교는 `jamming_speed_kph:=10`으로 낮출 수 있다.

## LOCAL에 map Frenet 검사가 없는 이유

Frenet은 기준 경로를 따라가는 종방향s와 그 옆의 횡방향d 좌표계다. 충돌 검사 자체의
이름이 아니다. LOCAL은 GPS/map anchor 없이 현재 관측 공간에서 경로를 만든다.
이를 map Frenet 도로 검사에 넣으려면 검증된 map 위치가 필요해 LOCAL의 GPS 독립성이
사라진다. LOCAL도 차체 footprint, 표본 사이 sweep, 현재 LiDAR/운동 예측,
관측 노면·미관측 영역·도로 외곽, 곡률·조향 변화율·정지 거리를 검사한다.
Frenet 패키지의 기하/차체 수학을 공유하는 것과 map 위치에 의존하는 것은 구분한다.

Localization은 anchor/DR로 map pose를 제공하므로 기존 Frenet의 경로·회피·HD맵 검사를
재사용한다. 마지막에는 공통 속도 모듈과 현재 OBB/차체 검사를 통과한다. 두 방식의
노면 근거는 다르다: LOCAL은 관측 raster, localization은 HD맵과 검증된 터널 확장이다.

## 공용 코드와 확장 지점

| 위치 | 책임 |
|---|---|
| `src/asmc_planner/src/Planning/` | 모드·A448 접근·준비된 인계·selector |
| `src/asmc_planner/config/jamming_transition.yaml` | 공통 진입 링크/station·접근25 |
| `src/asmc_planner/config/jamming_motion.yaml` | 두 MPC 방식의 차체·순항·탈출 속도 상한 |
| `src/asmc_planner/src/jamming_common/types.hpp` | 내부 Scene/Config/Result 계약 |
| `src/asmc_planner/src/jamming_common/motion.hpp` | 단일 상대이동 적분기, 명시적 yaw/예측 프로필 |
| `src/asmc_planner/src/jamming_common/geometry.*` | 차체/노면/OBB sweep·좌표 변환 |
| `src/asmc_planner/src/jamming_common/speed_profile.*` | 순항·상한·곡률·제동·시간 매개화 |
| `src/asmc_planner/src/jamming_common/stall_escape.hpp` | 반복 정지 감시와 검사 콜백을 받는 탈출 후보 |
| `src/asmc_planner/src/local/` | LOCAL 기준선·탐색·관측 기반 근거리 복구 |
| `src/asmc_planner/src/shadow_localization/` | map anchor/복귀·Frenet 실행·벽 확장 |
| `src/asmc_controller/src/local/` | 두 방식의 공통 MPC 실행·실제 명령 이력·인계 |

`asmc_local_planner_core`와 `asmc_shadow_localization_core`는 각각
`asmc_jamming_common`에 의존한다. localization이 LOCAL 검색 라이브러리를 링크하지 않는다.
기존 `local/motion.hpp`, `local/speed_profile.hpp`, `local/planner.hpp`는 얇은 호환 include다.
실행 구현이나 적분기를 복제하지 않으며, LOCAL 후속 변경의 include 경로를 보호한다.
내부 `local_planning` namespace와 ROS 메시지 계약은 유지한다.

LOCAL의 rate yaw/100ms 정렬 예측과 localization의 orientation yaw/50ms 예측·map anchor는
별도 프로필이다. localization의 Molit 경험 보정1.54를 LOCAL에 적용하지 않는다.
Frenet/LOCAL 시간 매개화도 검증 이력이 달라 공통 모듈 안에서 구분한다.

## 차체와 공간 여유

차체 폭은 camera, LOCAL, localization 및 이 checkout의 NORMAL Frenet에서1.892m다.
길이4.635m·후륜축에서 뒤범퍼0.790m·wheelbase3m는 유지한다. 추가 여유는 줄이지 않았다.

| 종류/이름 | 적용 위치·의미 | 현재 통합 기본/주의 |
|---|---|---|
| `vehicle_width`, `vehicle_length`, `rear_axle_to_bumper` | 실제 차체 footprint와 차량 기준점 |1.892/4.635/0.790m |
| `clearance_margin` | LOCAL의 차체 밖 추가 물체 여유. 구 설정에서 도로 값 미지정 시 도로 여유의 호환 기본값 |30cm, 물체 불확실성·일부 속도 항목 별도 |
| `road_clearance_margin` | LOCAL raster 비노면 검사에 사용하는 별도 차체 밖 여유 |30cm, 셀 양자화 보정 별도 |
| `preferred_clearance`, `maneuver_clearance` | LOCAL의 넓은 통로 선호 비용 / 근접 기동 감속 판정 |각55cm로 기존 동작 보존. 선호값 미만이라고 무조건 거절하지 않음 |
| `Config.margin` | localization의 추가 최종 OBB 검사 |10cm. native Frenet 검사를 대체하지 않음 |
| `collision_check.safety_margin` | native Frenet의 물체 충돌 여유 |45cm, 시간 증가율0.1m/s 유지 |
| `vehicle_shape.road_outer_width` | 일부 도로·실선·출발·정지선 검사에서 사용하는 보수적인 총 폭 |2.161m. 전 범용 footprint가 아니라 해당 호출에서만 사용 |
| `boundary_reserve_each_side_m` | 위 외형 proxy를 실제 폭과 구분한 편측 차이 |13.45cm. 총 폭과 함께 있으면 `width+2*reserve=road_outer_width`를 검증 |
| `wall_reserve_m` | 터널 확장 자산에 기록된 벽 여유의 선언·최소 검증값 |최소65cm. 로더가 폴리곤을 다시65cm 깎지는 않음. 작성 근거의 재확인은 별도 |
| `jamming_avoid_clearance` | camera 회피선의 차체-장애물 횡간격 |통합5cm. #142 camera 전용 설정 |
| `jamming_obstacle_lateral_margin` | camera 차체 모서리·도로 경계·일부 장애물 검사 |통합5cm. LOCAL 경로 알고리즘 값이 아님 |
| `avoid_plan_margin` | camera 후보 선택 때 추가하는 계획 여유 |`반차폭 + avoid_clearance + avoid_plan_margin`으로 사용 |
| `jamming_avoid_line_overrun` | camera 회피 corridor의 외곽 기준선 바깥 허용 |통합1.5m. camera 외곽 영역 확장 허용이며 충돌 여유가 아님 |
| `jamming_line_edge_expand` | camera의5개 기준선 중 바깥1·5번 가이드 이동 |30cm. 벽/노면 확인이나 차체 통과 보장이 아님 |
| `jamming_avoid_line_extra` | camera가 통과 가능한 회피선에서 더 바깥으로 미는 선 단위 값 |통합0.0. LOCAL 작업트리의1.0은1m라는 뜻이 아님 |
| `avoid_max_shoulder`, `avoid_wall_margin` | camera가 LiDAR 벽을 확인한 쪽의 확장 한계/벽 여유 |통합 shoulder2m. overrun 및 localization 지도 확장과 별개 |
| `uncertainty`, `source_age`, `extra_clearance` | 물체 나이/운동 불확실성과 취약 대상 여유 |실제 footprint 위의 별도 보수성, 무효 속도를0으로 바꾸지 않음 |

### 실제 검사·계산 경로

| 항목·분류 | 설정 로딩·사용 지점 | 계산·조절 효과 |
|---|---|---|
| 실제 제원 `width=1.892`, `length=4.635`, `rear=.790` [m] | `jamming_motion.yaml` → LOCAL/localization private parameter → `MakeEgoFootprint`; native는 `frenet/params.yaml`, camera는 자신의 launch | 전체 폭1.892, 반폭0.946. 앞 범퍼까지3.845; wheelbase3과 앞 overhang0.845는 별도 |
| LOCAL 제원 일치 | 공통 YAML → 알고리즘 YAML → 명시 launch 값. `local/node.cpp`가 최종 제원을 검증 | 알고리즘 설정에서 다른 제원을 조용히 덮지 않고 시작을 거절. 정책 override는 명시 순서에 따름 |
| 도로 외형 proxy | native `ConfigLoader` → `road_outer_width`; `TouchesPaint(all_solids)`, 출발·정지선 등 기존 호출 | `1.892+2×.1345=2.161`m. `DrivablePoseClear`는 전달받은 `shape.width`를 사용. 새 reserve를 모든 호출에 추가하지 않음 |
| LOCAL 물체 하드 여유 | `clearance_margin=.30` → `CheckFootprint` | 현재 OBB에 대한 차체 polygon 거리 ≤ `.30+min(.25,.015v²)+uncertainty+extra`이면 거절. recovery의 현재 polygon에는 속도 항을 빼는 기존 정책이 따로 있음 |
| LOCAL raster·셀 | `road_clearance_margin=.30`, grid 해상도 → `CheckFootprint`·탐색/복구 범위 | 비노면 하드 반폭 `.946+.30+resolution/√2`;0.1m 셀이면1.31671m. 속도 추가 띠는 일부 soft 비용 |
| LOCAL 통로 선호·기동 감속 | `preferred_clearance=.55` → `SegmentQualityCost`; `maneuver_clearance=.55` → `BuildSpeedProfile` | 선호 비용과 근접 기동 속도 상한을 분리. 구 설정에서 새 값 미지정 시 기존 선호값을 따른다 |
| 관측 불확실성 | LOCAL 입력의 `source_age`와 motion budget → `Obstacle.uncertainty` | 기본 `age×max(5m/s, 유효 속도 크기)`; 조용한 과거 형상 예외 등은 별도 조건. 유효하지 않은 속도를 유효0으로 바꾸지 않음 |
| 취약 대상 | LOCAL 원본 class → `extra_clearance` | person·dog 등은 물체 하드 거리의 추가0.25m. 운동 여부와 무관 |
| native 물체 검사 | `collision_check.safety_margin=.45`, `margin_growth_rate=.10` → `EffectiveSafetyMargin` | 예측 시각 t에서 미확정/동적 등은 `.45+.10t`m, LOW_THREAT 확정 등은 조건에 따라 기본값. 각각 독립 판단 |
| localization 최종 물체 검사 | `Config.margin=.10` → `StaticPlanner::footprint` | native 후보 검사와 다른 단계. 일반 후보의 두 검사를 `.45+.10`으로 합산하지 않음. 탈출은 `max(.10,.45)` |
| 벽 확장 | `tunnel_wall_corridor_molitcomp03.json` → `load_wall_corridor` | map SHA·frame·station·선언된 reserve 검증 후 원본 map에 국한해 추가. 실제 바닥·벽 원자료 확인 전 reserve 재설정 금지 |
| camera 후보 | `avoid_clearance=.05`, `avoid_plan_margin=.05` → candidate offset 검사 | 반폭+.05+.05=1.046m. `obstacle_lateral_margin=.05`은 도로/차체·다른 물체 검사의 별도 조건 |
| 속도·정지 공간 | `BuildSpeedProfile` | 경로 남은 거리에서0.10m를 뺀 뒤 제동·지연 계산; LOCAL의0.35m/s 추종 overshoot 보정은 길이 아님. native 시간 프로필은 별도 |
| 위치·운동 보정 | `vehicle_state.yaml`, `shadow_localization.yaml`, controller 설정 | GPS 장착점(.35,0,1.25)m, wheelbase3m, steering gain .844. localization의 orientation yaw·경험 보정을 LOCAL rate yaw에 대입하지 않음 |

실제 effective 값은 `rosparam get`으로 각 노드의 private namespace를 확인하고,
정본 YAML·launch 인자·source revision SHA와 함께 기록한다. 컴파일 기본값은
YAML 미지정 시의 fallback이다. 런타임 파일 SHA와 실제 실행파일 SHA도 별도다.

객체 여유·도로 여유·셀 반경을 무조건 더하지 않는다. 동일 대상에서 단계별로
독립 veto하는지, 하나의 거리식 안에서 누적하는지, 비용/감속에만 쓰는지 위
표를 따른다. 원본 LiDAR hull과 후단 source-age 확장의 중복 여부는 아직 감사
대상이며, 벽 reserve의 원시 측정 재검증도 미완료다.

기존 LOCAL 근거리 복구의 첫4m에는 내부 비노면·일부 미관측을 완화하는 별도
정책이 있다. 엄격한 관측 sweep과 동일하게 설명하지 않는다. 새 탈출 fan과
기존 근거리 복구는 각각 trace와 실제 검증 결과를 구분한다.

조절 기준: 물체 간격은 LOCAL `clearance_margin` 또는 native
`collision_check.safety_margin`, LOCAL 비노면 여유는 `road_clearance_margin`,
native 경계 proxy는 `boundary_reserve_each_side_m` 및 호환 총 폭을 함께,
넓은 후보 선호는 `preferred_clearance`, 근접 기동 감속은
`maneuver_clearance`, 속도는 순항·상한·곡률/제동 정책을 검토한다.
수치를 바꿀 때는 동일 검사 영역을 보존하는 이름 분리와 독립된 실험으로
기록한다. 차체 폭을 통로 튜닝에 사용하지 않는다.

이 항목들은 전부 더해서 쓰는 단일 숫자가 아니다. 객체와 도로 검사가 서로 다르고,
native 후보 검사와 마지막 sweep에서 각각 veto할 수 있다. 특히5cm와 overrun1.5m는
**LOCAL 연구 작업트리의 camera 옵션**에 남은 값이며 LOCAL/localization에 적용되지 않는다.
진행 중인 LOCAL 실행 설정을 통째로 복사해 최신 camera 기본값을 되돌리지 않는다.

## 경로·속도·시간 설정

| 종류 | 대표 설정·역할 |
|---|---|
| 횡방향 후보 | `avoid_offset`/목표d/가이드선: 어디로 피할지. 차체 여유와 별개 |
| 연결·복귀 길이 | camera `avoid_shift_length`7m, native transition/return schedule: 얼마나 길게 연결·본차로 복귀할지 |
| 탐색 거리·시간 | LOCAL20m/native 관측·연장35m, planning budget40ms/cooperative native slice25ms: 계산 범위·비용 |
| 차량 운동 한계 | 최대 곡률0.23/m, 조향 변화율1rad/s, wheelbase3m, gain0.844: 실제 통과 곡률·추종 가능성 |
| 속도 정책 | 두 방식 순항15·상한20km/h. 곡률/조향/정지 거리와 native bypass 상한4.2m/s 등이 추가 제한 |
| 제동·지연 | LOCAL comfort2/controlled3m/s², localization 기존 native 프로필 유지, 실행 지연0.5s: 실제 정지 가능 거리 |
| 가속 증가 | 양의 가속1m/s²; localization의 추가 증가 제한2m/s³ 유지. 긴급 감속을 지연시키지 않음 |
| 경로 기억 | localization 최근0.6s 경로를 현재 OBB/map으로 재검사; 과거 명령 재생 아님 |
| 관측 기억 | LOCAL 최근 노면5s/6m, 새 비노면·OBB veto 유지; 미관측 공간 자동 채움 아님 |
| 동적 판단 | 의미 클래스와 분리한 연속 위치/형상·재이동 근거, 미래 swept envelope |
| 시각·유효성 | 입력350ms/측위150ms 등 단계별 한계, selector250ms watchdog, 인계 epoch/생산자/frame 검증 |
| 측위 기준점 | GPS mount0.35/0/1.25m, 회복1s 검증·25cm/2°: 차량 치수나 충돌 여유와 별개 |

## 반복 정지 후 탈출

복구 목표를3km/h로 고정하지 않는다. 새 고유 source 관측이 들어오는데도 실제 속도가
0.3m/s 미만으로5초씩 두 번 지속되면 탈출 탐색을 우선한다. 연속10초 정지도 포함한다.
짧게 움직였다 다시 멈추면 확인 횟수를 유지하고,2m의 실제 진전·epoch/모드 변경·입력 공백은
누적을 초기화한다. 앞 진행 corridor에 실제 이동 sweep이 있으면 양보하고 탈출 누적을 해제한다.

탈출은 현재 곡률에서 이어지는 좌우9개 fan을 최대8m까지 검사한다. LOCAL은 현재 raster/
LiDAR sweep, localization은 map/OBB와 기존45cm native 물체 여유·도로 외곽 폭을 유지한다.
LOCAL의 기존 관측 근거리 복구도 대안으로 사용한다. 속도는 검증된 공간·조향·제동 거리에
맞추며 탈출 상한5km/h를 사용한다. 강제 속도 하한은 없다. 아직 관측되지 않은 바닥이나
실제 벽/막힌 차체 통로가 모두 거절되면 `stall_escape_no_observed_opening`을 표시한다.
`escape_requested`, `stationary_windows`, 선택 경로 source/정지 이유는 연구 trace에 남긴다.
실제 주행 수락은 LOCAL 작업 완료 후 비교 단계에서 확인한다.


### PR142·144 통합 변경

camera의 운영 목표는20km/h, 회피10, creep3이다. 통합 launch의 `avoid_clearance=.05`, `obstacle_lateral_margin=.05`, `avoid_line_extra=1`, `avoid_line_overrun=1.5m`, `avoid_max_shoulder=2m`를 사용한다. standalone camera launch의 값과 통합 override를 구분한다. 이 여유·정지 정책은 LOCAL/localization에 전파하지 않는다.

LiDAR 표준 profile은 `lidar_wall_vehicle.yaml`; sensor Z 하한-1.55m, 근거리 지면 제거 .23m, 일반/터널 최소 점수3, 진입30m 전·출구10m 후 window를 사용한다. `RearOverhang`은 물리 정본 .790m로 맞춰 기존 .700m 자기차 마스크의 뒤쪽만9cm 늘어난다. sensor mount나 지면 좌표는 바꾸지 않는다. 현재 프로필·window SHA는 최종 통합 manifest에 기록한다.
