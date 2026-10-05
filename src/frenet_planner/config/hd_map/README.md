# AVOID용 고정 HD map

원본: INHAautonav/2026-ASMC main `1ce434d0082e3aec6cf79145c05484e1f66fac1c`.
최신 main의 planner/perception 코드를 병합하지 않고 지도 자료만 가져왔다.

- `drivable.json`: `src/asmc_hd_map/config/kcity2025_compact_drivable_map.json` 원본. 검증된 영역 102개와 lane 70개의 형상·경계·연결 정보가 포함된다. Git blob `5d7137d19deb05c69997830745a6b6e4b8d8c3b9`.
- `lines.json`: `src/asmc_hd_map/reports/kcity2025_route_curation_20260917/route_curated_candidate_map.json` 원본. Git blob `0031c1872db9e4db189aba0a80f3cd7eea44ee02`. 선의 pattern/functional_role을 사용하며 도색이 끊긴 구간을 임의로 연결하지 않는다.
- `drivable_grid.rle`: 같은 runtime product의 공식 `src/asmc_hd_map/config/kcity2025_compact_drivable_grid.npz` occupancy를 손실 없이 행별 구간으로 표현한 파일. NPZ SHA256 `ee0db44e5d672e58e429a09a68ed119fb92400d2a83a2f304f9ddc9d1e011fd0`. 첫 행은 width height resolution origin_x origin_y, 이후 각 행은 row inclusive_start exclusive_end. 기록된 구간만 free(0), 그 밖은 주행 불가다. 해상도 0.2m.

좌표는 기존 `/vehicle/state`와 같은 map frame, UTM 52N 원점 (302595, 4124145)이다. 전역 경로는 기존 `2026_molit_comp_global_path.txt`를 그대로 사용한다. Lane 중심선으로 경로를 교체하거나 새로운 지도 ROS 노드를 요구하지 않는다. 현재 lane의 인접/연결 정보를 이용한 경로 탐색은 추가하지 않았으며, 이번 라인 적용은 실선 차선 경계 검사까지다.

## 적용 범위

`config/params.yaml`의 `avoid_hd_map.enabled: true`이면 AVOID가 지도 제약을 사용한다. 지도 초기화 실패는 planner Init 실패로 처리한다. false이면 기존 AVOID를 사용할 수 있다. 기본 자료 경로는 설정 파일 옆 `hd_map/`; 별도 설정 파일은 `avoid_hd_map.directory`로 경로를 지정할 수 있다. 테스트용 임시 설정에서 경로가 생략된 경우 빌드 시 package source의 기본 자료로 돌아간다.

`Map/AvoidRoadMap.cpp`는 후륜축 기준 차체 내부를 0.2m 이하 간격으로 검사하고, 궤적 샘플 사이도 0.2m/0.02rad 이하 간격으로 검사한다. 격자는 도로 폴리곤 접합부의 미세한 틈을 반영하는 공식 제품이다. 이는 해상도가 있는 표본 검사이며 연속 공간의 정확한 폴리곤 포함 판정은 아니다. 실선 lane_boundary_paint와 차체의 접촉은 실제 선분으로 별도 검사하지만, 도색 접촉은 후보 탈락 조건이 아니다. `EvaluatePath`가 전체 접촉 시간과 최대 연속 접촉 시간을 계산하고 `PaintCost`를 회피 후보 비용에 더한다. 도색 접촉이 없는 경로를 우선하고, 불가피하면 짧은 접촉·회복 경로를 선택한다. 사용자가 제공한 3초 감점 기준에 따라 2.5초부터 큰 추가 비용을 적용한다. 3초 이상 접촉도 물체 충돌과 도로 영역 검사를 통과하면 완주를 위한 후보로 유지한다. 이는 예측 시간 비용이며 공식 감점 판정이나 실제 접촉 3초 미만을 보장하지 않는다. 실선에 이미 닿은 초기 자세를 이유로 모든 후보를 탈락시키지 않는다. `PoseClear`는 엄격한 도색 접촉 진단용이고 `PathClear`는 주행 가능 영역 검사다. 점선, 교차로 유도선, 정지선은 벽으로 취급하지 않는다. 정지선의 정지/감속 정책은 기존 behavior 영역이다.

## 실행 변경

`Maneuver/Avoid/MapAvoid.cpp`는 실제 초기 속도/가속도에서 시작하는 jerk 제한 속도와 거리 기반 진입·통과·복귀를 함께 검사한다. 도로가 넓어지기 전까지 횡이동을 지연할 수 있다. 복귀 거리의 최소값은 회피 오프셋에서 중앙까지의 횡이동량으로 별도 계산한다. COMMIT 이후 방향과 진입 station을 고정하고, 같은 방향의 전환거리만 재평가한다. 다른 물체의 충돌 검사와 기존 margin recovery의 실제 OBB 안전 조건은 유지한다.

UNKNOWN/DYNAMIC_PENDING 대상의 속도를 0으로 바꾸지 않는다. LOW_THREAT로 확정되기 전에는 정적 회피 COMMIT을 허용하지 않고 현재 차체 자세에서 감속 유지 경로를 만든다. 실제 동적 대상의 FOLLOW 분류 자체를 바꾸는 수정은 아니다.

MPC의 유효한 planner STOP에서는 마지막 성공 조향을 설정된 steering_rate_max로 0까지 줄이며 제동한다. 차량이 0.3m/s 미만이면 조향을 0으로 한다. 오래된 입력·잘못된 입력·긴급 정지는 기존 정책을 사용하며 조향 이력을 비운다.

## 검증

`avoid_road_constraints_test`는 기록된 탈선 궤적 차단, 미확정 대상의 사전 감속, 실제 교차로 상태의 오른쪽 회피 및 복귀를 검사한다. 별도로 고정 OBB에 대해 2초 후 정적 판정이 주어진다는 가정에서 이상적인 경로 추종을 진행해 COMMIT→회피→복귀가 STOP 공백 없이 끝나는지 검사한다. 이 추종 시험은 MORAI/MPC 폐루프 주행 검증이 아니다. `avoid_tracking_handoff_test`는 지도를 끈 기존 생성기 회귀를 유지한다. MPC의 `braking_steering_test`는 감속 중 조향 변화율, 방향 및 정지/유효하지 않은 이력 처리를 검사한다.
