# S2 — 회전 OBB 보존과 주행가능 BEV 경로 검사

2026-10-28, `08d8fec6` 이후 `feat/shadow-local-footprint`의 2시간 작업이다.
후속 [S3 공유 IMU semantic 경로 실험](2026-10-28-shadow-semantic-space.md)에서 제어 적용 재확인과 새 후보를 대조한다. 아래는 S2 당시의 근거다.
선행 [S1 접점/local 실험](2026-10-28-shadow-contact-clearance.md)은 당시 결과로 보존한다.
운영 기본은 기존 정책이며, S1 clearance와 S2 BEV는 기본 승격하지 않았다.

## 구현과 확인된 원인

`LocalObstacleSupport`가 LiDAR의 8코너를 XY AABB로 만들고, follower가 다시 AABB로
확장하던 경로를 제거했다. 8코너의 XY convex hull에 기존 축별 0.25m padding을
Minkowski 합으로 적용한다. 기울어진 상자의 footprint는 4개보다 많은 꼭짓점을
가질 수 있다. private `geometry_msgs/Polygon` 배열로 이 형상을 보존하고,
차체 회전 footprint와 separating-axis 검사를 수행한다. 메시지 정의/MD5는 같다.

- r45의 동일 scan768·track94·24지지점에서 무패딩 hull 면적은 0.352924m²,
  AABB는 0.888240m²로 2.517배였다. 이는 제공된 OBB의 표현 차이이며 독립 GT가 아니다.
- 모든 local 관측 객체를 계속 검사한다. YOLO matching에 성공한 객체만 남기지 않는다.
- Point32 직렬화의 미세한 거의 일직선인 모서리는 측정 footprint에서만 convex hull로
  정규화한다. 영상 투영 사각형의 잘못된 ray 순서는 계속 거부한다.
- 차체 크기·횡/종 여유·관측 지지 범위·source/receive350ms·시각차50ms는 유지한다.
- 같은 r45 입력에 구 AABB/새 hull을 넣은 1,000회 대조는 둘 다 148번째 후보를
  선택했다. p95는 각각1.178/1.261ms였다. 이 장면의 폐루프 정지를 해결했다는 뜻이 아니다.

## BEV 시도와 적용 보류

`DrivableClearance`는 grid의 각 행에 비주행/미관측 prefix sum을 만들고,
후보 경로의 회전 차체·안전 여유와 교차하는 셀을 검사한다. 중심선만 검사하지 않는다.
현재 차체가 이미 점유한 영역과 새로 쓸 영역을 구분한다. 이 영역에서도 LiDAR와
영상의 양성 장애물 검사는 별도로 유지한다. 현재 prefix 초기 영역 처리는 검증 후보다.
확률 임계값166/255는 고정한 실험값이며 운영 수락값이 아니다.

현재 실행에서 확인한 제약은 다음과 같다.

1. `/perception/bev/camera/drivable_probability`와 `coverage`를 만드는 기존 raster
   경로는 고정 평면이다. RGB comparison 시각화의 공유 IMU 경로와 같지 않다.
2. 본네트의 비주행 semantic이 지면에 투영돼 전방 약4–6m에 가짜 구멍을 만든다.
   그 영역을 free로 덮으면 실제 가까운 장애물까지 숨길 수 있다.
3. BEV에는 장애물 실루엣·가림 뒤 영역이 지면 점유로 늘어나는 문제가 있다.
   모든 낮은 확률을 정확한 OBB footprint로 해석할 수 없다.

따라서 실행은 `bev_mode=off/audit`만 허용한다. 미수락 `guard`는 시작을 거부한다.
`audit`은 source/receipt350ms, exact probability/coverage stamp, image/family와50ms,
카메라 session/generation/cutoff를 검사한 뒤 **선택된 기존 경로**의 비교 결과만 남긴다.
현재 고정 평면 비교를 IMU 정합된 충돌 판정으로 해석하지 않는다.
새 모델 추론·새 제어기·새 운영 발행 계약은 추가하지 않았다.

다음 BEV 단위는 기존 `ComparisonProjection`/정본 보정과 family의 정확한
`CameraGroundProjection`을 재사용해 **semantic 전용 공유 평면** 출력을 만들고,
원본 영상의 본네트/차체 mask를 관측 불가로 전달하는 작업이다. 이후에만
LiDAR 근거리 기하 + 관측된 semantic free-space + family 도로 경계의 조합을 검증한다.
알려진 LiDAR 장애물을 높은 주행가능 확률로 제거하지 않는다.

설계 참고는 [Autoware path optimizer](https://autowarefoundation.github.io/autoware_universe/main/planning/autoware_path_optimizer/)의
차체 footprint·drivable area·기하 실행 가능성 분리와
[goal planner의 grid 충돌 검사](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_path_planner/autoware_behavior_path_goal_planner_module/)다.
Autoware 구현 전체나 성능 수치를 이식/재현한 것은 아니다.

## 반복 시험과 제어 전달 한계

D의 새 run `shadow-footprint-s2-20261028-r01`–`r06`과
`shadow-footprint-s2-20261028-audit01`에 설정·소스·재배치·복구 근거를 보존했다.
기존 run은 덮어쓰지 않았다. 사용자의 이번 실험 저장 예외를 기록했고 저장 정책은 수정하지 않았다.

| run | 조건/결과 | 해석 |
|---|---|---|
| r01/r02 | FOGGY13 plastic; 준비 실패 | 시뮬 전체 ESC pause로 IMU가 이전 자세를 계속 송신. 허용 yaw 확대 없이 중단 |
| r03 | 유효 GPS 접근 위치,180초 | 일반 구간 저속 접근 뒤 own-lane support 실패; 통과 미완료 |
| r04 | 더 가까운 시작 위치 | GPS/상태가 무효여서 기존 map 준비 실패. 정상 출발 성공으로 계산하지 않음 |
| r05 | 유효 GPS의 마지막 접근 위치,180초 | 통과 미완료. BEV는 audit |
| r06 | 실제 GPS0 local 준비,180초 | local 준비 성공. 정지/일반 구간 역진입·재진입, 통과 미완료 |

r06의 local planner 3,456회 계산은 p95 0.772ms·p99 1.072ms·max2.979ms였다.
`contact_all_paths_blocked`, `projection_source_expired`, `no_continuous_own_lane_to_lookahead`
실패는 남았다. 드문 `local_obstacle_geometry_invalid`는 직렬화 후 hull 정규화의
후속 회귀 대상이며, 수정 후 동일 폐루프 수락을 선언하지 않는다.

이번 반복에서 양의 전진 명령에도 실제 차량은 뒤로 밀리는 구간이 있었다.
Windows localhost9093으로 직접 brake1/steer0.2를 보낸 23/27바이트 본문 대조에서도,
첫 송신 확인 후 조회한 MORAI 적용 throttle/brake/steer는0이었다. P/정차로 복구했다.
`GetVehicleNetworkSetting`은 ROS 설정을 반환했다. 이것만으로 실행 중 UDP 선택을
확정할 수 없으므로, 네트워크 선택/제어 수신의 원인은 미확정이다.
현재 센서 수신, 제어 송신 횟수, 실제 제어 적용을 같은 성공으로 계산하지 않는다.
이 주행들을 회피 알고리즘의 폐루프 수락 자료로 사용하지 않는다.

## 시험 도구 보완과 검증

시나리오의 `readiness_profile: shadow_local`은 명시적 시험 설정이다. 기본은 `map`이다.
네이티브 시뮬 pose/정차 확인은 기존 runner가 담당한다. 새 프로필은 실제 GPS0,
JAMMING 모드, cutoff 이후 fresh family/local 관측, 정차 CAN 속도와 유효한5km/h 이하
음영 제어 명령이 모두 있어야 출발한다. map pose를 만들거나 GPS를 주입하지 않는다.

분석기는 빈 JPEG를 가진 invalid 진단 frame 때문에 전체 분석이 중단되던 문제를
수정했다. 해당 frame은 `gallery_errors`로 집계하고 나머지 판단/센서 기록은 분석한다.
r06 최초 실패 분석은 보존하고 `analysis-v2`에서 재분석했다. 빈 payload2건을 별도 표시했다.

검사 결과와 갤러리 정본은 audit run의 `validation.json`, `index.html`을 참조한다.
운영 승격·장기 공급 수락·충돌 없는 통과를 완료로 표시하지 않는다.

## 실행과 다음 한 단위

일반 운영 명령은 유지한다. 시험 supervisor에만 다음 환경을 지정한다.

```bash
ASMC_SHADOW_CONTACT_CLEARANCE=true ASMC_SHADOW_LOCAL_CONTACT=true \
ASMC_SHADOW_APPROACH_CAP=true ASMC_SHADOW_BEV_MODE=audit \
./tools/scen_runner/run_planning.sh
```

원상복구는 위 환경 없이 supervisor를 새로 시작한다. 이미 실행 중인 사용자 노드는 종료하지 않는다.
local 시작 시험은 등록한 run의 `stack-config.yaml`에만 `readiness_profile: shadow_local`을 둔다.

우선순위는 **실제 제어 적용 확인 → 같은 local 시작점에서 hull 수정 재대조 →
공유 IMU·가시성 mask를 갖춘 semantic BEV → 후보 경로/회피·복귀 검증**이다.
