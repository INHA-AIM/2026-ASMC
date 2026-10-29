# 음영구간 판제 개선: 현재 연결과 다음 작업

## 최신 LOCAL 실험

2026-10-29 LOCAL-2G의 정적·동적 시나리오, 여유·조향/속도와 자동 실험 웹 연결은
[LOCAL-2G 기록](../records/planning/2026-10-29-local-obstacle-campaign.md)을 따른다.
이전 [LOCAL-2F 기록](../records/planning/2026-10-29-local-braking-recovery.md)은 당시 결과다.
일반 제어와 기존 camera 기본 선택은 유지한다. 시험 소스와 profile, 성공·실패를
구분하며 과거 기록의 수치와 계약을 현재 운영 수락으로 확대하지 않는다.

## 기반 변경 (2026-10-28)

2026-10-28, PR #122 병합 main `54f6bb9a`와 실행 일치 수정 `4932b7d6`를 기준으로 한다.
3뷰 matching은 구현·기본 적용돼 있다. [3뷰 fusion](../../src/asmc_lidar/docs/three-view-fusion.md),
[좌표·시각 규약](perception-coordinate-refactoring.md),
[인지 소비 표](../handoffs/perception-to-planning.md)를 정본으로 사용한다.
과거 F1의 전방 검증·측방 투영 한계는 당시 기록이며 현재 미구현 목록이 아니다.
[이번 기반 검증 기록](../records/planning/2026-10-28-shadow-observation-foundation.md)에
빌드·시험과 private 메시지 MD5 및 배포 조건을 남겼다.

PR #124의 기반 변경은 아래와 같다. 이어진 PR #125에서는 사용자 승인으로 반복 수정 실험까지 확대했다.
[접점/local 관측 기반 실험](../records/planning/2026-10-28-shadow-contact-clearance.md)을 구분해 읽는다.
운영 기본 행동은 유지하며 새 clearance 정책은 내부 시험 설정이다. timeout 완화는 하지 않았다.
후속 [S2 회전 OBB·BEV 검사](../records/planning/2026-10-28-shadow-footprint-bev.md)는
형상 과대 확장과 고정 평면/본네트 한계를 확인했다. 당시 BEV는 audit만 허용했다.
이후 [S3 공유 IMU semantic 경로 실험](../records/planning/2026-10-28-shadow-semantic-space.md)은
별도 opt-in `semantic_trial`로 반복 검증한다. 근거리 가림 보조는 미수락이며 운영 승격하지 않았다.

- 같은 공용 관측의 `classification/camera_class`를 private 객체와 Behavior/Frenet 내부 객체까지 보존한다.
  숫자 type, 운동 상태, 속도 validity, 형상과 기존 행동 분기는 유지한다.
- private 배열에 session/generation/scan_sequence/config/calibration ID를 보존한다.
  별도 토픽의 최신 문자열을 뒤늦게 붙이지 않는다. ID와 header는 동일 관측에서 온다.
- planning 입력의 source-age/미래 허용 검사를 공용 정수 ns 비교로 모은다.
  유효 map만 수용하고 local을 map으로 이름만 바꾸지 않는다.
- 판제 웹에 실제 선택 제어기, 음영 정지 사유, bbox 판정, family support, 실제 선택 경로,
  local LiDAR 기하를 표시한다. 재피팅·재추론·제어 발행은 없다.
- 정본 LiDAR launch는 활성 fusion이 all이 아니거나 활성 터널이 최적화 경로가 아니면 시작을 거부한다.
  과거 비교 설정은 `namespace:=/test/lidar`에서 명시적으로 사용한다.

공용 `asmc_msgs` 필드·MD5는 유지한다. **private `planning_inputs/DetectedObject(Array)` MD5는 바뀐다.**
변경된 소스 전체를 재빌드하고 adapter·Behavior·Frenet·dashboard를 함께 재시작해야 한다.
실행 중 생성 메시지 파일만 교체하지 않는다. 이전 private bag은 당시 revision으로 재생하거나
명시적인 오프라인 변환을 거쳐야 한다. 이름이 같다고 새 타입으로 직접 연결할 수 없다.

## 실제 음영 실행과 정지 문제

폴더는 `src/jamming_lane_boundary_adapter`, ROS 패키지 이름은 `lane_boundary_adapter`다.
현재 autonomy launch가 family→영상 경계→bbox 차로 분류→차선변경/추종을 실행한다.
일반 MPC는 `/ctrl_cmd/normal`, 음영 추종기는 `/ctrl_cmd/jamming`을 제공하고,
**ControlSelector만 /ctrl_cmd를 발행**한다. 별도 제어 launch를 추가하지 않는다.

| 입력·상태 | 현재 의미 |
|---|---|
| `/planning/mode`, selector diagnostics | GPS 기반 모드와 실제 선택된 제어 소스. 명령 watchdog은250ms |
| `/perception/lane/boundary_family` | base_link의3경계·observed/inferred/invalid support |
| `/planning/camera_bbox_lane/result` | 전방16종 bbox의 차로 기하 분류. confidence 기본0.7; 카메라 생산자0.40과 다름 |
| `/planning/camera_bbox_lane_change/markers` | 후보 중심선과 실제 선택/차선변경 경로. 화면이 별도 경로를 계산하지 않음 |
| `/perception/lidar/tracks_local` | GPS 무효에도 제공 가능한 local 기하. **운영 기본은 미소비; S1 내부 실험에서만 동일 시각 관측 OBB를 사용** |
| `/perception/lidar/planning_observations_map` | 일반 판제 정본. GPS 무효 때 map 사용 불가를 전달하며 정상 빈 배열로 위장하지 않음 |

사용자 로그의 `map_observation_unavailable`와 일반 MPC의 `INVALID_EGO`는 GPS 무효와
일치한다. JAMMING에서 실제 정지 명령을 판단할 때는 selector와 음영 제어기 사유를 먼저 본다.
`far_obstacle_lane_unresolved`, `contact_overlap_too_small`,
`lane_change_timeout_restart_required`는 다른 원인이다.
16종 bbox는 수용하지만 기하 UNKNOWN이 남으면 결과를 무효 처리하므로, plastic_barrier/drum
사례의 정지를 “static 그룹을 일부러 무시한다”로 단정하지 않는다.
source age 약0.1–0.19초인 로그만으로 stale 또는3뷰 계산 지연을 원인으로 확정할 수도 없다.

이 작업은 그 정지를 해결한 주행 수락이 아니다. RANSAC 메시지도 호출 단계·stamp와 연결한
추가 근거 없이는 지면/벽 실패 및 제어 정지 원인으로 확정하지 않는다.

## 콘솔의 시각·좌표·성능 경계

`roslaunch asmc_planner dashboard.launch` → `http://127.0.0.1:8088/web/index_v2.html`.
자동/음영 local/일반 map을 선택할 수 있다. local 화면은 X 전방=위, Y 좌측=왼쪽이다.
family의 실제 관측은 실선, inferred는 점선, invalid support는 끊는다.
운영 기본의 local OBB는 참조용이다. S1 실험에서는 diagnostics의 contact_clearance/local 입력 상태로
실제 사용 여부를 구분하며, 시각화만 보고 제어 입력 사용을 추정하지 않는다.

- source age는 ROS clock끼리, 수신 age는 monotonic끼리 계산한다. 브라우저 clock과 ROS stamp를 빼지 않는다.
- 기하 freshness350ms, 미래 허용50ms. 서로 다른 기하의 source 차이가200ms를 넘으면 겹쳐 그리지 않는다.
  이 조건을 만족해도 ego motion 정렬이나 정밀 동시 관측을 보장하지 않는다. 시각차를 화면에 표시한다.
- 지도·GPS가 무효면 마지막 map pose로 local 객체를 올리지 않는다. 잘못된 frame·역행·과거 세대는 표시 불가다.
- selector는1Hz 진단이므로 표시 freshness2.5초를 사용한다. **250ms 제어 watchdog을 늘린 것이 아니다.**
  source stamp 없는 모드/selector는 수신 age만 표시한다.
- 최신값·queue1,3개 family·경계당256점·최대256객체, 진행 중HTTP 요청1개.
  전송 제한 수량을 표시하며 geometry/JSON을 잠금 안에서 다시 계산하지 않는다.
- `debug:=false`로도 typed local 객체·family·실제 경로를 확인한다. LiDAR 단계별 점군은 별도 view/debug 책임이다.

## 구 jamming 패키지와 새 local의 경계

이전의 `asmc_planner/src/jamming` 흡수안은 직접 실행하지 않는다. LOCAL-1은
`asmc_planner/src/local`과 `asmc_controller/src/local`에 독립 입력·실행 경계를
구현했다. 기존 BBox 차로 선택 알고리즘을 새 local의 내부 단계로 호출하지 않는다.
구 패키지는 기존 autonomy의 운영 경로와 이강태의 별도 개선을 위해 유지한다.
단순 폴더 이동으로 합치거나 기존 map 상태의 GPS 유효성 검사를 풀지 않는다.

| 책임 | 남길 위치·범위 |
|---|---|
| YOLO/TwinLite 추론, family, 실제 선택한 IMU 지면 context | asmc_camera |
| 점군·OBB·추적·벽/천장 근거·3뷰 연결·인지 유효성 | asmc_lidar |
| family/객체 입력 context, 통로, 회피/복귀 reference와 path | asmc_planner/src/local (LOCAL-1 구현; 전체 주행 수락 전) |
| 궤적 추종과 차량 제한 | asmc_controller/src/local 입력 경계와 재사용 MPC 코어 |
| 최종 제어권·stale 차단 | 기존 ControlSelector 단일 경계 |
| 외부 인지 관측 계약 | asmc_msgs; 내부 계산과 파일 I/O를 메시지 패키지로 옮기지 않음 |

영상 BBox용 private 메시지를 그대로 공용 메시지로 승격하거나 동일 이름 wrapper를 만들지 않는다.
새 local 경로가 영상 투영 단계를 대체할지 먼저 결정한다. 전환 중 비교는 읽기 전용이고
두 제어기가 동시에 /ctrl_cmd를 발행하지 않는다. 폐기 전 옛 package API·MD5/bag 재현 revision을 남긴다.

## LOCAL 실행 단일성과 기준 경로·자유공간 설계

2026-10-28의 실제 master 조회에서는 `/local_planner → /planning/local/trajectory →
/local_controller`만 연결돼 있었다. `enable_control=false`, `/ctrl_cmd` 발행자0,
`/ctrl_cmd/normal`과 `/ctrl_cmd/jamming` 발행자0인 **preview 상태**다. 기존 map
planner/controller·BBox lane classifier/follower·selector는 실행돼 있지 않았다.
이는 조회 순간의 실행 근거이며 모든 사용자의 launch 조합이 배타적이라는 뜻은 아니다.

| 경계 | 현재 실행·재사용과 제한 |
|---|---|
| `local.launch` | 기본은 planner/controller preview. `semantic_input`으로 노면 투영 입력 선택 가능. 구 BBox·센서 추론·selector는 시작하지 않음 |
| 계산 코어 | 기존 Frenet quintic·충돌 기하·속도 수학과 MPC solver를 링크해서 호출. 기존 ROS planner·행동 판단 루프·BBox 차선변경 루프는 호출하지 않음 |
| 제어 시험 | `run_local_trial.py`가 기존 제어/모드 발행자를 먼저 거부하고 selector 하나와 local만 시작 |
| `/ctrl_cmd/jamming` | 기존 selector의 선택 슬롯을 재사용한 이름. 새 local 제어를 연결할 때 구 follower도 가동한다는 뜻이 아님 |
| 실행 중 충돌 | 독립 시험은 NORMAL 생산자도 거부. 통합 실행만 지정된 NORMAL MPC와 selector를 허용하며, 같은 JAMMING 슬롯의 다른 생산자는 계속 거부. 1초 주기 검사이며 원자적 lease나 즉시 차단은 아님 |
| dashboard | 구 BBox 결과와 새 local 결과를 읽어 표시. 구 제어 알고리즘·경로 계산을 실행하지 않음 |

**독립 `local.launch`와 autonomy를 따로 겹쳐 실행하지 않는다.**
2026-10-29부터 `autonomy_no_bridges_v2.launch jamming_planner:=local`이 LOCAL을 한 번
include하고 구 BBox 경로를 제외한다. 기본 `camera`는 이전 행동을 유지한다.
통합 LOCAL MPC는 NORMAL 중 계산하지 않는다. 속도/IMU 이력을 유지하고 기존 음영
진입30m 전부터 경로만 준비한다. 같은 센서 session·설정·local frame의 준비 입력과
경로는 진입 때 유지한다. 모드 변경은 계산 중 이전 모드 결과만 차단하고, 센서 세대
변경·재시작은 입력·노면 기억·경로도 폐기한다. 이 둘을 같은 초기화로 취급하지 않는다.
통합 이름으로 허용된 `/mpc_controller` 외의 NORMAL 생산자, 다른 JAMMING 생산자,
지정 selector 외 wire 생산자를 허용하지 않는다. 기존 selector의250ms watchdog은 유지한다.
노면 투영 입력은 `asmc_camera/launch/semantic_space.launch`가 소유하며 시나리오 launch도
이를 include한다. 추가 추론이나 구 제어 루프를 실행하지 않는다.
실행·공유 master 설정은 [명령 안내](../operations/commands.md#공유-master에서-local-음영-주행-선택)를 따른다.

사용자가 확정한 새 local의 탐색 규약은 다음과 같다.

1. **출발 상태는 실제 후륜축**, 우측 본차로 중심은 복귀용 기준 경로다. 현재 기하
   role2(중앙선)와 role3(우측 흰선)의 중간을 사용한다. 중앙선 근처로 이동했다고
   본차로를 다시 선택하지 않는다. 본차로 편차는 비용이며 충돌 회피를 막는 강제 제약이 아니다.
2. 무장애에서는 본차로 reference와 이전 경로 주변에 후보를 우선 배치한다. 장애물이
   있으면 연결 가능한 주행가능 공간별 중심·가까운 reference 지점에 목표를 추가한다.
   서로 떨어진 양쪽 공간을 합쳐 무게중심 하나를 만들지 않는다. 각 목표까지 차체 전체의
   연속 swept footprint·조향·곡률·속도 유효성을 검사한다.
3. **3개/5개 추종선 중 하나를 선택하는 정책을 만들지 않는다.** 현재 목표 x=6/9/12m는
   세 개의 전방 단면이며 차선 번호가 아니다. 횡방향은 연속 곡선으로 탐색한다. family의
   경계·차로 중심은 방향과 복귀의 보조 근거다. 외곽 흰선을 넘어도 검증된 노면·벽·객체와
   차체 충돌이 없으면 허용하며 차선 침범 비용을 추가하지 않는다.
4. 노면 확률·coverage, 물리 통로, 객체 충돌을 분리한다. 노면의 검은 영역에는 장애물과
   뒤쪽 가림이 섞인다. LiDAR가 빈 배열이거나 통로폭을 알고 있다는 이유로 구멍을 전부
   자유공간으로 채우지 않는다. 노면이 불안정할 때 family는 진행 방향을 보완한다.

이는 reference와 drivable area를 나누고 후보 생성→충돌/곡률 검사→reference 이탈 등의
비용 평가로 분리하는 [Autoware Path Sampler 공식 구조](https://autowarefoundation.github.io/autoware_universe/main/planning/sampling_based_planner/autoware_path_sampler/)를
현재 입력에 맞춰 해석한 설계다. 외부 ROS2 노드나 해당 구현의 정지 정책을 그대로
도입했다는 뜻은 아니다. 현재 우리 속도·근거리 회복 정책의 제한은 LOCAL-1 결과에 유지한다.

### 거의 일정한 터널폭의 활용

외곽 흰선과 실제 터널 벽/통과 가능한 노면 끝은 다른 경계다. 좌우 각각의 여유를 실제
센서와 시뮬 장면으로 측정하고 곡선·입출구·벽 소실·오차를 검수한 뒤 통로 사전값으로
사용한다. 차선폭을 단순히 두 배 해서 벽 위치로 고정하거나 흰선 밖을 미리 차단하지 않는다.
[LOCAL-2A 측정](../records/planning/2026-10-28-local-input-alignment.md#흰선과-실제-수직-벽-사이-여유)에서
겹치는 두35m 구간의 흰선–수직 벽 간격을 기록했다. 첫 구간 중앙값은 좌1.42m/우2.20m,
25m 앞에서 시작한 구간은 좌1.87m/우2.13m였다. 원본 RGB에 재투영해 확인했지만
터널 전체·독립 충돌 mesh·통과 가능한 노면의 수락은 아니다. 좌측 간격이 구간별로 달라
고정 대칭 통로폭을 만들지 않았으며, 점 지지·시각·거리별 측정과 불가 사유를 보존했다.
턱·비노면·장애물을 확인하기 전에는 이 폭으로 semantic 구멍을 자유공간으로 채우지 않는다.

연산은 한 번 만든 scene·grid index를 재사용하고, 물리 통로로 명백히 불가능한 후보를
먼저 줄인 뒤 필요한 차체 footprint에서 노면·OBB를 상세 검사하는 방향이 적절하다.
현재40,000셀 grid를 쓴다는 사실만으로 전체 mask가 병목이라고 결론내리지 않는다.
벽 검출/RANSAC은 LiDAR가 소유하고 planner에 두 번째 벽 인식기를 만들지 않는다.

### LOCAL-2B: 우측 reference와 실제 검증 공간 분리

[LOCAL-2B 경로 안정화 결과](../records/planning/2026-10-28-local-path-stability.md)에
수정 전 실패부터 반복시험까지 연결했다. 입력은 우측 본차로의 role2/3 support를 사용하고,
사용하지 않는 왼쪽 흰선 support 때문에 전체 경로를 자르지 않는다. 그 끝은 후륜축의
참조 경로이며 차량 앞부분 길이를 다시 빼지 않는다. 실제 진전 가능 거리는 전체 차체의
semantic grid·OBB sweep으로 따로 검사한다. 전체3경계를3/5개 추종선으로 선택하지 않는다.

- raster index를 한 번 만들고 strict/근거리 보조 후보에서 재사용한다.
- 이전 경로의 남은 부분도 현 시각으로 정렬하여 다시 검사하고, 비슷한 비용이면 유지한다.
- 현재 관측되지 않는 근거리에는 과거 **실제로 관측된 노면**만 최대5초/6m로 정렬해
  보조한다. 최신 비노면·LiDAR 객체가 우선하며 source 시각을 갱신하지 않는다.
- 물리 차체+최소10cm+격자 오차는 hard 제한이다. 그 바깥 속도별 semantic 여유는
  선호 비용으로 분리한다. LiDAR 객체 충돌 제한은 유지한다.
- 속도 계획의 양의 가속·jerk 제동거리를 포함하고 가속/제동마다 상한이 왕복하지 않게
  한다. 실험 local 가속 상한1m/s²이며 NORMAL actuator 변환과 정책은 유지한다.
- 350ms source freshness를 제어 시점까지 지키기 위해250ms를 넘은 묶음으로 새 계산을
  시작하지 않고 최신 완료 묶음을 기다린다. 이전 명령을 새 timestamp로 발행하지 않는다.

[LOCAL-2A 결과](../records/planning/2026-10-28-local-input-alignment.md)의 signed 상태·독립
입력 선택·한 번의 상대 정렬·callback 분리는 유지한다. 빈 터널에서 측정한 LOCAL-2B
범위를 장애물 회피·전체 터널·25km/h 수락으로 확대하지 않는다. 다음 작은 단위는
가감속 시 노면 잔여점의 긴 OBB와 실제 장애물을 분리하는 검증이다. 측정된 벽 간격으로
semantic 구멍을 일괄 채우거나 인지 오류를 planner의 무조건 통과로 숨기지 않는다.

### LOCAL-2C: 속도 연속성과 노면 OBB 원인 대조

[LOCAL-2C 결과](../records/planning/2026-10-28-local-speed-continuity.md)에서10·15km/h
추종과 노면 OBB 원인을 분리했다. 사용자의 최신 결정으로25km/h 달성을 완료 조건으로
요구하지 않는다. LOCAL 궤적은 scalar cap 대신 기존 S-curve의 물리 끝점 상태를
재사용하며 NORMAL 속도 정책은 유지한다.200.9ms 계획 교체와67µs 미래 상태의
순간 급제동은 발행 없는 bounded 대기로 보완했다. 미래/만료 입력으로 새 계산을 하지 않는다.

문제 scan의262점은 기존 추정 평면 아래에 남은 노면 군집이었다. 지면 fitting만0.10m로
분리한 후보는 이를 제거했지만 다른 벽면 잔류점·계산 비용이 늘었다. 실제 barrier117scan
대조도 남겼으며 운영 fitting 기본값은 유지한다. 길이/나이/낮은 높이만으로 객체를 버리는
planner 필터는 추가하지 않았다. 반복시험의 노면 OBB 재발과 입력 지연 정지는 아직
미해결로 표시한다. 다음 우선 단위는 **지면 후보의 실제 장애물 보존·벽 혼입 검증**이다.
IMU localization 별도 worktree와는 같은 조건의 중간 평가 후 집중할 방향을 정한다.

## 두 개선 방향의 비교

### 1. IMU·family로 터널 위치를 추정하고 HD맵 사용

IMU만의 위치 이중 적분은 바이어스·중력 오차가 누적된다. 검증된 속도+IMU로 짧은 상대 이동을
구할 수는 있지만, GPS 유효 시 마지막 상태를 계속 정상 위치로 쓰는 방식과 구별해야 한다.
평행한 차선·벽은 횡방향 위치와 방향의 제약을 제공하나 긴 직선 터널의 종방향 위치를
유일하게 정해주지 않는다. family가 있다고 절대 map 위치가 자동 복원되는 것은 아니다.

별도 상대 pose를 도입한다면 진입 anchor, clock, reset/session, 누적 불확실성, 허용 horizon,
GPS 복귀의 jump/재확보 계약이 필요하다. 검증 전 이 pose로 HD맵 제외를 재활성화하지 않는다.
기존 정적/동적 미션의 행동 수학은 재사용하되 map 링크·TTC·절대속도 유효성 가정을 분리해야 한다.
현재 정지 문제의 첫 해결을 이 위치추정 완성에 종속시키지 않는다.

### 2. 위치추정 없이 local 통로·장애물로 경로 계획 — 우선 후보

family의 유효 support 내 reference와 실제 local OBB를 공통 기준 시각으로 받아,
충돌 검사·회피 곡선·속도 제한의 기존 코어를 재사용하는 방향을 먼저 대조한다.
현재 일반 Frenet/MPC는 map 차량 상태를 요구한다. 토픽 remap이나 base_link→map 이름 변경으로
재사용하지 않는다. 입력 frame/time context를 명시적으로 받는 코어 경계를 만들고,
계획 중 고정 기준계와 다음 scan의 기준계 변화를 검증해야 한다.

- family의15–20m는 모든 프레임의 확정 가용 길이가 아니다. valid_x/support 범위 내에서 정지거리,
  회피 출발·복귀·차량 폭과 곡률을 검사한다. 부족하면 멀리 외삽해 가용 도로를 만들지 않는다.
- TwinLite drivable은 관측 근거다. 가림·누락·장애물 경계 오차가 있으므로 HD맵의 확정 자유공간과
  동등하게 취급하지 않는다. 벽 통로·차선·객체 기하와 각각의 유효성을 함께 사용한다.
- 바깥 흰선 밖의 여유 공간과 그곳에 걸친 장애물을 보존한다. 흰선을 넘는 점을 모두 삭제하지 않는다.
  LiDAR 벽면의 유한 지지·높이·끊김을 통로 경계의 근거로 쓰고, 벽 분류를 판제에 중복 구현하지 않는다.
- 차량/person도 정지해 있을 수 있다. static 라벨도 운동의 보증이 아니다. GPS 무효 절대속도는
  UNKNOWN을 유지하고, 충돌 기하를 버리거나0속도의 확정 측정으로 만들지 않는다.
- 벽 끝은 터널 출구 후보이지 단독 복귀 trigger가 아니다. 시야 제한·가림·검출 중단과 구분하고,
  복귀 경로의 현재 가용성·차선 역할·GPS 회복 및 전환 조건을 함께 확인한다.
  출구 직전 급복귀 대신 관측 horizon 안에서 복귀 가능성을 미리 평가한다.

## 다음 작업 순서와 수락 근거

| 단위 | 산출물 | 완료/중단 기준 |
|---|---|---|
| S1 정지 재현·소비 계약 | drum/plastic/차량/person의 같은 입력 replay, 선택 제어·bbox·family·local 객체 대응 | 기하 UNKNOWN·진행시간 timeout·입력 공백·제어권 문제를 분리. 정지를 무조건 무시하는 수정 금지 |
| S2 구조 정리 | planner/jamming 계산 경계, 기존 입력·출력 시험, 실행 단일화 | 파일 이동과 알고리즘 변경을 분리. 기존 출력 동등성과 단일 /ctrl_cmd 확인 |
| S3 local 통로·객체 계획 대조 | family reference+local OBB로 기존 회피/충돌 코어 사용, bbox 방식과 동일 입력 비교 | 실제 장애물 보존·지지 밖 경로 금지·복귀 가능성·GPS0/복귀·오래된 context 배제 |
| S4 제어 연결·짧은 폐루프 | 현 제어기 재사용 또는 명시적 local 어댑터, selector 전환 회귀 | 중복 발행0, 실제 진척/충돌 여유·정지 및 재개, 처리 지연/공백과 사용자 주행 확인 |

S1에서 필요한 최소 기록만 새로 수집한다. 이전 공간 예외를 재사용하지 않는다.
2026-10-28에는 사용자가 **이번 S1의 50GiB 미만 D 기록을 별도로 허용**했다.
원태의 `scen_runner`에 등록 run 녹화·정상/실패 장면 저장·정확한 stamp 갤러리를 연결하고,
터널 6회(주행 허용창 합계 약365초)를 기록했다. [시험 결과](../records/planning/2026-10-28-shadow-scenario-reproduction.md),
[반복·녹화 명령](../../tools/scen_runner/docs/shadow-recording.md)을 따른다.
첫6회의 상태는 **재현 기반 구현·첫 실패 자료 확보**다. 이후 사용자가4시간 수정/주행 실험을 승인했고,
[후속 실험 기록](../records/planning/2026-10-28-shadow-contact-clearance.md)에 실제 변경·실패·수락 범위를 이어서 기록한다. bbox/family 소비 결함 수정과
음영 주행 전체 수락은 아직 완료가 아니다. `far_obstacle_lane_unresolved` 지속 정지와
플라스틱 방호벽의 실제 구간 종료 사례를 모두 보존해 클래스만으로 원인을 단정하지 않는다.
각 단위는 독립 커밋·복구 설정과 함께 검증한다. stop 사유를 숨기거나 freshness를 확대해 통과시키지 않는다.
성능은 단계별 처리시간과 입력→경로→명령 지연·최대 공백으로 비교하고 평균 Hz만으로 통과하지 않는다.
현재 F1의473.47ms 공급 실패와 미검증 실제 속도·얇은 객체 보존 문제를 해결된 것으로 표시하지 않는다.
이 인지 후속은 [보류 과제 인수 단위](../../src/asmc_lidar/docs/improvement-milestones.md#보류-과제-인수-단위)에
보존한다. S1에서 인지 소실·공급 공백이 직접 원인으로 재현되면 해당 M2/R1/Q1만 우선 재개하며,
터널·복구·OBB·추적·semantic 보조 전체를 음영 판제 착수의 필수조건으로 묶지 않는다.

재호의 tracker·터널 코드는 다시 작성하지 않았다. S1의 제한된 clearance 후보와 참고 논문은 후속 실험 기록에 둔다.
다음 계획에서 기존 연구의 EagerMOT/Autoware 연결 경계, 현재 Frenet·충돌 검사·제어 코어를
실제 실패 fixture와 대조해 적용 범위를 하나씩 확정한다. 새로운 역할 분배는 사용자와 팀이 확인한 뒤 반영한다.


## S1 반복 수정 후 우선순위

4시간 실험에서 source 대응·연속 support·고정15m 끝점·짧은 후보의 추종점 문제를 각각 재현하고
내부 시험 경로에서 수정했다. Drum 분리 장면의 반복 구간 종료와 concrete/SUNNY 입구 plastic의
지속 정지를 함께 보존한다. 결과와 실패를 합쳐 "음영 정지 해결"로 선언하거나 운영 기본으로 올리지 않는다.

다음 단위는 S2/S3 경계 중 **일관된 local 계획 입력과 진입 준비**를 먼저 구체화한다.

1. 음영 제어로 전환될 때 이미 약8m 앞인 장애물 사례를 고정한다. 유효한 GPS 구간에서 local 계획을
   미리 준비할 방법과 실제 제어권 전환을 분리한다. GPS 무효를 위장하거나 selector watchdog을 늘리지 않는다.
2. family/영상/local OBB를 하나의 시각 context로 받고, 실제 연속 support에서 차체 전체를 검사한다.
   카메라의 접점 사각형과 측정 OBB, 미관측 깊이를 구분한다. 원거리 좌우 범위를 근거리 AABB로 확대하지 않는다.
3. 기존 `scen_runner` 준비 검사는 map state/trajectory를 요구하므로 GPS0에서 시작하는 r30은 준비 실패다.
   음영 내부 시작을 지원하려면 실제 JAMMING 선택·fresh local 기하·CAN/IMU·정상 정지 상태를 검증하는
   별도 준비 계약이 필요하다. 현재 검사를 무시한 출발이나 가짜 map pose로 우회하지 않았다.
4. 가까운 plastic의 같은 관측에서 발행 local에 전방 물체가 없는 사례를 인지 담당에게 전달한다.
   이미 제공된 OBB는 사용하지만 미수신·빈 배열을 장애물 없음으로 취급하지 않는다. 이 사례의 원인 확인만
   우선하고, 얇은 객체·tracker·터널 알고리즘 전체를 다시 작성하는 작업으로 확대하지 않는다.
5. r45는 물체를 더 안쪽에 놓고 실제 local plastic을 받아도 변경 중 정지했다. 따라서 진입 준비나
   인지 누락 하나만으로 종결하지 않는다. `turn-blocked-*` fixture로 경로 끝점·차체 swept footprint·
   재계획 시 초기 조건과 실제 추종을 함께 비교한다.15m 완결 후보는 이 fixture의 검사를 통과하지만
   r48 실주행에서는 더 진행한 뒤 다시 정지했다. 단순 차체 여유 축소나 한 프레임의 통과로 종결하지 않는다.
6. r40/r43에서 약25%의 bbox frame에는 동일 원본 front projection이 없었다. 공통 시각 context의
   생산/선택과 경로 계산을 구분해 개선한다. 모델 각각의15Hz를 동일 입력 묶음15Hz로 해석하지 않는다.

S1의 추가 여유·조기 차로 변경 선택 후보는 concrete 통과를 개선하지 못해 실행 경로에서 제거했다.
실패를 해결하려고 차체 여유·도로 경계·freshness를 낮추지 않았다. 세부 결과와 재현 SHA는
[수정 실험 기록](../records/planning/2026-10-28-shadow-contact-clearance.md)에 연결한다.


## LOCAL-1 이후 입력·속도·DR 검토

[2026-10-28 시간/속도 정책과 AIMDrive DR pilot](../records/planning/2026-10-28-local-timing-and-dr-pilot.md)에
원인과 LOCAL-2A~2D 작업 순서를 보존한다. 후속 [LOCAL-2A 구현·실험](../records/planning/2026-10-28-local-input-alignment.md)은
후진 시 관측 소실·카메라 계산용 추가10Hz 제한·50ms local join과 callback 차단을
정리했다.3km/h 가림 회복·20m 관측/제동 상한은 남는다. 무장애25km/h는2B,
인지 노이즈 응답은2C, 정적 회피는2D의 별도 수락 단위다. AIMDrive 전체 이식 또는
GPS 무효 검사의 제거가 아니다.

## 2026-10-29 LOCAL-2D 착수 기반

[main 리베이스·전용 실행 경계](../records/planning/2026-10-29-local-rebase.md)를 따른다.
PR127 기존 음영 방식과 독립 LOCAL 제어를 동시에 실행하지 않는다.

LOCAL-2D는 리베이스·클린 빌드·NORMAL/LOCAL 경계 검증을 완료했다. 실제 시각의 MPC
이력, fitting0.10m, 공통 후보 비용은 독립 worktree에 구현됐으며
[오프라인 결과와 남은 수락](../records/planning/2026-10-29-local-rebase.md#local-2d-오프라인-구현검증)을 따른다.
그 이후 공급·소유권·시각·인계 보완과 실제 반복 시험은
[LOCAL-2D 연속성 결과](../records/planning/2026-10-29-local-continuity.md)가 정본이다.
실험 profile은 통합 전환·좌/우 장애물 각3회 접촉 없이 통과했지만 저속 복구, 일부
본차로 오차,250ms 초과 유효 경로 공백과 진입 full brake2회가 있어 **전체 미수락**이다.
기본 camera와350ms freshness를 유지하고 노면 reference 후보는 기본 OFF로 둔다.
다음은 준비 경로의 시간 여유·첫 계산 위상과 reference 전환을 다루는 LOCAL-2E다.
이전25km/h 목표를 반복하거나 보호 동작을 제거해 성공으로 표시하지 않는다.

## LOCAL-2E 반복 회피·전환 보완

[LOCAL-2E 결과와 실행 profile](../records/planning/2026-10-29-local-avoidance-continuity.md)에
모드 즉시 첫 계산, source-aged 방향 선호, 짧은/0진행 prefix 재탐색과 공정한 측방 후보
계산 순서를 기록한다. 일반 판제·차체 외곽·신선도는 유지하며 별도 localization 연구를
병합하지 않는다.10km/h 배치와12km/h 확장 시험을 구분하고, 복구/미관측 보조와
남은 terminal braking 실패를 정규 회피 수락으로 표시하지 않는다.


## LOCAL-2H 1차 비교 체크포인트

[10월29일 연속성·검증 경로·실제 실패](../records/planning/2026-10-29-local-first-comparison.md)를
최신 결과로 따른다. family 일시 누락은 현재 노면으로 이어가고, 긴 reference와 검증 후 짧은
경로를 구분한다. FOGGY 여러 배치의 거리 도달과 남은 짧은 정지·이격·공급 실패를 함께 보존한다.
6m 복구 목표는 이득이 없어 기본4m를 유지한다. 새 경로 유지/단계적5개 기준선은 내부 비교
정책이며 기본OFF다. 최신 main 반영·배포·운영 승격은 별도 요청으로 진행한다.
