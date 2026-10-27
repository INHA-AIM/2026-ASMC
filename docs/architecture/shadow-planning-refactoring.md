# 음영구간 판제 개선: 현재 연결과 다음 작업

## 현재 기준과 이번 변경

2026-10-28, PR #122 병합 main `54f6bb9a`와 실행 일치 수정 `4932b7d6`를 기준으로 한다.
3뷰 matching은 구현·기본 적용돼 있다. [3뷰 fusion](../../src/asmc_lidar/docs/three-view-fusion.md),
[좌표·시각 규약](perception-coordinate-refactoring.md),
[인지 소비 표](../handoffs/perception-to-planning.md)를 정본으로 사용한다.
과거 F1의 전방 검증·측방 투영 한계는 당시 기록이며 현재 미구현 목록이 아니다.
[이번 기반 검증 기록](../records/planning/2026-10-28-shadow-observation-foundation.md)에
빌드·시험과 private 메시지 MD5 및 배포 조건을 남겼다.

이번 단위는 다음 기반만 변경한다. 음영 회피·제어 알고리즘이나 timeout 완화는 하지 않는다.

- 같은 공용 관측의 `classification/camera_class`를 private 객체와 Behavior/Frenet 내부 객체까지 보존한다.
  숫자 type, 운동 상태, 속도 validity, 형상과 기존 행동 분기는 유지한다.
- private 배열에 session/generation/scan_sequence/config/calibration ID를 보존한다.
  별도 토픽의 최신 문자열을 뒤늦게 붙이지 않는다. ID와 header는 동일 관측에서 온다.
- planning 입력의 source-age/미래 허용 검사를 공용 정수 ns 비교로 모은다.
  유효 map만 수용하고 local을 map으로 이름만 바꾸지 않는다.
- 판제 웹에 실제 선택 제어기, 음영 정지 사유, bbox 판정, family support, 실제 선택 경로,
  local LiDAR 기하를 표시한다. 재피팅·재추론·제어 발행은 없다.
- 정본 LiDAR launch는 활성 fusion이 all이 아니거나 활성 터널이 최적화 경로가 아니면 시작을 거부한다.
  과거 비교 설정은 `namespace:=/migration/lidar`에서 명시적으로 사용한다.

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
| `/perception/lidar/tracks_local` | GPS 무효에도 제공 가능한 local 기하. **현재 음영 제어기는 이를 회피 입력으로 소비하지 않음** |
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
local OBB는 참조용이며 기존 회피기가 사용한다고 표시하지 않는다.

- source age는 ROS clock끼리, 수신 age는 monotonic끼리 계산한다. 브라우저 clock과 ROS stamp를 빼지 않는다.
- 기하 freshness350ms, 미래 허용50ms. 서로 다른 기하의 source 차이가200ms를 넘으면 겹쳐 그리지 않는다.
  이 조건을 만족해도 ego motion 정렬이나 정밀 동시 관측을 보장하지 않는다. 시각차를 화면에 표시한다.
- 지도·GPS가 무효면 마지막 map pose로 local 객체를 올리지 않는다. 잘못된 frame·역행·과거 세대는 표시 불가다.
- selector는1Hz 진단이므로 표시 freshness2.5초를 사용한다. **250ms 제어 watchdog을 늘린 것이 아니다.**
  source stamp 없는 모드/selector는 수신 age만 표시한다.
- 최신값·queue1,3개 family·경계당256점·최대256객체, 진행 중HTTP 요청1개.
  전송 제한 수량을 표시하며 geometry/JSON을 잠금 안에서 다시 계산하지 않는다.
- `debug:=false`로도 typed local 객체·family·실제 경로를 확인한다. LiDAR 단계별 점군은 별도 view/debug 책임이다.

## 패키지 흡수 판단: 가능하나 다음 단위에서 수행

`asmc_planner/src/jamming`으로 판제 책임을 모으는 방향은 타당하다.
단순 폴더 이동만으로 bbox→차로 분류 실패나 일반 MPC의 map 의존성이 해결되지는 않는다.
현재 패키지의 메시지·launch·시험·설치 경로와 planner의 실행 의존성도 함께 전환해야 한다.
기존 동작과 연결을 먼저 고정하고, 계산과 ROS 경계를 나눠 한 번에 하나의 제어 경로만 활성화한다.

| 책임 | 남길 위치·범위 |
|---|---|
| YOLO/TwinLite 추론, family, 실제 선택한 IMU 지면 context | asmc_camera |
| 점군·OBB·추적·벽/천장 근거·3뷰 연결·인지 유효성 | asmc_lidar |
| family/객체 입력 context, 통로, 행동·회피/복귀 reference와 path | asmc_planner/src/jamming (제안, 아직 생성하지 않음) |
| 궤적 추종과 차량 제한 | asmc_controller의 재사용 가능한 코어 |
| 최종 제어권·stale 차단 | 기존 ControlSelector 단일 경계 |
| 외부 인지 관측 계약 | asmc_msgs; 내부 계산과 파일 I/O를 메시지 패키지로 옮기지 않음 |

영상 BBox용 private 메시지를 그대로 공용 메시지로 승격하거나 동일 이름 wrapper를 만들지 않는다.
새 local 경로가 영상 투영 단계를 대체할지 먼저 결정한다. 전환 중 비교는 읽기 전용이고
두 제어기가 동시에 /ctrl_cmd를 발행하지 않는다. 폐기 전 옛 package API·MD5/bag 재현 revision을 남긴다.

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
각 단위는 독립 커밋·복구 설정과 함께 검증한다. stop 사유를 숨기거나 freshness를 확대해 통과시키지 않는다.
성능은 단계별 처리시간과 입력→경로→명령 지연·최대 공백으로 비교하고 평균 Hz만으로 통과하지 않는다.
현재 F1의473.47ms 공급 실패와 미검증 실제 속도·얇은 객체 보존 문제를 해결된 것으로 표시하지 않는다.
이 인지 후속은 [보류 과제 인수 단위](../../src/asmc_lidar/docs/improvement-milestones.md#보류-과제-인수-단위)에
보존한다. S1에서 인지 소실·공급 공백이 직접 원인으로 재현되면 해당 M2/R1/Q1만 우선 재개하며,
터널·복구·OBB·추적·semantic 보조 전체를 음영 판제 착수의 필수조건으로 묶지 않는다.

이번에는 새 논문/알고리즘을 선택하거나 재호의 tracker·터널 코드를 다시 작성하지 않았다.
다음 계획에서 기존 연구의 EagerMOT/Autoware 연결 경계, 현재 Frenet·충돌 검사·제어 코어를
실제 실패 fixture와 대조해 적용 범위를 하나씩 확정한다. 새로운 역할 분배는 사용자와 팀이 확인한 뒤 반영한다.
