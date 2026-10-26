# LiDAR 전환·재밍 연결 검증 — 2026-10-26

## 판정

**새 LiDAR 연결과 재밍 카메라 수정을 구현하고 두 bag 전체·격리 ROS에서 검증했다.**
처음에는 속도 미확정 객체 하나 때문에 전체 관측을 보류해 300ms를 넘는 공백이 생겼다.
private 입력에 `velocity_unavailable`을 추가해 UNKNOWN 점유 상태와 기하를 보존하면서 해결했다.
이 0 슬롯은 관측 위치 점유 가설이며 측정된 정지 속도가 아니다. LOW_THREAT 승격은 거부한다.
보완 후 두 bag 모두 map 정상 구간의 300ms 초과 공백이 0회다.
`scen_runner/config/stack.yaml` 기본은 `asmc_lidar`, `main`은 복구 경로다.
실제 시뮬레이터의 재밍 출구 통과 수락은 별도로 남는다.
`src/lidar+sensorfusion` 파일 변경·삭제는 0개다. 현재 실행 중인 실제 ROS 그래프의
노드를 종료하거나 시뮬레이터에 시험 제어를 보내지 않았다.

## 구현

- `autonomy_asmc_lidar.launch`: 새 LiDAR와 typed 판제 어댑터를 함께 실행한다.
- 새 `LidarPlanningObservation`: 한 scan의 객체·4단계 운동 상태·생산자/세대/순서/설정 식별자를 묶는다.
  frozen `asmc_msgs`/nested `Msg`의 기존 MD5와 기본 shadow 출력 validity는 유지한다.
- 판제 경계: stale·중복·이전 세대·무효 map을 거부한다. 정상 빈 관측만 빈 배열로 전달한다.
  속도 미확정 객체는 명시적인 unavailable flag와 UNKNOWN 상태로 기하를 유지한다.
  생산자 재시작/세대 변경 때 판제 내부 ID를 재사용하지 않는다.
- 재밍 프레임 브리지: `/perception/camera/objects`의 전방 결과를 원본 영상과 정확한 시각으로 연결한다.
  기본 소비 cutoff 0.7을 유지하고, 정상 빈 검출·오류·영상 유실을 구분한다. Main 의존성을 제거했다.
- 판제 parser·내부 객체에 unavailable flag를 보존하고 기존 미확정 점유 정책에 연결했다.
  회전 추정에는 자리값을 넣지 않으며 충돌 검사의 불확실성 여유를 유지한다.
  기존 source는 flag=false 기본값으로 기존 정책을 유지한다. GPS 무효 시 STOP도 유지한다.
  private `planning_inputs/DetectedObject*`의 MD5는 변경됐으므로 상위 workspace 전체 재빌드가 필요하다.

## 원인 근거

사용자가 제공한 두 bag 끝에서 GPS 0/0/0이 시작된다. 보존된 당시 ROS 로그에서
정상→재밍 전환은 수행됐지만 follower가 `bbox_timestamp_expired`로 정지했다.
master 로그에는 `/planning/camera_bbox_lane/yolo`를 구독한 기록이 있고 해당 토픽의
`+PUB` 등록은 0개였다. 첫 bag의 구독 등록은 00:25:54.721, 두 번째는 00:46:43.158이다.
당시 `/YOLO/BBox`로 나가던 결과와 구독 토픽이 달랐다.

이 수정은 **확인한 연결 단절 원인**을 해결한다. bag에 없는 출구 주행이나 다른 정지 원인까지
해결됐다고 판정하지 않는다.

## 두 원본 bag 전체 재처리

카메라 observation/status가 기록되지 않아 새 LiDAR replay는 fusion 없이 실행했다.
원본은 읽기 전용이고, 출력은 `$ASMC_DATA/derived/lidar-migration-20261026/`에 저장했다.
전체 bag 처리 성공과 객체 속도 정확도/실주행 수락은 별개다.

| 항목 | `00-25-48_0.bag` | `00-43-15_0.bag` |
|---|---:|---:|
| 길이 | 307.948초 | 572.478초 |
| LiDAR scan 처리 | 3,208 / 3,208 | 5,771 / 5,771 |
| typed 입력 무효 / 중복 시각 | 0 / 0 | 0 / 0 |
| map 관측 가능 프레임 | 3,162 | 5,674 |
| 실제 판제 gate 통과 프레임 | 3,162 | 5,674 |
| 속도 미확정 표시를 유지한 객체 관측 | 112 | 160 |
| 잘못된 미확정 flag/운동 상태 | 0 | 0 |
| map 유효 중 전달 간격 300ms 초과 | 0회 | 0회 |
| map 유효 중 최대 전달 간격 | 0.208초 | 0.225초 |
| 끝 20초 전방 검출 / 원본 영상 일치 | 141 / 141 | 146 / 146 |
| 전방 검출 최대 지연 | 259ms | 246ms |
| GPS 상실 300ms 후 잘못된 정상 map 출력 | 0 | 0 |
| 실제 GPS 복구 기록 | 없음 | 없음 |

기계 판독 요약은 [첫 bag](../../../config/baselines/evidence/lidar_cutover_20261026/first.json),
[두 번째 bag](../../../config/baselines/evidence/lidar_cutover_20261026/second.json)이다.
`planning_gate`는 실제 C++ 어댑터를 통과한 결과이며,
`strict_velocity_gate_before_fix`는 초기 전체 보류 방식의 반례를 재현하는 비교값이다.
새 HD맵·높이 ROI 설정을 유지했으므로 Main과 최종 객체 수가 동일하다는 기준은 사용하지 않았다.

## 자동 시험

기존 build/devel과 분리한 `/tmp/asmc-migration-build`, `/tmp/asmc-migration-devel`에서
ROS Noetic 전체 상위 workspace를 새로 빌드했다.
일반 `/root/ws/build`에는 `asmc_lidar`를 제외한 whitelist가 남아 있어 이를 해제하고
Release 실행 파일도 `/root/ws/devel`에 빌드했다. 실제 실행 중인 노드는 재시작하지 않았다.

- LiDAR CTest **33/33**, 필수 CI gate 통과. node·reset/restart·fusion·GPS 상실/복구·동일 scan
  companion 식별자·기존 shadow validity 보존을 포함한다.
- 새 판제 경계 C++ **5개**, 실제 ROS 어댑터 시험 **1개**, planning_inputs CTest **7/7** 통과.
  미확정 속도가 있어도 기하·flag가 전달되고 잘못된 정적 승격은 거부되는 것을 포함한다.
  일반 `/root/ws/devel` 실행 파일로도 어댑터 ROS 시험과 새 launch 해석을 다시 확인했다.
- 복구용 기존 LiDAR wire → private 객체 연결 및 validated 입력 ROS smoke 통과.
  별도 master에서 검사했으며 기존 Main 패키지 파일은 변경하지 않았다.
- planner CTest **62/64** 통과. 새 unavailable flag 전달·가까운 미확정 장애물 정지·먼 물체 때문에
  정지하지 않음·기존 충돌 여유·회전 추정 회귀를 포함한다.
  실패한 `command_avoid_target_test`와 `roundabout_execution_test`는 변경 전 `c98db3c6` 소스를
  별도 경로에서 빌드해 재실행했으며, 실패 체크 117개의 내용/횟수가 모두 같았다.
  [동일 실패 비교](../../../config/baselines/evidence/lidar_cutover_20261026/planner-baseline-comparison.json).
  이 기존 실패를 삭제하거나 성공으로 바꾸지 않았고, 추가 실패는 없었다.
- 직접 카메라 프레임 연결 **1개** 통과: 정상/빈 검출, cutoff, 잘못된 confidence, 잘못된 좌표,
  영상 유실, 다른 카메라 영상, 측면 결과 제외를 검사했다.
- canonical 카메라 입력을 통한 기존 차선변경 **22/22** 통과.
- 재밍 비활성 30초 초과·빠른 재진입 **2/2** 통과. 새 입력 수신 후 실제 follower 명령의 주행 재개를 확인했다.
- 격리 master의 정상→GPS 0/0/0 재밍→복구 전환과 `/ctrl_cmd` 단일 발행자 검사 통과.
  이 시험의 정상 주행 분기는 실제 경로 추종 수락을 대신하지 않는다.
- 공용 계약 **40개 메시지** 등록 검사, 기존 MD5/직렬화, 실행 계약,
  scen_runner lifecycle **12개**, readiness **7개** 통과.
  일반 devel 환경에서도 perception/planning의 `--check`가 새 LiDAR 설정으로 통과했다.

실행 로그는 `$ASMC_DATA/derived/lidar-migration-20261026/logs/`에 보관했다.

## 남은 수락 작업

1. 새 카메라 observation/status·차선 family/projection·mode/제어 진단을 함께 기록해 실제 fusion 포함 주행을 확인한다.
2. 실제 시뮬레이터에서 정상 주행과 재밍 진입·탈출·재진입을 확인한다.
   두 기존 bag만으로는 출구 수락을 할 수 없다.

실행·복구 명령은 [전환 안내](../../operations/lidar-cutover.md)에 있다.
