# 재밍 차선 추종과 LiDAR 장애물 회피

## 현재 개발 브랜치의 입력 정책 — 2026-10-29

- 변경 대상은 장애물의 미터 단위 위치·크기 입력이다. 카메라 차선 추종·신호등·객체 검출과
  기존 SensorFusion 클래스 보완은 유지한다. LiDAR 생산자·메시지·퓨전 분류 코드는 변경하지 않는다.
- 판단 노드는 카메라 BBOX/result를 구독하지 않는다. BBOX 하단 역투영·고정 6m 박스·
  카메라 차로 점유·카메라 잘림/먼 박스에 따른 제동/속도 상한을 주행 입력에서 제거했다.
  차로 판정·영상 표시는 `start_classifier=true`로 기존 실행을 유지한다.
- `use_lidar_obstacles=true`가 기본 장애물 형상 입력이다. 관측 envelope가 이미
  발행 중이면 `/perception/lidar/observations_local`의 session/generation을 사용하고,
  그렇지 않으면 기존 `objects_local` 배열 입력을 유지한다. C를 먼저, D를 나중에 실행한다.
- 형상은 OBB의 8개 꼭짓점에서 얻는다. 스캔 지연 동안의 자차 이동·IMU 회전과 유효한
  LiDAR 절대 속도를 한 번 보정한 스냅샷을 경로·동적 위험·RViz가 공유한다. 도로 위
  주행선 검사는 이 OBB를 감싸는 보수적인 범위를 사용하며, 동적 충돌 검사와 파란
  `road_obstacle` 마커는 OBB의 XY 윤곽을 사용한다.
- 유효한 LiDAR 절대 속도를 우선 사용한다. GPS 재밍으로 이 속도가 무효면 같은 객체의
  연속 LiDAR 위치와 자차 속도·IMU로 운동을 추정한다. 이때 카메라 박스는 사용하지 않는다.
  안정적인 관측 3개 전이나 ego/IMU 무효·관측 간격 과다·예측 객체는 여전히 운동 미확정이며,
  위험 영역을 확장하고 가까이 있으면 최대 3km/h로 제한한다.
- 재밍 노드가 별도의 객체 수명을 만들지 않는다. 같은 source+ID의 형상은 교체하고,
  ID 변경/producer 재시작 때 과거 위치를 누적하지 않는다. 현재 LiDAR typed 출력은
  `OBSERVED` 객체만 발행하므로, 최신 유효 관측에서 빠진 ID는 소비자에서도 제거한다.
  향후 `PREDICTED`가 제공되면 그 형상은 유지하되 속도는 불확실하게 처리한다.
  유효한 `OBSERVED_EMPTY`는 빈 관측 목록으로 반영하며, 이는 가려진 객체가 없다는
  보장이 아니다. 이번 수정은 별도의 가림 추적기를 추가하지 않는다. 입력 무효/중단은
  빈 목록으로 반영하지 않고 기존 기억을 보존한다.
- 이번에 추가했던 JAMMING 진입 후 필수 LiDAR 대기와 전역 `lidar_observation_timeout`
  정지를 제거했다. 차선 추종에 새 LiDAR 입력을 무조건 요구하지 않는다. 동적 위험 때문에
  이미 정지한 상황에서는 입력 유실을 안전 확인으로 간주하지 않는 기존 처리를 유지한다.
  외부 E-Stop, 차선 유효성 검사, 모드 전환·GPS 복귀, 기존 경로 생성과 creep 설정은 유지한다.
  **`no_clear_line`을 새 정지 정책으로 바꾸는 작업은 이번 범위에서 제외했다.**
- 최종 경로는 횡이동 축소 후 도로 경계와 장애물 footprint를 다시 검사한다. 주행 명령 직전에도
  최종 형상을 확인하며, 경로 점 사이에서 차체가 쓸고 지나가는 범위도 포함한다. 거부된 후보의
  진단 사유를 성공한 최종 경로의 실패로 표시하지 않는다.
- 장애물 형상은 계속 최신 LiDAR로 교체하며 과거 OBB를 쌓지 않는다. 복귀 판단은 별도로
  `line_return_clear_sec=1.0`초와 서로 다른 유효 스캔 3개를 요구한다. 한 프레임 미검출이나
  입력 중단만으로 복귀하지 않는다. `line_commit_sec=1.0`초 동안 반대쪽 회피 전환을 억제하고,
  후보에는 이전 목표에서의 변경 비용과 반대 방향 전환 비용을 적용한다.
- 클래스와 무관하게 도로 방향을 가로지르는 유효 운동이 있으면 현재 차로 기준 4초 예측
  (관측 경로 범위 안)의 충돌 위험을 먼저 본다. 위험하면 급한 우회선 탐색보다 최대 3km/h
  양보를 우선하며, 현재 점유/최종 경로 불통과/감속으로 해결 불가이면 정지한다. 도로 방향으로
  나란히 움직이는 객체는 단순히 base_link y 속도가 있다는 이유로 횡단으로 분류하지 않는다.
- 일반 내부 안전 정지 후 재출발은 **차속 0.1m/s 이하를 0.25초 유지**하고, 동일한 목표 경로가
  **1초 연속 안전하며 서로 다른 유효 LiDAR 스캔 3개 이상** 확인돼야 한다. 목표선 변경·
  위험 재등장·입력 무효는 안전 확인을 초기화한다. 정지 중 시간만 지났다고 출발하지 않는다.
  재출발 직후 1초는 최대 3km/h로 제한한다. 외부 E-Stop 인터페이스는 유지한다.
- 일반 `no_clear_line` creep 설정은 유지한다. 다만 이미 내부 안전 정지에 들어갔다면 유효한
  경로가 없는 creep으로 정지를 해제하지 않는다. creep의 동적 위험 검사는 이전 선택 경로 대신
  실제 마지막 조향 명령의 원호를 사용한다.
- 진단: `obstacle_geometry_source=lidar_obb`, `object_camera_geometry_used=false`,
  `lidar_source`, `lidar_snapshot_sequence`, `lidar_predicted_objects`,
  `line_obstacles`, `dynamic_state`, `dynamic_blocking_id`, `dynamic_ttc_sec`,
  `dynamic_stop_hold`, `dynamic_at_rest`, `dynamic_safe_frames`, `crossing_yield`,
  `final_path_check`, `line_clear_frames`.
  빨간 `line_obstacle`은 주행선 검사용 범위이며 파란 실측 윤곽과 구분한다.

빌드·순수 코어 단위 시험·기록 경로의 오프라인 검사로 회귀를 확인한다. ROS 노드 실행,
bag 재발행·gRPC·실제 주행은 자동 실행하지 않는다. 운전 검증은 사용자가 실행하며, 여기의 정책은 완주/무충돌 검증 결과가 아니다.

## 변경 전 동작 기록

아래는 PR #127 당시와 카메라 장애물 입력을 사용하던 변경 전 기록이다.
현재 실행 정책·기본값은 위 항목과 [launch 소스](../launch/camera_bbox_lane_change.launch)를 따른다.


## PR #127 통합·시험 경계 — 2026-10-28

- `CameraBBoxLaneResult`의 `contact_x/contact_y_min/contact_y_max` 배열은 최신 main(PR #128)과
  같은 계약을 유지한다. 그 이전 checkout은 MD5가 다르므로 producer·consumer를 함께 재빌드한다.
  실행 중인 다른 worktree의 `build/devel`을 재사용하거나 덮어쓰지 않는다.
- LOCAL 실험 제어기와 이 통합 launch를 같은 master에서 동시에 제어 출력자로 띄우지 않는다.
  먼저 별도 ROS master에서 합성 입력으로 검사한다. 실제 시뮬 시험은 기존 제어 생산자를
  명시적으로 종료한 뒤 하나만 선택한다. 브릿지·카메라를 임의로 재시작할 필요는 없다.
- `scripts/run/B_camera_lane.sh`와 `C_camera_objects.sh`는 RTX 50 개발 환경의
  `/opt/yolo-venv`·PyTorch/COCO7 설정을 선택한다. 공용 GPU 기본 실행을 대신하는 명령이 아니다.
  해당 환경이 없는 PC는 기존 `roslaunch asmc_camera camera.launch`와 검증된 설정을 유지한다.
- 이번 CI 정합성 수정은 launch 인자 안내를 실제 XML에서 재생성한 것이다. 회피 알고리즘과
  팀원이 조정한 주행 파라미터는 재튜닝하지 않았다. 통합 launch의 creep 3km/h,
  `jamming_creep_clear_distance=0`, 측면 여유 0.05m, 벽 미확인 overrun 1.5m는
  실제 장애물·벽과의 안전 여유를 보증하지 않으며 합성 시험 통과를 실주행 인수로 간주하지 않는다.
  standalone의 creep 기본값 0 및 clearance 기본값과 통합 launch override를 구분한다.
- `shadow_zone_start_m`의 30m 전진은 planner 전환 정책이며 HD맵의 공식 미션 경계나
  geometry를 변경한 것이 아니다. NORMAL→JAMMING→복귀와 LiDAR pause/resume을 함께 확인한다.

2026-10-27: PR #128에서 1024의 접지점·5개 주행선·속도/IMU 유지 경로를 통합했다.
PR #127은 그 위에 GPS 판정·복귀·LiDAR 기억·creep 개선을 적용한다. 양서준의 고주로/대시보드
연결과 `/Ego_topic/velocity.x` 종방향 속도 계약은 유지한다.

2026-10-26: 공용 카메라 결과를 기존 frame bridge가 직접 소비한다. 구 Main CPU 변환과
`camera_bbox_detector.launch`는 제거했고 Main/Msg가 필요 없다. cutoff0.7과 판정/주행 계산은 유지하며
원본 subpixel bbox를 private frame에 보존한다. 별도 detector 실행 없이 카메라와 아래 launch를 사용한다.

**IMU 투영은 연결됐고 폐루프 제어 인수는 별도다.** Classifier는 정확한 영상의 보정 context를
사용하고 follower와 동일한 현재 config SHA를 기대한다. 원래 미터 단위 xy 소비에는 자세를
다시 적용하지 않는다. metadata 누락·세대 전환은 UNKNOWN이며 기존 제어 정책이 처리한다.
[판제 인수 체크리스트](../../../docs/handoffs/camera-to-planning.md#71-판제-담당의-별도-인수-작업)에 따라
read-only/preview 이후 제어를 확인한다. standalone `enable_control=false`는 유지하며,
planner 통합 launch가 전달하던 `enable_control=true`도 변경하지 않았다.

`src/camera_bbox_lane_change_node.cpp`가 차선 중심 주행과 BBOX 기반 차선변경을 수행한다.
파일 상단에 모든 인자와 설명을 두고 같은 파일에 main을 둔다.
외부 LiDAR 모듈은 수정하지 않는다. 이 패키지의 구형 카메라 추종·LiDAR 회피 구현은 정리했다.

통합 모드에서는 `/planning/mode` (`std_msgs/UInt8`, NORMAL=0,
JAMMING=1)를 구독한다. NORMAL 동안 카메라/BBOX 관측은 갱신하지만
주행 주기와 차선변경 타이머/상태 전이는 멈추며 주행 명령을 발행하지 않는다.
JAMMING 진입 시 이전에 실행되지 않은 명령의 차선변경 진행률과 타이머를
초기화하고 새 카메라 frame으로 실제 차로를 다시 확인한다. 실제 제어 출력은
`/ctrl_cmd/jamming`이며 통합 selector만 `/ctrl_cmd`를 발행한다.

## 동작

`dynamic_obstacle_stop=true`는 사람·동물·차량 등 클래스와 무관하게 선택한 최종 주행선과
객체의 현재/단기 예측 영역을 비교한다. 기본 예측 시간은 2초이며 현재 속도의 정지 시간이
더 길면 그 시간까지, 관측된 경로 범위 안에서 검사한다. 옆에서 나란히 가는 객체는 이름만으로
정지시키지 않는다. 충돌이 예상되면 감속 후보를 검사하고, 피할 수 없으면 제동한다.
위험이 사라진 상태가 `dynamic_clear_sec=0.3`초 지속되어야 다시 출발한다. 기존 사람의
3초 정지/4초 재정지 억제는 제거했으며, `person_stop`은 새 기능의 호환용 별칭이다.
`person_stop_sec`와 `person_ignore_sec`는 기존 실행 명령 호환을 위해 받지만 판단에는 사용하지 않는다.

LiDAR 객체 ID별 위치 변화에서 자차 속도·IMU 회전을 보정해 운동을 추정한다. 입력의 유효한
절대 속도가 있으면 우선 사용한다. 카메라만 보이거나 운동이 아직 불확실한 객체는 정지 물체로
단정하지 않고 영역을 확장하며, 가까이 있으면 `dynamic_uncertain_speed_kph=3`으로 제한한다.
카메라·LiDAR가 서로 유일하게 대응하는 경우 LiDAR 형상/운동을 사용한다. 벽/천장 입력 필터와
기존 정적 회피 경로 생성은 유지한다. 이 검사는 고정 제동 모델에 따른 단기 위험 판단이며,
실제 제동 성능·회피 완주를 보장하는 검증 결과는 아니다. 입력/정지 이유는 `dynamic_state`,
`dynamic_blocking_id`, `dynamic_ttc_sec`, `dynamic_known_motion`, `dynamic_unknown_motion`으로 확인한다.
이 위험 검사는 아래 BBOX 제동 제외 옵션과 별개로 차선변경·회피·기어가기의 제어 명령에도 적용한다.

기본 `ignore_bbox_during_lane_change=true`: 장애물 회피용 차선변경 시작 조건을 통과한 후부터 이동 완료까지는
BBOX 검출(목표 차로 포함), UNKNOWN/보닛 잘림, 먼 물체 판정, BBOX 유실·지연으로 제동하지 않는다.
BBOX 기반 속도 상한도 이 구간에서는 제외하며 변경 속도/곡률 상한은 유지한다.
카메라 차선 입력 유실, 경로 생성 실패, 비상정지, 변경 시간 초과에서는 여전히 정지한다.
완료 시 BBOX 검사를 복원한다. 새 목표 차로에 장애물이 나타나도 변경 중에는 BBOX로 제동하지 않는다.
기존 변경 중 BBOX 제동 동작이 필요하면 `ignore_bbox_during_lane_change:=false`로 실행한다.

- 초기 자기 차로를 선택해 중심선을 따라간다.
- 내 차로 BBOX 검출은 서로 다른 2 frame, 옆 차로 NO_DETECTION은 3 frame 연속
  확인되면 옆 차로를 즉시 내 차로(`locked_lane`)로 지정한다. 기존 내 차로는 옆 차로가 된다. 옆 차로 확인은 평상시부터 누적하고, 검출·입력 오류 시 초기화한다.
  확인 중에는 제동한다. BBOX 판정은 관측된 차선에 한해 전방 20m까지 사용한다.
  카메라가 15m까지만 지원하거나 INVALID gap이 있으면 그 밖을 추정해서 채우지 않는다.
- 두 차로 모두 12m 이상 관측되고 BBOX 하단이 두 차로의 관측 끝보다 먼 영상 영역에
  있으면 `DEFERRED_FAR`로 보류한다. 이때 현재 차선에서 최대 5km/h로 접근한다.
  관측 범위에 들어오면 실제 차로 판정에 따라 차선변경을 판단한다.
  보류 중에는 옆 차로가 비었다고 간주하지 않으며 새 차선변경을 시작하지 않는다.
  가까운 내 차로 장애물이 함께 검출되면 변경 시작 전에는 정지한다.
  변경 중에는 `ignore_bbox_during_lane_change=false`일 때만 BBOX로 정지한다.
  중간 차선 gap은 정지한다. 화면 하단 잘림/소실선 위 BBOX 등 UNKNOWN 정지는 변경 중 제외 옵션을 따른다.
- 장애물까지 미터 거리나 BBOX 크기에 따른 거리 threshold는 사용하지 않는다. 예외는 아래
  [중앙선 걸침 장애물 회피](#중앙선-걸침-장애물-회피)의 하단 접점 지면 투영이다.
- quintic 연결 경로를 생성하고 lookahead 지점을 추종한다. 차량 모서리가 두 차로의
  바깥 경계를 벗어나는 경로는 거부한다. 이 검사는 장애물과의 충돌 거리 검사가 아니다.
- 변경 시작 시 갱신한 내 차로 index를 classifier에 보낸다. 내 차로 선택, 점유 판정 기준, RViz 초록 중심선은 같은 index를 사용한다. 상대 OWN/ADJ 라벨이 바뀌어도
  consumer는 메시지의 own_lane을 이용해 점유를 기하 차로 0/1에 매핑하므로 목표가 뒤집히지 않는다.
- `ignore_bbox_during_lane_change=false`일 때 변경 중 목표 차로에 검출이 생기면 정지한다. 다시 NO_DETECTION이 3 frame 확인되면
  같은 목표로 이어간다. 목표를 반대쪽으로 바꾸지 않는다.
- 새 차로 중심과 횡오차 0.3m / 방향오차 8도 이내가 서로 다른 차선 frame 3개에서
  확인되면 차량 이동 완료만 기록한다. 내 차로는 변경 결정 때 이미 전환되어 있으며 계속 유지한다.
  변경 중 정지나 입력 유실이 생겨도 출발 차로로 되돌리지 않는다.
- 기본 `one_lane_change_only=false`: 완료 후 새 내 차로에 장애물이 생기면 반대편 차로로 다시 변경한다.
  변경 완료 후 `lane_change_cooldown=4.0`초 동안은 재변경을 대기하며, 이때 내 차로 장애물이 있으면 제동한다.
  대기 후에도 옆 차로 연속 미검출과 유효 경로가 확인되어야 변경한다. standalone 기본 설정에서는 장애물 없는 자동 복귀를 하지 않는다.
  한 번만 변경하려면 `one_lane_change_only:=true`로 실행한다. 각 변경 시작 시 완료 상태를 초기화한다.
- `return_to_entry_lane=true`인 통합 `autonomy_no_bridges_v2.launch`에서는 회피 후 진입 차로로
  복귀할 수 있다. 변경 완료 후 기존 4초 대기, 양쪽 차로의 1초 연속 미검출과 서로 다른
  BBOX 3 frame, 관측된 목표 차로와 차체 경계 내 경로를 모두 확인한다. UNKNOWN, 먼 물체,
  잘림 추적, 입력 유실은 복귀 확인을 초기화한다. 복귀 조건이 부족하면 현재의 안전한 차로를
  계속 따라간다. 복귀 중에는 `ignore_bbox_during_lane_change=true`여도 목표 차로 검출과
  BBOX 신선도 검사를 유지한다. `one_lane_change_only=true`이면 복귀하지 않는다.
  이미 시작한 복귀가 입력 공백으로 멈췄다면, 새 유효 frame에서 목표 차로 미검출을
  확인하는 즉시 같은 복귀를 재개한다. 시작 전의 1초/3 frame 대기를 매번 반복하지 않는다.
  standalone 기본값은 false다. 진입 차로는 JAMMING 진입 때 새로 관측해 저장하며
  `entry_lane`, `returning_to_entry` 진단으로 확인한다. 전방 미검출의 한계는 아래 설명과 같다.
- 차선 입력 누락/차선 단절/비상정지는 항상 제동한다. 오래된 BBOX/UNKNOWN/양 차로 점유는
  변경 전후 또는 `ignore_bbox_during_lane_change=false`인 변경 중에 제동한다.
- 변경 중에는 목표 차로를 추종한다. 원래 차로의 먼 부분이 가려져도 목표 경로와 차량
  footprint를 현재 관측으로 검증할 수 있으면 계속 진행한다. INVALID gap을 접선으로 메우지 않는다.
  경계 안에 들어가는 후보를 찾을 때 횡이동량만 축소하고 도로 방향 정렬은 유지한다.
  변경 도중 차량이 도로와 비스듬해져도 방향 보정이 함께 약해지는 문제를 방지한다.
- 전진 명령 누적 15초 또는 연속 정지 대기 30초를 초과하면 재시작할 때까지 정지한다.
  전진 명령 시간은 실제 이동량이 아니므로 UDP 수신 실패로 움직이지 않아도 누적된다.

전방 BBOX의 NO_DETECTION은 실제 빈 차로가 보장된다는 뜻이 아니다. 이 구현에는
장애물 거리, 후방 차량, 맞은편 차량의 미래 움직임을 확인하는 기능이 없다.
현재 경계 계약은 왕복 2차로 후보라 옆 차로가 반대 방향일 수 있다.
직선 시뮬레이터에서 차선변경을 시험하는 코드이며 일반 도로 회피 검증 결과가 아니다.

## 중앙선 걸침 장애물 회피

### 선 선택 (기본 `avoid_line_straddle=true`, `/Ego_topic` 속도가 있을 때)

카메라 차선으로 왕복 2차로에 기준선 5개를 둔다. 1 왼쪽 도로 끝, 2 왼쪽 차로 중앙, 3 중앙선, 4 오른쪽 차로 중앙,
5 오른쪽 도로 끝. **이 중 따라갈 선을 고르는 것이 곧 내 차선 선택**이다. 고른 선이 속한 차로를 내 차선(초록)과
판정기 기준 차로로 쓴다(선 3 위면 이전 차선 유지). 별도의 차선변경·offset 회피 상태는 없다.

- 장애물(카메라 접점, 켜면 LiDAR)은 덮는 범위를 선 번호로 바꿔 기억하고 속도 적분으로 위치를 옮긴다.
  `obstacle_confirm_frames`번 확인된 것만 쓰며, 화면 밖으로 나가도 지나갈 때까지 유지한다.
- 매 주기: 지금 선이 비어 있으면 유지한다. 막히면 지금 선에서 가까운 순서(`avoid_line_step` 0.25, 같으면
  오른쪽 먼저)로 피할 수 있는 선을 고른다. 앞 장애물의 끝+차량 후방+`avoid_pass_margin`을 모두 지났고
  가까운 차로 중앙(2 또는 4) 경로가 비어 있으면 복귀한다(옆을 지나는 동안에는 고른 선을 유지). 피할 선이 없으면 `no_clear_line`으로 정지한다.
- "피할 수 있음": 판단 여유(반폭+`avoid_clearance`+`avoid_plan_margin`), 차체 모서리가 주행 영역 안,
  차로 중앙 기준 횡이동 `avoid_max_offset` 이하, 그리고 연결 경로(전환거리 1.0~1.6배)가 장애물 영역·곡률 검사 통과.
- 장애물 영역은 시작·끝 x의 기준선으로 도로 방향에 정렬한다. 차량이 비스듬해도 옆 차로로 번져 보이지 않는다.
- 진단: `line_mode`, `line_target`, `line_decision`(예: `cover 3.09-4.91 switch to line 2.00`), `line_obstacles`.
  RViz: 흰 선 1~5, 주황 TARGET, 빨간 사각형(기억한 장애물).

- 차선 유실 유지: 선 선택 중 카메라 차선이 끊기면(회피 중 차량이 도로에 대해 크게 틀어지면 차선 노드가
  `too_few_observed_roles`로 거부할 수 있음) 마지막 정상 차선을 IMU 회전 각속도·차속 추측 항법으로 옮겨 계속 쓴다.
  `lane_hold_distance`(15m)·`lane_hold_time`(8s) 안에 차선이 돌아오면 이어서 주행하고, 넘으면 정지한다.
  관측 범위 밖 차선은 끝 1m 기울기로 직선 연장한다. 진단: `lane_hold_active`, `lane_hold_travelled_m`, `imu_fresh`.

예(3.5m 차로, 내 차로=오른쪽, 선4): 중앙선 걸침이 내 차로로 0.45m → 선 4.25, 1.1m → 4.5, 1.55m → 4.75,
내 차로 중앙까지 → 선 5, 내 차로 전체 → 선 2(내 차선이 왼쪽 차로로 바뀜), 차로 중앙 0.5m 물체 → 선 5.
속도 입력이 없으면 아래 기존 동작(카메라 점유 기반 차선변경·정지)을 쓴다.

### 공통

기본 `straddle_avoidance=true`. 내 차로 점유가 확인되고 아래 입력이 모두 있을 때만 동작하며,
하나라도 없으면 위의 기존 동작(BOTH/옆 차로 검출 시 정지)을 그대로 따른다.

- classifier가 OWN/ADJACENT/BOTH 박스의 하단 양 끝을 지면 homography 역행렬로 투영해
  `contact_x`, `contact_y_min`, `contact_y_max`(base_link, m)를 낸다. 잘린 박스·연속 추적 박스·
  UNKNOWN/OUTSIDE/DEFERRED_FAR는 NaN이며, 점유 박스 중 하나라도 NaN이면 회피 판단을 하지 않는다.
  평지 가정이며 비스듬한 물체의 먼 측면은 자차 중심선 쪽으로 넓게(보수적으로) 추정된다.
- 차로마다 차량 중심이 들어갈 수 있는 범위를 계산한다. 차선 경계에서 `반폭+obstacle_lateral_margin`,
  장애물 하단 가장자리에서 `반폭+avoid_clearance`를 뺀 구간 중 차로 중심에 가장 가까운 점이 offset이다.
- 실제 횡이동량이 작은 차로를 고른다(옆 차로는 두 차로 중심 간격 + 그 차로 offset). 차이가 `avoid_lane_tie` 이하면 내 차로를 유지한다.
- 기본 `avoid_line_straddle=true`: 일반 구간 Frenet AVOID와 같은 방식이다. 회피는 내 차로 기준 횡 offset
  (최대 `avoid_max_offset` 3.5m)이며 차선은 넘을 수 있다(페인트는 hard 한계가 아님). 차로 소유권은 바꾸지 않고
  지나간 뒤 원래 중심으로 복귀한다. hard 한계는 장애물 간격과 주행 가능 영역이다.
- 주행 가능 영역: 카메라 차선의 도로 바깥 끝. LiDAR가 그 바깥 구조물(벽·기둥·갓길 물체)을 확인한 쪽만
  구조물 − `avoid_wall_margin`(0.3m)까지, 최대 `avoid_max_shoulder`(2m) 넓힌다. 벽을 확인하지 못한 쪽(카메라만
  사용할 때 포함)은 차체가 바깥 차선을 `avoid_line_overrun`(1.0m)까지 넘을 수 있다(차량 중심이 선1·선5까지). 중앙선에 걸친 장애물을
  바깥쪽으로 더 비켜 가기 위한 값이며, 3.5m 차로에서 장애물이 내 차로 중앙 근처까지 들어와도 선1/선5로 지나간다.
- 전환 길이는 실제 횡이동량과 곡률 한계(조향 한계의 80%)로 정하고, 일반 구간 저속 AVOID처럼 1.0~1.6배 후보 중
  충돌 없는 가장 짧은 것을 쓴다.
- 복귀(RETURN): 기억한 모든 장애물의 중심을 지난 뒤 d=0 복귀 경로가 충돌·주행 영역 검사를 통과하면 즉시 복귀한다.
  통과 거리 적분은 그 밖의 해제 경로로 남긴다.
- offset 판단에는 `avoid_plan_margin`(0.05m)을 더해 경로 검사와 경계가 맞닿지 않게 한다.
- 먼 장애물(경로 범위 밖)도 판단에 포함한다. 차선 관측으로 차로에 대응시킬 수 없으면 회피 판단 없이 기존 동작을 따른다.
- 내 차로 회피 경로가 막히면 `avoid_reconsider_distance`(0.5m) 이동 전에는 회피를 풀고 차로 선택을 다시 하고,
  이미 진행했다면 장애물 기억을 유지한 채 정지한다. 같은 물체의 재측정은 기록을 교체한다.
  - 옆 차로가 더 덮임 → 같은 차로에서 중심을 장애물 반대쪽으로 offset만큼 밀어 지나간다.
  - 내 차로가 더 덮임 → 옆 차로로 변경하고, 옆 차로에도 걸쳐 있으면 그 중심을 바깥쪽으로 민다.
  - 두 차로 모두 공간이 없으면 기존 정지 동작.
- 내 차로 겹침이 점유 기준(10%) 미만이라 ADJACENT로 분류된 옆 차로 장애물도 같은 계산을 한다.
  넘친 만큼 간격이 부족하면(offset≠0) 같은 차로 안에서 중심을 밀고, 확인(`adjacent_clear_frames`) 전에는
  정지하지 않고 기존 중심선을 따른다. 내 차로에 공간이 없으면 `avoid_lane_space_insufficient`로 정지한다.
  offset = 넘친 폭 − (차로 반폭 − 차체 반폭 − `avoid_clearance`). 3.5m 차로·기본 간격 0.5m에서는
  약 0.3m까지의 넘침은 이미 간격 안이라 중심을 옮기지 않는다.
  - offset 0이고 옆 차로가 NO_DETECTION이면 기존 차선변경 절차를 그대로 쓴다.
- 시작 조건: 내 차로 검출 `obstacle_confirm_frames`, 같은 차로 선택이 `adjacent_clear_frames`의 새 BBOX frame.
  확인 중에는 `confirming_straddle_avoidance`로 정지한다.
- 회피 경로 검사: 장애물을 하단 접점부터 전방 `avoid_obstacle_length`(기본 6m)인 사각형으로 가정하고
  차량 footprint(방향 포함)와 겹치면 거부한다. 차로변경은 `max_lateral_step`으로 중간에서 끝나는
  실행 경로가 아니라 재계획으로 수렴할 제한 없는 연결 경로로 검사한다.
- 기억한 장애물의 횡위치는 회피 차로 중심 기준 상대값으로 저장하고 매 주기 현재 차선 관측으로 되돌린다.
  종방향 위치는 `/Ego_topic` 속도 적분으로 줄인다. 속도가 `ego_speed_timeout`을 넘게 끊기면
  `avoid_ego_speed_timeout`으로 정지한다.
- 장애물 끝 + 차량 후방 + `avoid_pass_margin`을 지나면 offset을 해제하고 차로 중심을 따른다.
- 회피 중 속도 상한 `avoid_speed_kph`. `ignore_bbox_during_lane_change=true`이면 차선변경과 같이
  회피 중 BBOX 정지·감속을 제외한다(잘린 박스로 인한 UNKNOWN 포함). 새로 측정된 장애물은 경로 검사에 추가되고,
  같은 방향으로 더 큰 offset이 필요하면 키운다. 같은 차로 안 이동은 `avoid_shift_length`,
  추종점은 `avoid_lookahead_distance`를 쓴다.

LiDAR 반영(기본 `use_lidar_obstacles=false`, 실주행에서 LiDAR 오검출로 회피가 막혀 기본 해제. 켜려면 `use_lidar_obstacles:=true`): `Main`을 `enable_common_geometry:=true`로 실행하면 음영에서도
`/perception/lidar/objects_local`(base_link OBB)이 나온다. 각 물체의 축 정렬 외접 사각형을 현재 카메라 차선으로
계산한 차도 폭(두 차로 바깥 끝)과 비교해 `lidar_road_min_overlap`(0.2m) 이상 겹치는 것만 차도 위 물체로 쓴다.
터널 벽·기둥(차도 밖)과 `lidar_max_length`(20m) 초과 벽 조각, 낮은 노면 점(`lidar_min_height`)은 제외한다.
차도 위 LiDAR 물체는 카메라 접점과 합쳐 offset 판단·경로 검사에 쓰고, 실측 길이로 통과 거리를 계산한다.
LiDAR가 내 차로와 `lidar_lane_min_overlap` 이상 겹치면 카메라 박스가 없어도(신뢰도 미달·잘림) 점유로 본다.
LiDAR가 있으면 카메라의 먼 미확정 박스가 있어도 차로 선택을 하고, 잘린 박스 시작 금지 규칙을 적용하지 않는다.
측정 기준으로 두 차로 모두 공간이 없으면 `avoid_no_room_in_road`로 정지한다(LiDAR를 모르는 기존 차선변경으로
막힌 차로에 들어가지 않음). LiDAR가 `lidar_timeout`(0.6s)을 넘게 끊기면 카메라만 사용한다.
회전한 차량은 외접 사각형이라 폭이 넓게(보수적으로) 잡힌다. 장애물 차도 판정은 카메라 차선 기준이다.
LiDAR 최신 여부는 도착 시점 지연과 수신 간격으로 판단하고(현재 시각 기준 나이는 처리 지연+주기가 겹쳐 흔들림),
측정 이후 이동 거리(속도×지연)만큼 장애물을 가깝게 보정한다. 벽 등 정적 구조물은 `lidar_structure_hold`(1.5s)
동안 이동 보정해 유지해 주행 영역이 흔들리지 않게 한다. 경로 거부 이유는 diagnostics `path_block_reason`,
현재 주행 영역은 `corridor_at_lookahead_m`으로 확인한다.

한계: 차로가 3.5m이면 고른 차로에 장애물이 약 0.8m 이상 침범하면 통과하지 않는다. 회피 차선변경은 연결 경로(10m)와
차량 앞 길이(약 4.6m) 때문에 장애물이 대략 14m 이상 앞에 있을 때 결정되어야 경로 검사를 통과한다.
장애물 길이는 보이지 않으므로 가정값이며 버스·트럭은 `avoid_obstacle_length`를 늘려야 한다.
오프셋 없는 기존 차선변경(장애물이 내 차로 안에만 있음)에는 이 장애물 검사를 적용하지 않는다.

`/planning/camera_bbox_lane_change/diagnostics`의 `avoid_active`, `avoid_lane`, `avoid_offset_m`,
`avoid_travelled_m`, `measured_obstacles`, `ego_speed_fresh`와 RViz `avoid_obstacle` 주황 상자,
BBOX 영상의 `x .. y [..]m` 표시로 확인한다. 메시지 필드가 추가되었으므로 classifier와 controller를 함께 재시작한다.

검증(asmc-ros-noetic, 합성 입력): 차선변경 통합 37개(선 선택 구조로 회피 테스트 재작성, 신규 15개: 차선 유실 시 3초 유지 후 정지, 내 차로 막힘+먼 미확정 박스 → 선 2, 중앙선 걸침 1.15m 침범 → 바깥 차선 0.5m 넘어 회피, LiDAR 벽 확인 시 갓길까지 주행 영역 확장 후 회피, LiDAR 단독 차도 위 물체 회피(벽·긴 조각 제외, 실측 길이로 해제), 카메라는 비었다는데 LiDAR가 막힌 옆 차로는 정지, 먼 미확정 박스와 가까운 걸침 장애물, 차로 중앙 0.5m 장애물 → 차선 위 1.75m 회피 후 22m 이동 뒤 복귀, 경로 범위 밖 차로 가로막힘 → 옆 차로 변경, ADJACENT 분류 상태의 0.15m 넘침 → 간격 0.75m에서 -0.10m offset, 옆 차로 쪽 걸침 → 같은 차로 -0.15m offset 후
18.8m 이동 뒤 해제, 내 차로 쪽 걸침 → BOTH 상태에서 옆 차로 변경 후 완료, 공간 부족 정지, 속도 입력 없음 시 기존 정지),
차로 판정·절단·ego mask·비활성 lifecycle 테스트 통과. 차량 운동이 없는 합성 입력이며 실제 시뮬레이터 회피는 미검증이다.

## 종방향 제어

기본 `pedal_speed_control=true`: `/Ego_topic` 실제 속도와 목표 속도의 차이로 가속·브레이크 페달을 PI로 계산해
`longlCmdType=1`로 보낸다(`speed_kp` 0.3, `speed_ki` 0.1, 페달 상한 0.5). 가속과 브레이크를 동시에 밟지 않고,
정지는 기존처럼 `stop_brake`(1.0)를 쓴다. MORAI 내장 속도 제어(`longlCmdType=2`)는 5km/h 같은 저속 목표에서
가속 페달을 거의 밟지 않아 1km/h 안팎으로만 움직였다. 속도 입력이 없으면 내장 속도 제어로 대신한다.

## 실행

브리지, 카메라 차선 인식, GPU YOLO는 켜둔다.
통합 운용은 `roslaunch asmc_planner mode_control_integration.launch`를 사용한다.
아래 레거시 단독 launch는 `/planning/mode=1`의 권위 있는 publisher와
별도의 selector가 없으면 생산 `/ctrl_cmd`를 내보내지 않는다. 같은 `/ctrl_cmd`를
발행하는 다른 controller와 BBOX 관측 전용 launch는 종료한다. 이 launch가
BBOX 판정 노드도 함께 실행한다. GPU detector는 중복 실행하지 않는다.

카메라 YOLO가 아직 안 켜져 있으면 별도 터미널에서 실행한다. 이미 켜져 있으면 반복 실행하지 않는다.

```bash
roslaunch asmc_camera camera.launch mode:=active models:=objects
```

전방 검출은 `/perception/camera/objects`에서 직접 받는다. 별도 detector 어댑터와 중첩 workspace source는 필요 없다. 기본 소비 cutoff는 0.7이며 `bbox_conf_thres`로 지정한다.

차선 추종 + BBOX 차선변경 + RViz (**JAMMING 모드에서 `/ctrl_cmd/jamming` 발행**):

```bash
docker exec -it -e DISPLAY="$DISPLAY" asmc-ros-noetic bash -lc '
source /opt/ros/noetic/setup.bash &&
source /root/ws/devel/setup.bash &&
roslaunch lane_boundary_adapter camera_bbox_lane_change.launch \
  enable_control:=true \
  target_speed_kph:=20.0 \
  inferred_speed_kph:=20.0 \
  lane_change_speed_kph:=15.0 \
  far_obstacle_speed_kph:=5.0 \
  lookahead_distance:=8.0 \
  one_lane_change_only:=false \
  ignore_bbox_during_lane_change:=true \
  obstacle_confirm_frames:=2 \
  bbox_consumer_x_max:=20.0 \
  expected_config_sha256:=1657f282fe70c13bdb285bbf2536405768a07b2b6462590e71c75787ef6d0cf9 \
  calibration_file:=/root/ws/src/asmc_camera/config/cameras_v3.yaml \
  use_rviz:=true
'
```

최신 `asmc_camera perception.launch`는 카메라 처리량에 맞춰
`camera_model_grid_max_hz:=5.0`으로 실행한다. classifier와 controller의
`max_sensor_skew`는 같은 launch 인자로 전달된다. 위 해시는 현재 v3 설정에 대한 값이며
설정이 달라지면 검토 없이 실시간 메시지에서 읽어 자동 승인하지 않는다.

MolitComp03 로컬 실행은 MORAI에
`config/morai/network/261027_ASMC_Local_Network_25.S4.MolitComp03.json`을 Import하고
네트워크를 연결해야 한다. Ctrl Host 9093, Ego Status 목적지 9099가 현재 브릿지와 일치해야 한다.
ROS `/ctrl_cmd` 발행만으로 실제 차량 수신·이동을 확인할 수는 없다.

`enable_control:=false`는 판단/경로 미리보기만 한다. launch 기본값도 false다.
카메라/모델 설치와 관측 전용 실행은 [CAMERA_BBOX_LANE.md](bbox-lane.md)를 참고한다.

## 주요 인자와 토픽

- `one_lane_change_only=false`: 장애물에 따른 반복 변경 허용. 기존 실행 명령의 `:=true`도 변경해야 한다.
- `lane_change_cooldown=4.0`: 변경 완료 후 재변경까지 최소 대기 시간(s).
- `straddle_avoidance=true`, `avoid_clearance=0.5`, `avoid_speed_kph=8`, `avoid_obstacle_length=6`,
  `avoid_pass_margin=2`, `avoid_lane_tie=0.1`, `ego_topic=/Ego_topic`: 중앙선 걸침 회피(위 절 참고).

- `ignore_bbox_during_lane_change=true`: 변경 중 BBOX 정지·감속 제외. 시작 판단과 완료 후 BBOX 검사는 유지.

- `obstacle_confirm_frames=2`, `adjacent_clear_frames=3`: 시작/재개 확인 frame 수.
- `bbox_consumer_x_max=20`: BBOX 차로 판정의 관측 범위 상한(m). 제어 경로 범위와 별개이며 거리 추정값이 아니다.
- `lane_change_timeout=15`, `lane_change_pause_timeout=30`: 전진 명령 누적/연속 정지 시간 제한(s).
- `lane_change_length=10`: 연결 경로 전방 길이(m). 장애물 거리 threshold가 아니다.
- `lane_change_speed_kph=15`: 변경 중 속도 상한. 곡률에 따라 더 낮아질 수 있다.
- `far_obstacle_speed_kph=5`: 먼 BBOX 차로 미확정 시 현재 차선 접근 상한(km/h), 허용 0~5. 0이면 정지.
- `lookahead_distance=8`: 추종 목표점 전방 x(m).
- `bbox_source_timeout=0.5`, `bbox_receive_timeout=0.5`: BBOX 시각과 미수신 제한(s).
- `source_timeout=0.5`, `receive_timeout=0.35`: 차선 시각과 미수신 제한(s).
- `start_classifier=false`: classifier를 별도로 관리할 때만 사용. 이 경우 classifier에
  `reference_lane_topic:=/planning/camera_bbox_lane_change/reference_lane` 연결이 필요하다.

`/planning/camera_bbox_lane_change/diagnostics`의 `change_state`, `target_lane`,
`locked_lane`, `change_completed`, `lane0_state`, `lane1_state`, `bbox_stop_suppressed`를 확인한다.
변경 중 BBOX 정지를 제외할 때 상태 메시지는 `changing_without_bbox_stop`이다.
RViz 경로/빨간 목표점은 `/planning/camera_bbox_lane_change/markers`, BBOX 영상은
`/planning/camera_bbox_lane/image`다. 비상정지 Bool은
`/planning/camera_bbox_lane_change/emergency_stop`이며 true가 제동이다.

## 빌드와 테스트

```bash
docker exec -it asmc-ros-noetic bash -lc 'source /opt/ros/noetic/setup.bash && source /root/ws/devel/setup.bash && cd /root/ws && catkin_make --pkg lane_boundary_adapter -j2'
```

```bash
docker exec -it asmc-ros-noetic bash -lc 'source /opt/ros/noetic/setup.bash && source /root/ws/devel/setup.bash && source /root/ws/src/lidar+sensorfusion/devel/setup.bash && rostest lane_boundary_adapter camera_bbox_lane_change.test && rostest lane_boundary_adapter camera_bbox_lane.test'
```

통합 테스트는 별도 ROS master에서 합성 영상과 실제 YOLO 메시지를 입력해
bridge → classifier → controller를 연결한다. 제어 명령은 `/bbox_change_test/command`로
격리하며 `/ctrl_cmd` publisher가 없음을 확인한다. 좌우 변경, 합성 차선 이동 중 중앙선
통과/완료/유지, 목표 차로 검출, 중복 frame, 센서 단절, 비상정지, timeout을 검사한다.
이 합성 입력 테스트에는 차량 운동이 포함되지 않는다.

검증 결과: 빌드 통과, 차선변경 통합 테스트 18개, 차로 판정 10개, 절단 추적 3개(총 31개) 통과.
추가 검증은 원래 차로 가림 후 변경 완료, 목표 차로 gap 정지, 정지 시간과 주행 시간 분리,
연속 정지 timeout 유지, 18m 관측 BBOX 판정 및 INVALID 영역 거부를 포함한다.
먼 BBOX의 5km/h 접근, 관측 끝 진입 시 정지 띠 없음, 차로 판정 후 변경,
가까운 내 차로 검출과 먼 BBOX 동시 존재/변경 중 미확정 시 정지, 센서 단절 정지,
`far_obstacle_speed_kph=0`으로 접근 비활성화도 확인했다.
최종 테스트 로그(asmc-ros-noetic): 차로 판정 `/tmp/bbox-far-final-regression-wj57g0mr`,
제어 `/tmp/bbox-heading-regression-crmrc6wu`.
좌우 변경 도중 차선 기울기 ±0.26, 중심 횡이동 상태에서도 경로 생성과 변경 완료를 검사했다.
실제 센서 입력 미리보기에서는 먼 BBOX 라벨 5와 속도 5km/h/브레이크 0을 확인했다.


실제 시뮬레이터에서는 `/Ego_topic` 속도 5km/h와 브레이크 0, 변경 시작을 확인했다.
그 후 발생한 경로 거부는 횡이동 제한과 방향 정렬을 분리하는 수정으로 회귀 검증했다.
전체 실제 회피 완료는 아직 확인하지 않았다. 변경 중 비스듬히 정지한 상태에서 재시작하면
전방 차로 선택 지점이 경계에 걸려 `no_continuous_own_lane_to_lookahead`가 나올 수 있다.
이 경우 차량을 시작 차로 중앙에 도로와 평행하게 놓은 뒤 launch를 다시 실행한다.


## RViz 두 차로 중심선

`/planning/camera_bbox_lane_change/markers` (MarkerArray), Fixed Frame `base_link`:

- 초록색 `MY LANE`: 현재 제어 기준 차로의 인식 중심선과 lookahead 중심점.
- 자홍색 `ADJACENT`: 옆 차로의 인식 중심선과 lookahead 중심점.
- 하늘색: 실제 추종 경로. 빨간색: 실제 조향 목표점.
- 회색 `UNASSIGNED`: 초기 차로가 아직 확정되지 않은 관측 중심선.

각 중심점은 `lookahead_distance`(기본 전방 x=8m)에서 표시한다.
차선변경을 결정하면 내 차로 상태 자체를 옆 차로로 전환한다. 새 내 차로를 즉시 초록색 MY LANE (CHANGING)으로 표시하고, 출발 차로는 자홍색으로 바꾼다.
완료 후에는 같은 차로를 초록색 MY LANE으로 유지한다.
BBOX나 경로 조건 때문에 정지해도 최신 유효 차선은 계속 표시한다.
차선 입력이 오래되거나 invalid이면 지우고, 끊긴 관측 조각 사이를 잇지 않는다.
기존 launch의 `use_rviz:=true`로 이 표시가 포함된 설정을 연다.


## YOLO 보닛 제외

아래 crop 설명은 구형 입력의 기록이다. 현재 기본은 카메라 원본 검출과 classifier의 ego mask이며, 주행 전용 crop 추론을 시작하지 않는다.

현재 전방 1280×720 프레임에서 자차 보닛이 y≈578부터 보이며 YOLO가 이를 car로 검출했다.
`camera_bbox_crop.py`가 주행 전용 YOLO 입력의 아래 21%를 제외해 1280×568로 전달한다.
resize/상단 이동 없이 원본 좌표와 header를 유지하므로 기존 지면 투영에 그대로 대응한다.
차선 추론 입력과 LiDAR 공용 YOLO 구현은 변경하지 않는다.

- `hood_crop_bottom_ratio=0.21`: 아래쪽 제외 비율. 0이면 비활성화, 허용 범위 [0, 0.5).
- 이 값을 바꿀 때 **detector launch와 lane change/classifier launch에 동일하게 지정**한다.
- `/planning/camera_bbox_lane/detector_input/compressed`: YOLO에 실제 들어가는 절단 영상.
- 기존 RViz BBOX 영상은 원본 크기를 유지하며 아래 제외 영역을 회색과 `YOLO EXCLUDED`로 표시한다.
- 절단선에 닿거나 아래에 있는 BBOX는 `bottom_contact_clipped_by_hood_crop`로 UNKNOWN 처리한다.
  잘린 물체의 BBOX 하단을 실제 도로 접점으로 해석하지 않는다.

실제 저장 프레임 비교: 원본 보닛 car confidence 0.737 → 절단 후 해당 검출 없음.
독립 ROS 검증: 1280×568 입력, 원본 시각/좌표 유지, 일반 BBOX 판정,
절단선/원본 보닛 BBOX UNKNOWN 처리 통과. 로그: `/tmp/hood-crop-check-eoyj323k` (asmc-ros-noetic).


## 차로 영역과 BBOX 접촉 기반 변경

판정 노드는 BBOX 하단 주변의 작은 영역과 영상에 투영한 실제 차로 영역의 겹침 비율을
계산해 OWN/ADJACENT/BOTH를 보낸다. 기본적으로 하단 위·아래 각 8px를 검사하며,
각 차로의 비율 10% 이상을 검출로 본다. 세부 조건은 CAMERA_BBOX_LANE.md를 참고한다.
내 차로 검출이 확인되고 옆 차로는 연속 미검출이면 기존 상태 전이가 옆 중심선을 목표로 고정한다.
양 차로 검출은 변경 시작을 막는다. 변경 목표 차로 검출에 의한 변경 중 정지는
`ignore_bbox_during_lane_change=false`일 때 적용한다. 부분 관측을 빈 차로로 간주하지 않는다.

`contact_band_height_px`, `contact_row_tolerance_px`, `min_contact_overlap_ratio`,
`min_contact_support_rows`를 주행 launch에서 조절할 수 있다.
추가 검증은 관측 끝 ±5px 접점 변화, 좌우 관측 길이 차이, 작은 겹침/양쪽 겹침,
관측 끝 장애물에 의한 변경 시작과 목표 차로 침입 시 정지를 포함한다.
접촉 영역 변경 시 로그: `/tmp/bbox-contact-regression-ehqvp_pi` (차로 판정 10개, 주행 15개).
현재 실제 영상은 옆 장애물 BBOX가 보닛 제외선에 잘린 상태여서 정지가 유지됐다.
이 변경으로 전체 실주행 회피 완료를 확인한 것은 아니다.


## 보닛 절단 시 기존 장애물의 제한된 연속 추적

절단 전에 같은 기하 차로에서 2개 이상 frame으로 확인한 장애물에 한해 다음 조건을 모두 검사한다.

- 같은 class, IoU≥0.30, 이전 frame과 간격≤0.25초, 유일한 일대일 대응.
- 현재 보이는 하단 띠도 기존 차로와 80% 이상 겹치고 다른 차로와는 2% 이하 겹침.
- 마지막 잘리지 않은 접점 관측 후 최대 3초. 잘린 frame으로 이 시간을 갱신하지 않음.
- 보닛 절단 경계에 걸린 경우만 적용. 입력 단절/invalid 및 매칭 실패 때 연결을 버림.

결과의 `clipped_association[]`으로 불완전한 현재 관측임을 명시한다.
이 근거만으로 새 차선변경을 시작하지 않는다. 아래 변경 중 동작은
`ignore_bbox_during_lane_change=false`일 때 적용된다. 이미 변경 중인 출발 차로 장애물의 연속 관측이면
`clipped_tracking_speed_kph=5` 상한으로 이어간다. 완료 후 기존 차로 장애물을 지나갈 때도 같은 상한을 쓴다.
목표 차로 장애물, 새로운 잘린 물체, 다중 매칭, 다른 차로로 이동, 추적 만료이면 정지를 유지한다.
BBOX 연속성에 기반한 제한된 추정이며 가려진 접점/충돌 여유를 복원한 결과가 아니다.

메시지에 필드가 추가되었으므로 빌드 후 주행 launch(classifier+controller)를 함께 재시작해야 한다.
카메라 추론과 YOLO는 그대로 사용할 수 있다. 실행 중 노드는 개발 과정에서 임의 재시작하지 않았다.


최종 검증: 기존 30개 테스트 `/tmp/bbox-clipped-continuity-gdttrzz6`,
반복 변경 차단 추가 테스트와 통과/정지 재확인 `/tmp/clipped-second-change-vilh70c1`.
목표 중심점이 변경 시작 즉시 초록색으로 전환되는 것과 classifier 기준 차로 전환,
새 물체/추적 만료 정지, 목표 차로 검출 시 정지, 잘린 박스로 두 번째 변경 시작 금지를 검사했다.
실제 시뮬레이터 노드를 재시작하거나 차량을 구동하지 않았다.
처음부터 박스가 잘린 상태에서 재시작하면 정상 접점 이력이 없으므로 정지가 유지된다.
장애물이 완전히 보이는 접근 위치부터 실행해야 이 연속 추적 동작을 검증할 수 있다.


내 차로 즉시 전환 수정 후 차선변경 통합 테스트 18개를 다시 실행해 통과했다.
좌우 변경 결정 시 `locked_lane`이 완료 전에 전환되는지, 변경 불가 시 기존 차로를 유지하는지,
목표 차로 검출/차선 gap으로 정지해도 새 내 차로를 유지하는지 확인했다.
로그(asmc-ros-noetic): `/tmp/lane-ownership-x50jl225`. 실제 주행 노드는 재시작하지 않았다.


BBOX 제동 제외 추가 후 빌드 및 주행 통합 테스트 21개 통과. 기존 제동 모드 18개와
기본 제외 모드 3개(변경 중 목표 박스/UNKNOWN/수신 유실 지속, 완료 후 검사 복원,
먼 물체 속도 0의 간접 제동 제외, 차선 유실/비상정지/목표 차선 gap 정지)를 검사했다.
독립 master 로그(asmc-ros-noetic): `/tmp/bbox-suppress-6z4wgux6`.
실제 주행 노드는 실행하거나 재시작하지 않았고 실주행 회피 완료는 미검증이다.


반복 변경 기본 허용 수정 후 빌드 및 주행 통합 테스트 22개 통과. 새 기본 설정으로
오른쪽→왼쪽→오른쪽→왼쪽 세 번의 변경 완료, 매 변경 시작 시 완료 상태 초기화,
classifier 기준 차로 전환, 4초 cooldown, 옆 차로 검출 시 시작 차단, 장애물 없는 자동 복귀 방지를 확인했다.
독립 master 로그(asmc-ros-noetic): `/tmp/repeat-lane-change-tzo8kah4`.
실제 주행 노드는 재시작하지 않았고 실주행 반복 회피는 별도 확인이 필요하다.
