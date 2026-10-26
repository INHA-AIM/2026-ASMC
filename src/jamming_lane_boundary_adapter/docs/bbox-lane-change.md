# BBOX 검출에 따른 반복 차선변경

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
- 장애물까지 미터 거리나 BBOX 크기에 따른 거리 threshold는 사용하지 않는다.
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
