# 통합 재밍 실행

2026-10-28: 아래는 PR #128의 1024 이식 근거다. 이후 PR #127에서 GPS 판정·진입 차로 복귀·
LiDAR 기억·creep와 통합 launch override를 추가했다. 현재 세부 정책과 격리 시험·재빌드 경계는
[BBOX 차선변경](bbox-lane-change.md)을 따른다. `velocity.x` 종방향 속도·단일 카메라 소유권·
고주로/대시보드 연결은 유지하며, 이전 standalone 기본값을 통합 override로 해석하지 않는다.

판단·제어와 BBOX 접지점 생성은 `1024`의
`d544dd3c8cb401012f89064caea8e5b70e36ba29`에서 가져왔다.
노드 두 개, `CameraBBoxLaneResult`의 접지점 필드, 제어 launch 기본값을 함께 적용한다.
기존 normal planner, GPS 모드 판단, 제어 선택기와 카메라 추론 소유권은 유지한다.

입력은 기존 `/perception/camera/objects` → 원본 영상 동기화 bridge → 접지점 classifier →
재밍 제어기 순서다. Main/Msg YOLO 변환기를 추가 실행하지 않는다.
`/Ego_topic/velocity.x`의 종방향 속도(m/s)와 `/imu`의 yaw rate를 사용한다.
현재 `asmc_planner/config/competition_speed_state.yaml`의 normal 속도 입력도 같은 토픽이다.
이 배포의 MORAI CompetitionInfo는 기존 bridge의 Ego receiver로 들어오므로 토픽 이름만
보고 `/Competition_topic`으로 바꾸지 않는다. standalone에서는 `ego_topic`으로 명시할 수 있다.
제공되지 않는 velocity.y/z, 자차 절대 위치, 가속도는 적분 입력에 사용하지 않는다.
속도가 오래되면 측정 회피/추측 항법의 유효 조건이 해제된다.
`use_lidar_obstacles=false` 등 1024의 원래 기본값을 유지한다.

`far_obstacle_lane_unresolved`는 제거한 오류 코드가 아니다. 측정된 접지점으로 가능한
주행선·회피를 먼저 검사한 뒤, 관측 밖 박스로 차로를 정할 수 없는 경우의 대기 이유다.
지도/카메라 hash와 정확한 영상 시점 projection 검사를 우회하지 않는다.

## 재빌드

현재 실행 중인 planner launch는 사용자가 종료한 뒤 컨테이너에서 실행한다.
새 메시지와 producer/consumer를 함께 빌드해야 한다.

```bash
cd /root/ws
source /opt/ros/noetic/setup.bash
catkin_make -j4 -l4 -DCATKIN_WHITELIST_PACKAGES=""
source /root/ws/devel/setup.bash
```

UDP bridge, 카메라 perception, LiDAR 실행법은 기존과 같다.
planner가 classifier와 재밍 제어기 및 대시보드를 포함한다.
`jamming_dev.launch`나 별도 mode/control selector는 함께 실행하지 않는다.

```bash
roslaunch asmc_planner autonomy_no_bridges_v2.launch highway_merge:=true
```

```bash
timeout 5s rostopic echo -n 1 /planning/control_selector/diagnostics
timeout 5s rostopic echo -n 1 /planning/camera_bbox_lane/diagnostics
timeout 5s rostopic echo -n 1 /planning/camera_bbox_lane_change/diagnostics
timeout 5s rostopic echo -n 1 /Ego_topic/velocity
```

`CameraBBoxLaneResult` MD5가 바뀌므로 기존 classifier/controller 프로세스는 재시작해야 한다.
검증 환경에서 ROS transport·GPU·MORAI 실행은 별도 확인해야 한다.
