# asmc_lidar 전환과 재밍 연결 — 2026-10-26

**현재 상태:** 두 초기 bag과 격리 ROS 시험에 더해, [04:18 실제 주행 bag](../records/lidar/2026-10-26-full-drive-bag-validation.md)에서 재밍 진입·탈출·일반 주행 복귀 1회를 확인했다. 차선 지연 경고와 재진입·다른 조건 검증은 남아 있다. 기존 `src/lidar+sensorfusion`은 **삭제·수정하지 않는다**. Main을 실행하는 기존 명령도
복구용으로 유지한다. 새 경로는 `asmc_lidar → typed 관측 → planning_inputs → 기존 판제`다.
Main의 JSON 진단 토픽을 새 경로의 입력으로 사용하지 않는다.

## 실행

ROS Noetic 컨테이너의 `/root/ws`에서 상위 workspace를 빌드한다.

```bash
source /opt/ros/noetic/setup.bash
cd /root/ws
catkin_make -DCMAKE_BUILD_TYPE=Release -DASMC_LIDAR_PIPELINE_REQUIRED=ON \
  -DCATKIN_WHITELIST_PACKAGES= -DCATKIN_BLACKLIST_PACKAGES= -j2
source devel/setup.bash
```

이 PC의 기존 build에는 `asmc_lidar`가 빠진 whitelist가 남아 있었다. 위 명령은 이
캐시를 해제해 새 패키지가 실제로 빌드되게 한다. 기존 패키지를 삭제하는 설정이 아니다.

브리지와 `asmc_camera/camera.launch`는 기존 방법으로 실행한다. 이전 Main·판제 실행을
종료한 뒤 다음 명령 하나로 새 LiDAR와 판제를 실행한다. 이미 실행한 `lidar.launch`와
중복 실행하지 않는다. 이 명령은 제어를 포함한다.

```bash
roslaunch asmc_planner autonomy_asmc_lidar.launch
```

LiDAR만 확인하려면 `roslaunch asmc_lidar lidar.launch planning_output:=true`를 사용한다.
관측은 `/migration/lidar/planning_observations_map`, 최종 판제 입력은 `/planning/objects`,
어댑터 상태는 `/asmc_lidar_planning_adapter/diagnostics`다.
`scen_runner`는 `config/stack.yaml`의 `lidar_backend` 한 설정을 perception/planning에 함께 전달한다.
저장소 기본은 `asmc_lidar`이며, `main`으로 기존 경로를 복구할 수 있다.
기존 사용자별 stack 설정에 이 키가 없으면 이전 `main`을 유지한다.

## 재밍 수정

두 bag 끝에서 GPS가 0/0/0으로 바뀌고 정상 주행 입력이 무효화되는 것은 확인됐다.
당시 로그에는 재밍 follower가 켜진 뒤 `bbox_timestamp_expired`가 반복됐다.
프레임 브리지는 발행자가 없는 `/planning/camera_bbox_lane/yolo`를 기다리고 있었다.
이제 카메라가 실제 발행하는 `/perception/camera/objects`의 전방 결과를 직접 받는다.
원본 영상과 시각이 정확히 같은 결과만 사용하고, 정상 빈 검출도 전달한다.
GPS 무효를 정상으로 꾸미거나 장애물·입력 오류 STOP을 해제하지 않는다.

## 검증과 한계

검증 결과는 [실행 기록](../records/lidar/2026-10-26-cutover-validation.md)에 남긴다.
원본 두 bag은 읽기 전용으로 사용하고 파생 출력은 `$ASMC_DATA/derived/lidar-migration-20261026/`에 둔다.
두 bag에는 GPS 복구 장면이 없으므로 실제 출구 통과 검증으로 설명할 수 없다.
재진입·복구·중복 제어 방지는 격리 ROS 시험으로 검증한다.
기록된 bag에는 새 카메라 observation/status가 없어 새 LiDAR replay는 fusion 없이 수행한다.
최종 실제 주행에서는 정상 주행, 정지/이동 장애물, 재밍 진입·탈출·재진입을 확인해야 한다.

새 경로는 HD맵·높이 ROI의 asmc_lidar 설정을 유지한다. Main의 최종 객체 수와 같다는
수락 기준을 사용하지 않는다. 새 객체의 속도를 아직 모르면 명시적인 `velocity_unavailable` 표시와 UNKNOWN 상태로
위치·OBB를 유지한다. 임의의 측정 속도 0이나 빈 도로로 바꾸지 않는다.
이 private 메시지 필드 추가로 상위 workspace 전체 재빌드가 필요하다.
지도·기하 정보 자체가 무효이거나 입력이 끊기면 기존 제어기가 정지한다. 전진·평면 주행을 넘는 속도 정확도 수락은 별도다.

## 복구

새 launch 전체를 종료한다. 기존 Main을 실행하고
`roslaunch asmc_planner autonomy_no_bridges_v2.launch`를 사용한다.
scen_runner는 같은 설정 파일에서 `lidar_backend: main`으로 바꿔 perception과 planning을 함께 재시작한다.
같은 센서/판제 출력에 두 생산자를 동시에 실행하지 않는다.
