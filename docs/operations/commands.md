# ASMC 실행 명령

> 2026-11-10 PR #141 병합 이후 main의 기본 음영 선택은 localization+LOCAL 복구입니다.
> 세 방식·속도·전환·콘솔은 [세 재밍 플래너 실행](jamming-comparison.md), 보존·아직 남은 재빌드는
> [대회 종료 기록](../records/repository/2026-11-10-postcompetition-preservation.md)을 따릅니다.

`asmc-ros-noetic`의 `/root/ws`는 실행할 checkout입니다. 현재 worktree를 유지하고
다른 checkout을 빌드한 setup과 섞지 않습니다. 인자는 `name:=value`이며
[실제 launch 인자/기본값](launch-reference.md)은 소스에서 자동 생성합니다.

운영은 한 컨테이너에서 ROS·bridge·인지·판제를 실행합니다. 세 방식과 기존 비교
컨테이너의 차이는 [세 재밍 플래너 비교 실행](jamming-comparison.md)을 따릅니다.
자동 시험은 별도 master와 공용 시뮬 lease를 사용합니다.

## 1. 개별 구성으로 전체 스택 실행

### Host와 빌드

```bash
export ASMC=$(git rev-parse --show-toplevel)
cd "$ASMC"
# GUI를 쓸 때만
xhost +local:docker
./scripts/docker_ros_up.sh up
docker exec -it asmc-ros-noetic bash
```

LiDAR·판제·수집은 outer workspace만 빌드합니다. 주행 중 container 재생성이나 빌드를 하지 않습니다. 구 패키지는 폐기했고 재현은 고정 revision의 별도 checkout을 사용합니다.

```bash
cd /root/ws
source /opt/ros/noetic/setup.bash
./scripts/build_ws.sh runtime -j2 -l2
source devel/setup.bash
```

기본 `runtime`은 운영 노드·메시지·native 모듈을 준비한다. 개발 도구·C++ 시험이 필요하면
같은 workspace에서 `./scripts/build_ws.sh full -j2 -l2`를 먼저 실행한다.
모드 전환에 clean은 필요하지 않으며, 기본 Release 설정과 cache 정책은 [빌드 안내](build.md)를 따른다.

각 터미널은 아래 outer 환경만 적용합니다.
```bash
cd /root/ws
source /opt/ros/noetic/setup.bash
source /root/ws/devel/setup.bash
```

PR #122의 터널 최적화·3뷰 fusion도 위 `/root/ws` 빌드에 포함됩니다.
개인 M1 overlay를 자동 source하는 과거 설정은 해제하고 같은 workspace로 통일합니다.
병합·설정 파일 변경만으로 기존 실행파일이나 실행 중인 노드가 갱신되지는 않습니다.
재빌드 후 다음 점검이 `source_matches: true`인지 확인합니다.

```bash
python3 /root/ws/src/asmc_lidar/tools/check_runtime_build.py
```

이 점검은 ROS 노드를 시작하지 않습니다. 실행 후에는 기존 LiDAR diagnostics의
`build_package_source_sha256`, `fusion_views=all`, `original_tunnel_pipeline=true`로 확인합니다.
[빌드·실행파일 확인과 재시작 주의](../../src/asmc_lidar/docs/three-view-fusion.md#빌드와-실제-실행파일-확인)를 따릅니다.

### 터미널별 실행

각 행은 별도 터미널에서 foreground로 실행합니다. 기본 실행 명령은 인자 없이 사용하고,
일부 모델 점검·복구에 필요한 옵션은 아래 별도 표에서 확인합니다. 이 구성과 아래 시나리오 통합 구성을 동시에 켜지 않습니다.

| 순서 | 실행 | 포함·주의 |
|---|---|---|
| A | `./scripts/udp_bridge.sh` | 허용 센서·기본 `/vehicle/state`, UDP 포트당 수신자 하나 |
| B | `roslaunch asmc_camera camera.launch` | 세 모델 상시 ACTIVE·발행, YOLO26L E3·TwinLite Large E8·ResNet18 기본; GUI 별도 |
| C | `roslaunch asmc_lidar lidar.launch` | 공용 local/map 추적·속도/운동·전달 필터; 카메라 모델 미실행 |
| D | `roslaunch asmc_planner autonomy_no_bridges_v2.launch` | **차량 제어 포함**, 새 공용 관측 직접 소비; GPS/IMU 위치와 Competition 종방향 실측 속도 조합, MPC/selector/재밍 어댑터 |

D의 통합 판제 실행은 `highway_merge:=true`가 기본입니다. 고속도로 기능을 끌 때만
`highway_merge:=false`를 지정합니다. 개별 `planner_v2.launch`는 기존 `false` 기본값을 유지합니다.
GPU가 다른 PC에서 만든 객체 엔진을 사용하던 경우, 추론 노드를 정지한 뒤
`bash tools/scen_runner/prepare_camera_engines.sh`로 현재 모델·GPU·TensorRT에 맞는
엔진을 생성합니다. 검증을 마친 로컬 객체 엔진은 직접 실행하는 `camera.launch`와
시나리오 실행기 모두에서 선택됩니다. [로컬 엔진 안내](local-lane-engine.md)를 참고하세요.
현재 main의 D는 `jamming_planner:=localization`과
`localization_local_recovery:=true`가 기본입니다. localization MPC 하나가 native 경로와 검증된 LOCAL 대체 경로를 추종합니다.
기존 방식 복귀는 `jamming_planner:=camera localization_local_recovery:=false`입니다.
[공유 master·중복 방지·연구 범위](planning-control.md#localization-음영-위치-추종-연구-옵션)를 먼저 확인합니다.
판제 대시보드는 D에 포함되어 별도 실행하지 않습니다. 실행·보존·검증 범위는
[통합 안내](../../src/asmc_planner/docs/main-integration-20261026.md)를 참고하세요.

`jamming_planner:=camera`를 명시하면 기존 camera BBox/family·LiDAR 기반 방식을 사용합니다.
현재 launch에는 `/perception/camera/objects`를 받는 BBox bridge와 분류/선택 경로가 포함됩니다.
이는 기본 localization의 검증 LOCAL 복구와 다른 방식이며, 선택한 제어기만 selector에 연결됩니다.
[재밍 입력·기억 정책](../../src/jamming_lane_boundary_adapter/docs/bbox-lane-change.md)을 참고하세요.

C/D는 기본 정본 조합입니다. source 선택 인자는 제거됐으며 [실행 기준선](../../src/asmc_lidar/docs/planning-runtime.md)을 따릅니다. 전체 미션·속도 정확도 인수는 후속입니다.

`roslaunch asmc_hd_map competition_map.launch`는 **선택 실행**입니다. 지도·미션 ROS 토픽과
현재 lane/context 또는 지도 RViz가 필요할 때 사용합니다. 현재 판제/Frenet과 LiDAR는 각 설정에
지정된 지도 파일을 직접 읽으므로 이 launch 없이도 지도 기반 주행·객체 필터가 작동합니다.
지도 토픽을 켜는 것만으로 판제의 파일 bundle이 자동 교체되지는 않습니다.
[역할과 소비 경계](../../src/asmc_lidar/docs/planning-runtime.md#hd맵-launch의-역할)를 참고하세요.

신호등의 단일 추론·발행자는 카메라 담당의 `asmc_camera`입니다. 판제는 결과만 소비하고
`traffic_light:=false` 인자는 제거되어 지정하지 않습니다. 중단/재개·주기 제어는 카메라의
`/perception/camera/traffic_light/set_mode`를 사용합니다. 판제 신호 규칙은 유지합니다. bridge `/vehicle/state`와
판제 `/planning/vehicle/state`는 다른 토픽·추정기이며 같은 토픽의 중복 producer는 금지합니다. 기본 D는 `/Ego_topic`의 허용된 종방향 속도만 `/planning/vehicle/gps_imu_state`의 GPS/IMU 위치와 결합합니다. 두 입력의 시각이 맞지 않거나 끊기면 상태를 무효로 처리합니다.

기본 카메라는 세 모델 모두 추론·발행합니다. 준비가 끝나기 전에 판제가 결과를 사용하지 않도록
상태와 새 결과의 validity/freshness를 확인합니다. [구간별 소비 안내](../handoffs/perception-to-planning.md)는
판제 사용 범위이며 생산자를 끄는 설정이 아닙니다. Lane 주행 소비는 음영 ODD에 한정하고 A126 진입 전에
유효 family를 확인합니다. A448은 접근 구간, 주변 차량 소비는 A153 끝/A451 진입까지입니다.

현재 차선 기본 모델은 Large v7 E8 TensorRT다. 사용자가 알려진 공급·freshness 제한을 확인한 뒤 전환을 승인했다.
[선정·실측 기록](../records/camera/2026-10-18-twinlite-large-deployment.md)를 확인한다.
공식 BDD100K Large의 추가학습 전 결과는 과거 [별도 실험 기록](../../src/asmc_camera/docs/bdd-pretrained-experiment.md)를
따릅니다. 이진 차선 시각화 실험이며 위 B의 운영 family를 대체하지 않습니다.

### 개별 디버그·시각화

일반 사용은 위 B의 `camera.launch`와 아래 `view.launch`를 각각 한 번 실행합니다.
두 launch 모두 `models:=all`이 기본이며, `camera.launch`는 `mode:=active`도 기본이므로
이 인자를 반복 지정할 필요가 없습니다. 추론과 시각화 선택은 서로 독립적입니다.

```bash
# B에서 이미 camera.launch를 실행했다면 추론을 또 시작하지 않음
roslaunch asmc_camera view.launch
# 별도 터미널: rqt는 한 개만 실행
rosrun rqt_image_view rqt_image_view --clear-config /perception/camera/objects/debug/mosaic/compressed
```

E8은16종입니다. pretrained 비교는 현재 배포 선택에서 제외했습니다. 입력은 YOLO/TwinLite 모두
전방544×960 B1·좌우480×640 B2, 목표15/15Hz·ResNet10Hz, confidence0.40·NMS0.50입니다.
Ego mask·보정·engine은 YAML/manifest가 소유합니다. 자동 PyTorch fallback은 없습니다.

같은 rqt에서 `/perception/camera/debug/mosaic/compressed`는 주행가능영역·차선 semantic,
`/perception/lane/debug/shared_family_bev/compressed`는 피팅된 family를 표시합니다.
`/perception/bev/debug/comparison/compressed`의 RGB·semantic은 같은 family 프레임의 IMU 지면을
재사용합니다. 별도 추론·피팅을 하지 않으며 `IMU shared`/`M0: 사유`로 적용 상태를 표시합니다.
Family 화면은 새 family 도착 즉시 갱신합니다. 신호등도 결과 도착 즉시 표시합니다.
큰 객체·semantic 합성과 RGB BEV 비교 화면은 최대10Hz이며, 추론/판제 발행 주기와 독립적입니다.
최신 결과 하나만 대기시키며 같은 source 결과 재표시를 신규 인지 공급으로 세지 않습니다.
정본 지면은 `cameras_v3.yaml`의 z=-0.36m이며 변경 후 카메라/투영 노드를 다시 시작해야 합니다.
Family는 원본 시각 IMU 자세·공동 폭 피팅을 기본 사용합니다. IMU/timing 근거가 부족하면
세 뷰 모두 고정 평면·폭 prior0.35로 처리하며 명목 거리는0.36m 그대로입니다.
[기본 전환과10분 검증](../records/camera/2026-10-18-joint-family-stabilization.md)을 확인합니다.
고정 평면 복구는 [IMU 비교·복구 안내](../records/camera/2026-10-18-joint-family-stabilization.md)를
따릅니다. [0.36m·표시 검증](../records/camera/2026-10-18-nominal-ground-and-arrival-view.md)을 함께 확인합니다.
`view.launch`는 시각화 worker만 시작하며, 추론 모델이나 rqt 창을 시작하지 않습니다.
일부 모델만 추론하거나 표시하는 `models` 옵션은 아래 선택 옵션 표에서 안내합니다.

`visualization.launch`는 `view.launch`와 구형 실행 경로가 공유하는 내부 시각화 구성입니다.
단독 기본 실행은 세 표시 플래그가 모두 false라 시각화 노드를 만들지 않습니다. 일반 사용자는
`view.launch`를 사용하며 둘을 동시에 실행하지 않습니다. 상세 legacy/raw 옵션은 재현·진단용입니다.

GUI 종료 후 사용한 process가 남지 않았는지
`rosnode list`, `pgrep -af rqt_image_view`로 확인하고 해당 터미널의 Ctrl+C로 정리합니다.
사용자/다른 작업의 프로세스를 일괄 종료하지 않습니다.

```bash
rostopic info /perception/lidar/planning_observations_map
rostopic info /ctrl_cmd
rostopic hz /image_jpeg/compressed
rostopic hz /velodyne_points
rostopic hz /planning/objects
rostopic hz /planning/vehicle/state
rostopic echo -n 1 /morai_sensor_bridge/diagnostics
```

웹 화면은 아래 기본 명령 중 필요한 것을 별도 터미널에서 실행하고 주소를 직접 엽니다.
판제 웹은 모드에 따라 map/local을 자동 선택하며 수동 선택도 가능합니다. 음영 패널에 실제 선택 제어기,
정지 사유, bbox 판정, family·실제 선택 경로·local LiDAR 참조 객체를 표시합니다.
GPS 무효 때 마지막 map 위치로 올리지 않으며 debug OFF에서도 typed 기하를 볼 수 있습니다.
local 객체를 현재 음영 제어기가 회피에 사용한다는 의미는 아닙니다.
private 메시지 갱신 후에는 위 전체 빌드를 수행하고 adapter·판제·웹을 함께 재시작합니다.
[표시·시간 계약과 후속 계획](../architecture/shadow-planning-refactoring.md)을 참고하세요.

| 화면 | 기본 실행 | 주소 |
|---|---|---|
| 판제 웹 콘솔 | 통합 `autonomy_no_bridges_v2.launch`에 포함; 별도 launch 불필요 | `http://127.0.0.1:8088/web/index_v2.html` |
| LiDAR 웹 시각화 | `roslaunch asmc_lidar view.launch` | `http://127.0.0.1:8091/` |

판제는 통합 launch에서 서버가 기본 ON이고 브라우저 자동 열기는 OFF입니다.
별도 dashboard launch 없이 터미널에 출력된 주소를 눌러 패널을 엽니다.

LiDAR `view.launch`는 모니터만 시작합니다. 기본 `debug:=false`에서도 발행 객체 수·진단 상태를
확인할 수 있습니다. 웹의 OBB·점군 그림과 필터 전후·ROI 단계를 보려면 처리 노드의
`debug` 옵션을 켭니다. 판제 주행에는 이 옵션이 필요하지 않습니다.
기존 `lidar.launch`가 있다면 두 번째 처리 노드를 시작하지 않습니다.
판제는 C의 `planning_observations_map`을 사용합니다. 상세는 [웹 모니터](../../src/asmc_lidar/docs/web-monitor.md)를 봅니다.
`asmc_lidar`를 고르는 별도 source 옵션은 없습니다. 기본 연결은
`/perception/lidar/planning_observations_map` (`asmc_msgs/LidarPlanningObservation`)
→ `planning_objects_adapter` → `/planning/objects`입니다. `tracks_map`은 다른 타입의
분석용 출력이므로 판제 입력으로 대신 지정하지 않습니다. 수집용 planner도 같은 기본 입력을 상속합니다.

판제 웹은 `http://127.0.0.1:8088/web/index_v2.html`, LiDAR 웹은 `http://127.0.0.1:8091/`입니다.
기본 포트·ROS 노드 이름이 서로 다르고 두 웹 모두 관측만 합니다. 하나씩 사용하려면 해당 웹 launch
터미널만 Ctrl+C로 종료하고 다른 웹을 실행합니다. 브라우저 탭 종료는 서버 종료가 아닙니다.
동일한 웹 launch를 중복 실행하지 않습니다. `autonomy_no_bridges_v2.launch`에 음영 어댑터가
포함돼 있으므로 별도 차선변경 제어 launch도 추가하지 않습니다.
LiDAR `debug:=true`는 중간 점군과 필터 전후 표현을 위한 샘플 발행입니다. 기본은 OFF이며
추적·속도·HD맵/높이 필터·판제 발행은 유지됩니다. 웹 체크박스는 표시만 바꾸고 실제 발행 필터를 변경하지 않습니다.

### 선택 옵션·수동 모드 제어

아래는 기본 실행에서 바꿀 필요가 있을 때만 사용하는 인자입니다. 일반 주행은 위 기본 명령을
그대로 사용합니다. 전체 목록은 [launch 인자](launch-reference.md)를 봅니다.
`camera.launch`의 `models`는 **추론할 모델**, `view.launch`의 `models`는 **표시할 결과**를 선택합니다.
예를 들어 `models:=objects`는 객체만, `models:=objects,lane`은 객체와 차선을 선택하며
`view.launch`에서 선택을 바꿔도 추론·발행 모델은 바뀌지 않습니다.
실행 선택을 바꿀 때는 해당 launch를 종료한 뒤 대체하며 같은 생산자나 시각화 worker를 중복 시작하지 않습니다.

| 실행 | 옵션·기본값 | 용도 |
|---|---|---|
| `asmc_camera camera.launch` | `models:=all` | 기본 객체·차선·신호등 모두 추론. 선택값은 `objects`, `lane`, `traffic_light`의 쉼표 조합 |
| `asmc_camera camera.launch` | `mode:=active` | 기본 상시 추론·발행. `standby`는 로드·예열 후 대기, `off`는 모델 미실행 |
| `asmc_camera camera.launch` | `detector:=e8` | 현재 선택은 E3·E8. 가중치 변경 시 재시작 |
| `asmc_camera camera.launch` | `confidence:=0.40`, `nms_iou:=0.50` | 현재 검증된 객체 검출 기본값 |
| `asmc_camera view.launch` | `models:=all` | 기본 세 모델의 결과 표시. 일부 화면만 선택해도 추론은 유지 |
| `asmc_lidar lidar.launch` | `fusion:=true` | 카메라 분류 연결. `false`는 LiDAR 단독 진단용 |
| `asmc_lidar lidar.launch` | `debug:=false` | 웹 OBB·점군 그림과 필터 전후 비교가 필요할 때 `true`; 주행에는 불필요 |
| `asmc_lidar lidar.launch` | `rviz:=false` | 웹만 쓸 때 그대로 유지 |
| `asmc_lidar lidar.launch` | `debug_rate_hz:=5.0`, `debug_max_points:=20000` | 화면 샘플 상한. 실제 검출·판제 발행 주기나 처리 점 수를 바꾸지 않음 |
| `asmc_planner dashboard.launch` | `dashboard_port:=8088` | 판제 웹, 기본 state/객체 토픽은 정본 |
| `asmc_lidar view.launch` | `dashboard_port:=8091`, `open_browser:=false` | LiDAR 웹. 처리 노드와 독립적인 관측용 실행; 주소는 직접 열기 |
| `asmc_hd_map competition_map.launch` | `rviz:=false` | 선택 지도 publisher. `true`면 지도 RViz도 실행; 기존 지도 launch를 대체하고 중복 시작하지 않음 |

명시적으로 대기 실행이 필요하면 `camera.launch`의 `mode`를 `standby`로 지정합니다. 다음은 그 경우에만 필요한 객체/신호등 복구 예입니다. 모드 요청자는 하나만 둡니다.

```bash
rosservice call /perception/camera/objects/set_mode '{mode: 1, target_hz: 0.0}'
rostopic echo /perception/camera/objects/status
# prepared=true 확인 후 echo 종료, ACTIVE
rosservice call /perception/camera/objects/set_mode '{mode: 2, target_hz: 0.0}'
rosservice call /perception/camera/traffic_light/set_mode '{mode: 1, target_hz: 0.0}'
rostopic echo /perception/camera/traffic_light/status
# 같은 generation 준비 확인 후
rosservice call /perception/camera/traffic_light/set_mode '{mode: 2, target_hz: 0.0}'
```

Lane도 기본 상시 ACTIVE입니다. 수동으로 대기시켰다면 다음 PREPARE → prepared/pipeline_ready
확인 → ACTIVE 순서로 복구합니다. 현재 구간 진입/이탈로 자동 중단하지 않습니다.

```bash
rosservice call /perception/camera/lane/set_mode '{mode: 1, target_hz: 0.0}'
rostopic echo /perception/camera/lane/status
# 준비 확인 후
rosservice call /perception/camera/lane/set_mode '{mode: 2, target_hz: 0.0}'
# 수동 중단이 필요한 경우에만 (기본 운용에서는 호출하지 않음)
rosservice call /perception/camera/lane/set_mode '{mode: 0, target_hz: 0.0}'
```

원본 좌표·빈/무효·세대/시간 규칙은 [카메라 계약](../contracts/camera-runtime-v1.md),
카메라 공용 관측과 기본3뷰 matching 계약은 [LiDAR 인계](../handoffs/camera-to-lidar.md),
판제 단독/통합 차이는 [판제 운영](planning-control.md)을 봅니다.

## 2. Container·build 명령

### ROS container

```bash
./scripts/docker_ros_up.sh <command>
```

| command | 기본값·의미 |
|---|---|
| `config [show]` | compose의 host network·port publish 여부 검사; `show`면 전체 출력 |
| `build` | RTX 40 기본 image build |
| `up` | RTX 40 기본 container 재생성·실행 |
| `rebuild` | cache 없이 build 후 재생성 |
| `build-rtx50` | 기본 image + RTX 50 파생 image build |
| `up-rtx50` | RTX 50 container 재생성·실행 |
| `rebuild-rtx50` | RTX 50 파생 image no-cache rebuild |
| `status` | `network_mode=host`, published port 없음 검사 |
| `check [port]` | Windows↔container loopback UDP. 기본 `39199` |
| `verify [seconds] [molitcomp03\|26r1]` | competition allowlist topic. 기본 `8 molitcomp03` |
| `verify-field [seconds] [molitcomp03]` | 주최판 Competition Status까지 필수. 기본 `8 molitcomp03` |
| `verify-gt [seconds] [molitcomp03\|26r1]` | local development oracle topic 검사 |
| `down` | ROS container 종료·제거 |

### Perception training container

```bash
./scripts/docker_perception_up.sh <config|build|up|down|rebuild>
```

`rebuild`는 no-cache build다. 이 컨테이너는 모델 학습과 학습 중 validation만 담당한다.
live 또는 bag replay ROS 추론·시각화는 모두 `asmc-ros-noetic`에서 실행한다. 학습
컨테이너는 host ROS master에 연결하지 않는다.

### ROS workspace build

```bash
docker exec -it asmc-ros-noetic bash
cd /root/ws
source /opt/ros/noetic/setup.bash
./scripts/build_ws.sh
source devel/setup.bash
```

## 3. UDP bridge 옵션

권장 진입점:

```bash
./scripts/udp_bridge.sh [options] [bridge.launch override:=value ...]
```

| option | 기본 | 허용값·의미 |
|---|---:|---|
| `--transport` | `local` | `local`, `lan2lan`; Ctrl 목적지 IP를 결정 |
| `--gt` | OFF | 일반 입력에 Object·Semantic·Instance LiDAR 추가; local 전용 |
| `--require-competition-status` | false | 9099 Competition → `/Ego_topic`을 필수로 검사 (MolitComp03 확인) |
| `-h`, `--help` | — | 도움말 |

추가로 덮어쓸 수 있는 `bridge.launch` 인자:

| launch arg | 기본값 | 설명 |
|---|---:|---|
| `runtime_profile` | `morai_udp` | 두 MORAI release가 공유하는 고정 runtime; wrapper override 금지 |
| `transport_profile` | `local` | 직접 launch용 `local`, `lan2lan`; wrapper에서는 `--transport` 사용 |
| `stream_profile` | `competition` | 내부 `competition`, `development`; wrapper에서는 `--gt`로 GT만 추가 |
| `timestamp_mode` | `receive` | `receive` 또는 source timestamp 실험용 `source` |
| `publish_map_tf` | `false` | raw GPS/IMU `map→base_link` TF; 권위 localization과 동시 금지 |
| `lidar_frame_mode` | `azimuth` | VLP16 frame assembly 모드; 현행은 `azimuth` |
| `lidar_rotation_rate` | `15.0` | v3/GT 조립 참고값. 실제 송신 주기는 MORAI 센서셋 설정; v2 비교는10.0 |
| `cmd_ip` | transport에서 결정 | local `127.0.0.1`, LAN2LAN `192.168.0.1` |
| `cmd_port` | `9093` | MORAI Ego Ctrl Cmd listen port |
| `competition_port` | `9798` | 로컬 Detail 진단 호환 port; LAN에서는 비활성 |
| `competition_status_required` | `false` | true면9099 `/Ego_topic` 입력을 요구; Detail9798로 대체하지 않음 |
| `enable_vehicle_state` | `true` | GPS/IMU 기반 `/vehicle/state` producer |

`runtime_profile`, `transport_profile`, `stream_profile`은 wrapper가 관리한다.
`transport_profile:=...`를 직접 넘기지 말고 `--transport`를 사용한다.
`--streams`와 `ASMC_STREAMS`는 폐기했다. GT 개발은 `./scripts/udp_bridge.sh --gt`를 사용하고,
MORAI UI에서 `config/morai/sensors/261027_ASMC_Local_SensorGT_25.S4.MolitComp03.json`을 로드한다. Intensity·Instance
LiDAR 모두15Hz로 설정되어 있다. 주기 변경은 JSON 재로드 후 적용되며 실제 수신률은 별도로 확인한다.

직접 launch도 가능하지만 local 디버그 외에는 wrapper가 더 안전하다.

```bash
roslaunch asmc_bridge bridge.launch
```

LAN2LAN을 직접 launch할 때는 `transport_profile` 만 바꾸면 Ctrl IP가 자동으로
바뀌지 않는다. 반드시 두 값을 함께 주거나 wrapper를 사용한다.

```bash
roslaunch asmc_bridge bridge.launch \
  transport_profile:=lan2lan \
  cmd_ip:=192.168.0.1
```


## 4. 팀 시나리오 통합 실행

[scen_runner](../../tools/scen_runner/README.md)는 #86의 전체 씬·단일구간·충돌/정체 재현 경로입니다.
개인 `local.yaml`을 작성한 뒤 같은 컨테이너의 네 터미널에서 실행합니다.
앞 절의 수동 인지/판제 실행과 동시에 켜지 않습니다.

```bash
# 터미널1
./tools/scen_runner/run_bridge.sh
# 터미널2
./tools/scen_runner/run_perception.sh
# 터미널3: 요청 대기 후 판제/MPC 시작·재시작
./tools/scen_runner/run_planning.sh
# 터미널4: 객체/환경 배치·차량 주행을 포함한 개발 검증
./tools/scen_runner/run_scenario.sh --mode scene --seed 42
```

구간만 시험할 때는 터미널4를 다음 중 하나로 대체합니다.

```bash
./tools/scen_runner/run_scenario.sh --mode section --section tunnel
./tools/scen_runner/run_scenario.sh --replay-browser
# 키보드/외부 제어기 검증에는 run_scenario 대신 사용
./tools/scen_runner/run_validation.sh --mode section --section dynamic
```

`--section`은 static/dynamic/roundabout/high_speed/tunnel입니다.
난수·날씨·주행 횟수·정체 시간은 공유 YAML, PC 주소/DLL 경로는 Git 제외 local.yaml이 소유합니다.
시나리오 자체의 gRPC/DLL/신호 고정은 개발 검증용입니다. 본선 입력 경로가 아닙니다.
현재 시나리오 perception 구성도 카메라 전체 ACTIVE입니다. 판제가 lane 결과를 음영 구간에서
소비하는 정책과 모델 활성화는 구분하며, full-stack 부하/모드 gate는 별도 확인합니다.

## 5. 수집·bag·오프라인 진입점

수집은 명시적으로 실행할 때만 시작합니다. 현재 문서 작업은 이 명령을 실행하지 않습니다.
Capture는 학습 GT, raw bag은 실제 비동기 지연/재현 측정이며 서로를 대체하지 않습니다.

```bash
# 수동 Capture 요청만: 차량 제어를 시작하지 않음
roslaunch data_collection capture_mode_collector.launch capture_hz:=1.0 max_captures:=100
# 자동 Capture 주행: 기존 주행 제어를 종료하고 승인된 장부/설정으로만 실행
roslaunch data_collection capture_route_collection.launch controller:=mpc target_kph:=20
# raw bag
roslaunch data_collection bag_recorder.launch profile:=full_raw scenario:=manual_route controller:=wheel
# live bridge를 종료한 별도 재현 환경; paused 상태로 시작
roslaunch data_collection bag_replay.launch bag_path:=/data/rosbags/raw/<run_id> required_profile:=camera_raw
```

[수집 README](../../src/data_collection/README.md)의 센서셋/파일 QA와 [bag 재현](rosbag-evaluation.md)의
시계·consumer·result recorder 준비 순서를 따릅니다. raw gRPC 자동수집과 운영 판제를 동시에 실행하지 않습니다.
개별 component·전체 인자는 [launch 목록](launch-reference.md)에서 찾습니다.

## 6. LAN2LAN 연습 사전 준비

처음 연결은 [팀원용 안내](lan2lan-teammate.md), 채점 프로그램 경계는
[두 PC 준비](lan2lan-rehearsal.md)를 따른다.

| 역할 | 전용 NIC IPv4 | 현재 MolitComp03 설정 |
|---|---|---|
| Windows MORAI PC | 192.168.0.1/24 | CompetitionInfoPublisher Host9088 → .10:9099 |
| 실행 PC | 192.168.0.10/24 | Windows/Hyper-V 수신 → bridge → /Ego_topic |

전용 NIC는 gateway/DNS를 비우고 MAC·ARP로 상대를 확인한다. 인터넷 NIC의 IP를 대신
바꾸지 않는다. 설정 도구는 미리보기 후 `-Apply`로 적용하며 기존 ASMC 규칙도 갱신한다.
시뮬레이터에는 release별 Network JSON과 `261027_ASMC_LAN2LAN_Sensor_<시뮬레이터버전>.json`을 사용한다.
이미 LAN 센서셋이 적용돼 있으면 Competition 수정 때문에 다시 로드하지 않는다.

Windows/Hyper-V의 팀 수신 UDP는 다음과 같다. 상대 `.1`, 로컬 `.10`, 전용 NIC로 범위를 제한한다.

```text
9099, 9092, 9291, 9293, 9295, 9297, 9299, 9301, 9303
```

실행 PC의 WSL은 mirrored, Docker Desktop은 host networking 활성화 및 컨테이너 host mode를
확인한다. 설정을 바꾼 경우 실행 PC의 작업을 저장하고 반영하며, 시뮬레이터 PC는 재시작하지 않는다.
TCP용 portproxy로 UDP 전달을 해결하지 않는다. 별도 채점 프로그램의 endpoint는 별도로 확인한다.

```bash
./scripts/udp_bridge.sh --transport lan2lan --require-competition-status
```

별도 컨테이너 터미널에서 실제 메시지를 검사한다.

```bash
python3 scripts/verify_morai_topics.py --profile competition --transport lan2lan --require-competition-status --duration 30
./scripts/docker_ros_up.sh verify-field 30 molitcomp03
```

`--require-competition-status`, `competition_status_required`, `verify-field`는 실제9099의
`/Ego_topic`을 검사한다. 과거9798 검사 의미를 정정했다. 9798 Detail은 LAN에서 수신·발행하지
않고 속도 fallback으로 사용하지 않는다. 기존 `competition_port=9798`은 로컬 진단 호환 인자다.
local·`--gt` 동작은 유지하며, 이번 MolitComp03 허용 판단을26.R1에 일반화하지 않는다.
새 기록을 할 때만 [rosbag 안내](rosbag-evaluation.md)의 `competition_raw`와 저장소 사전 점검을 따른다.


## 7. 종료

각 foreground `roslaunch`는 해당 terminal에서 `Ctrl+C`로 종료한다. container를 내릴 때:

```bash
# 학습 container를 별도로 실행했을 때만:
./scripts/docker_perception_up.sh down
./scripts/docker_ros_up.sh down
xhost -local:docker
```

주행·녹화 중에 `up`, `down`, `rebuild`로 container를 재생성하지 않는다.


## LiDAR 병행 비교 실행

일반 주행은 앞의 기본 명령과 `/perception/lidar` namespace를 사용합니다.
구 이관용 `/migration/lidar`는 운영 경로·별칭이 아니며 더 이상 현행 실행 예시로 사용하지 않습니다.
과거 bag·실험 기록의 토픽 이름은 당시 근거로 보존합니다. `/test/lidar`는 아래 격리 시험의 예시 이름일 뿐,
새 파이프라인이나 자동 실행 노드를 추가하는 기능이 아닙니다.
운영 LiDAR와 별도 시험 처리를 **동시에 비교할 때만** 아래처럼 시험 namespace를 명시합니다.
여기서는 출력 격리를 위한 인자가 필수입니다. 센서·카메라 입력은 기존 정본을 공유하며
판제는 운영 namespace를 계속 소비합니다.

```bash
roslaunch asmc_lidar lidar.launch namespace:=/test/lidar
```

이 시험 결과를 웹으로 볼 때는 기존 웹 launch를 종료한 뒤 같은 namespace로 실행합니다.
포트가 같은 웹 서버를 중복 실행하지 않습니다.

```bash
roslaunch asmc_lidar view.launch namespace:=/test/lidar
```

선택 옵션은 앞의 표와 동일합니다. `fusion:=false`는 카메라 연결 없는 비교,
`debug:=true`는 중간 점군·필터 전후 표시, `rviz:=true`는 해당 namespace의 처리된
OBB·ID·분류와 ego box 표시입니다. 원본 `/velodyne_points`와 처리 결과를 구분합니다.
리셋이 필요한 시험에서만 다음 서비스를 호출합니다. 대기·추적·pose 이력이 초기화되고
processing generation이 증가합니다.

```bash
rosservice call /test/lidar/lidar_pipeline/reset "{}"
```

보정·시간·터널 비교 등의 내부 옵션과 관측·진단 규칙은
[asmc_lidar README](../../src/asmc_lidar/README.md),
[웹 모니터](../../src/asmc_lidar/docs/web-monitor.md)를 따릅니다.


## LOCAL-1 실험 계산과 모니터

일반 autonomy 실행과 별도로 개발 중인 GPS 독립 경로다. 현재 운영 기본이나
## 구성 요소별 LOCAL 음영 주행 선택

2026-10-29 후속 요청의 현재 권장은 아래 [단일 컨테이너 실행](#local-배포-선택과-기본-방식-복귀--2026-10-29)이다.
공유 master는 이전 운용의 선택 사례이며 자동으로 다른 컨테이너의 master/bridge에 연결하지 않는다.

여러 host-network 컨테이너가 브릿지를 공유할 때 master는 기본 컨테이너 한 곳에서만 실행한다.
각 컨테이너는 **자기 `/root/ws/devel/setup.bash`**를 source한다. 다른 worktree의 setup을
겹쳐 읽지 않는다. `network=none` 컨테이너는 이 설정만으로 공유 master에 접속할 수 없다.

```bash
# 기본 컨테이너의 별도 터미널: 이미 roscore가 있으면 다시 시작하지 않는다.
roscore -p 11311
```

LOCAL worktree를 연결한 컨테이너에서 아래 환경과 실행 명령을 사용한다.
새 대화형 Bash에 같은 설정이 적용돼 있으면 export는 생략할 수 있다.
현재 LOCAL 컨테이너의 대화형 셸과 `/root/local-env.sh` 모두11311이다.
bag/회귀 실행기는 자식 프로세스에 시험용 master를 명시하며 사용자 셸을 바꾸지 않는다.

```bash
cd /root/ws
source /opt/ros/noetic/setup.bash
source /root/ws/devel/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11311
export ROS_HOSTNAME=127.0.0.1
unset ROS_IP
roslaunch asmc_planner autonomy_no_bridges_v2.launch jamming_planner:=local
```

이 명령은 **차량 제어를 포함**한다. bridge·camera·LiDAR는 공용 master에 각각 한 세트만
별도로 실행한다. 다른 컨테이너의 autonomy나 `local.launch`, 구 BBox follower와 동시에
실행하지 않는다. `jamming_planner`의 현재 기본값은 `localization`+LOCAL 복구이며, 독립 `local`을 선택하면:

- 기존 GPS/구간 mode 판단과 selector를 재사용한다. 일반 구간은 기존 NORMAL MPC,
  음영구간은 LOCAL planner/MPC가 담당하고, `/ctrl_cmd` 생산자는 selector 하나다.
- 구 BBox classifier/follower를 시작하지 않는다. LOCAL MPC는 NORMAL 중 계산하지 않는다.
  경로는 기존 음영 진입 지점30m 전부터 준비한다. 같은 session·설정·local frame이면
  준비 입력과 경로를 진입 때 유지하며, 센서 초기화·세대 변경 때만 폐기한다.
- 기존 카메라 결과에서 IMU 정합 노면 probability/coverage를 만드는 입력도 시작한다.
  추가 모델 추론은 없다. 같은 `semantic_space_trial`을 이미 사용 중이면
  `local_semantic_input:=false`를 지정한다.
- `jamming_speed_kph`의 LOCAL 기본 상한은20이며 순항 목표는15km/h다.
  동일 조건 비교에는 `jamming_speed_kph:=10`을 사용한다. 관측·곡률·복구
  제한은 그대로이며 속도/완주 수락을 보장하는 옵션이 아니다.
- 웹은 동일한 `http://localhost:8088/web/index_v2.html`이다. LOCAL 경로·노면·family·
  OBB와 제어 trace를 표시한다. 별도 dashboard 서버를 켜지 않는다.

판제 알고리즘 기본값 승격이 아니라 명시적 실행 선택이다.
현재 이 브랜치의 LOCAL YAML은 노면 reference 복구·짧은 기하 운동 예측·안쪽 최소
여유30cm·쾌적 감속2m/s²·controlled brake3m/s² 후보를 사용한다. 정적 asset 기억은
OFF이며, CAN 조향 시작상태 후보는 별도 시험 profile에서만 켠다.
현재 선택한 LOCAL profile은 최신 rigid 기하의 비영점 나이 외곽 후보와 실제 좁은 sweep의
5km/h maneuver 상한을 사용한다. 이는 YOLO 검출 감속이나 물체의 정적 확정이 아니다.
장애물 없는 LOCAL 상한10km/h, 사용자 요청 회복 목표5km/h를 사용한다. 물리 검사는 더 낮출 수 있다. 6m 복구/경로 유지/단계적5선은
별도 실험 설정이며 사용자 기본 실행에 자동 활성화하지 않는다.
[LOCAL-2H 1차 비교·가용 범위](../records/planning/2026-10-29-local-first-comparison.md)와
[LOCAL-2G 당시 검증](../records/planning/2026-10-29-local-obstacle-campaign.md)을
확인한다. 거리 도달과 공급·충돌 수락은 구분하며, 늦은 횡단·작은 동물 사례까지
해결된 운영 정본으로 표시하지 않는다. 기본 camera 선택과 LOCAL10km/h 상한은 유지한다.

## 자동 LOCAL 시나리오의 실시간 대시보드

이 worktree의 `tools/scen_runner/run_local_trial.py`는 통합(`--integrated`)·LOCAL 단독
시험 모두 관측 서버를 자동으로 시작한다. 시험 중에는 다음 주소만 열거나 새로고침한다.
별도 roscore·autonomy 실행이나 master 포트 입력은 필요 없다.

**http://localhost:8088/web/index_v2.html**

기본 `자동 NORMAL/map · JAMMING/local` 보기는 통합 전환을 따라간다. LOCAL 후보·선택
경로·노면·family·fusion·정지 사유·MPC를 집중 확인하려면 **LOCAL 경로·노면·fusion**을
선택한다. 상단에 branch/revision(+미커밋 여부)·run·실제 master·준비/주행/종료·연결
상태가 표시된다. run 변경·시험 종료·heartbeat 만료 때 과거 관측/경로를 비운다.
원본 노면은 기존 source-exact API로 받으며 새 추론·영상 인코딩을 추가하지 않는다.

8088이 다른 실행에 점유돼 있으면 해당 실행을 유지하고18088부터 빈 포트를 선택한다.
정확한 주소는 시험 `run.log`의 `Live trial dashboard`와 `monitor.json`에서 확인한다.
이후 같은 worktree의 시험은 그 주소를 재사용한다. 실제 ROS 관측 서버는 시험 소유
master에만 연결되고 종료 때 정리한다. 작은 HTTP 입구만 다음 시험/종료 안내를 위해
남는다. 이후 같은 worktree에서 수동 dashboard를 시작하면 **종료된 관측 입구만**
자동 반환한다. 진행 중인 시험이나 다른 worktree의 포트는 반환하지 않는다.

`--no-dashboard`는 관측 비용 OFF 대조용이다. `resources.jsonl`에 관측 프로세스의
별도 RSS/CPU 표본, `monitor.json`에 실제 주소·master·identity·HTTP 수량/시간을 남긴다.
웹을 여는 부하와 계획/제어 지연을 섞어서 성능 향상으로 보고하지 않는다.

## LOCAL 실험 계산과 모니터

아래는 통합 주행 대신 계산만 보는 독립 preview 방법이다. 현재 운영 기본이나
전 터널 수락 구성이 아니다. [검증 상태·제한](../records/planning/2026-10-28-local-planner.md)을 먼저 확인한다.
bridge·camera·LiDAR가 한 세트 실행 중인 outer workspace에서 다음을 사용한다.

```bash
cd /root/ws
source devel/setup.bash
roslaunch /root/ws/tools/scen_runner/launch/semantic_space.launch
```

semantic 실험 launch는 저장소 tools의 launch 파일을 직접 지정한다.
별도 nested workspace를 source하지 않는다.
이미 `/experimental/shadow/semantic_space_trial`이 실행 중이면 중복 시작하지 않는다.
다른 터미널에서 계산과 MPC preview를 켠다. **차량 제어는 기본 OFF**다.

```bash
roslaunch asmc_planner local.launch
```

기존 판제 웹 서버를 재사용한다. `http://localhost:8088/web/index_v2.html`에서
자동 보기 또는 `LOCAL-1 경로·노면·fusion`을 선택한다. 추론 노면, family·본차로,
후보, 선택 경로, 차체 sweep, local fusion OBB와 원본 YOLO 관측을 볼 수 있다.
NORMAL의 map 궤적과 local 궤적을 같은 좌표처럼 섞지 않는다. 재빌드 후 실행 중인
웹 서버도 재시작하고 브라우저를 새로고침한다. 서버를 이중 실행하지 않는다.

| 내부 실험 인자 | 기본·용도 |
|---|---|
| `enable_control` | false. true는 기존 음영 제어 생산자를 제외한 소유권 통제시험에서만 사용 |
| `enable_control` | false. 독립 제어 시험 또는 위 통합 launch 내부에서만 true. 별도 autonomy와 중복 시작 금지 |
| `speed_cap_kph` |25. 경로·관측·회복 제한이 더 낮으면 그 이유와 실제 상한 표시 |
| `recovery` |true. 신선한 semantic의 저속3km/h 회복 시험; 충돌 없음의 증명 아님 |
| `near_blind_assist` |false. S3 미관측 보조의 명시적 비교용, 켤 때≤5km/h 시험만 |
| `run_id`, `journal_path` |새 등록 run과 새 판단 파일. preview 기본은 파일 쓰기 없음 |

폐루프 반복시험은 `tools/scen_runner/run_local_trial.py`가 simulator fixture·selector·
local 제어의 소유권을 함께 관리한다. 실행 중인 정상 판제를 두고
`enable_control:=true`만 추가하지 않는다. 저장 등록과 실제 actuation·native
접촉·원복 결과를 기록한다. bag 비교는 `replay_local.py`, 본차로/급제동 평가는
`scen_runner.records.analyze_local_following`, 시각/자세 대조는
`scen_runner.records.audit_local_inputs`를 사용한다.

LOCAL-2B는 [경로·속도·실패 검수](../records/planning/2026-10-28-local-path-stability.md)에
현재 결과를 기록한다. `http://localhost:8000/`의 **LOCAL-2B 주행·실패 검수** 바로가기와
**로컬 경로·주행·실패 검수** 분류에서 시험을 선택한다. 주행 그래프와 각 실패의 정확한
RGB/semantic source·선택/탈락 경로를 연결한다. 실시간 화면에는 선택 길이/탐색 한계,
본차로 근거 끝, 과거 노면의 나이도 표시한다. 청회색은 새 추론이 아닌 과거 실제 노면이다.

`src/asmc_planner/config/local_planner.yaml`의 `causal_road_memory`는 근거리 과거 노면
보조의 내부 비교 스위치다. planner/controller의 `acceleration_limit_mps2`는 둘 다1.0으로
맞춘 local 전용 설정이다. 실행 목표25와 실제 허용속도는 다르며 관측 범위와 정지거리
때문에 낮아질 수 있다. NORMAL 설정이나 기존 jamming follower에는 적용되지 않는다.

실험 목록 일부만 갱신할 때는 `build_experiment_gallery_hub.py --artifacts <root> --group <run-id>`를
사용한다. 지정하지 않은 실험과 기존 번호를 보존한다. 전체 재탐색은 `--group` 없이 실행한다.


## LOCAL 배포 선택과 기본 방식 복귀 · 2026-10-29

현재 권장은 **한 컨테이너가 ROS master·bridge·카메라·LiDAR·판제·두 웹을 모두 소유**하는 방식이다.
main 포트 ROS11311·판제8088·LiDAR8091과 기존 bridge UDP를 그대로 쓴다. host network는
컨테이너별로 포트를 분리하지 않으므로 다른 브랜치의 전체 실행은 종료한 뒤 교대한다.
다른 실행이 있으면 시작을 거부하며 임의로 종료하거나 다른 master에 붙지 않는다.

컨테이너 내부 `/root/ws`에서 기본 카메라 재밍 방식:

```bash
./scripts/drive.sh
```

LOCAL 방식:

```bash
./scripts/drive.sh jamming_planner:=local
```

실행기는 설정을 펼쳐 중복 노드·TCP/UDP 수신 포트를 확인하고, 공유 simulator lease와11311을
소유한 뒤 `asmc_planner/autonomy.launch`를 시작한다. 센서 포트 목록을 별도로 복제하지 않는다.
종료는 Ctrl+C다. 자기 ROS 노드가 종료된 뒤 자기 master와 lease를 반납한다.
기본으로 복귀할 때 현재 실행을 종료하고 옵션 없는 명령을 쓴다. roscore·bridge·인지·웹을
별도로 추가 실행하지 않는다. 시작 전 확인만 하려면 `./scripts/drive.sh --check`를 사용한다.

기존 구성 요소별 실행도 유지한다. 이때 같은 컨테이너에11311 master와 bridge를 한 번 실행하고,
다음 명령은 각각 별도 터미널에서 실행한다. 전체 실행기와 구성 요소 실행을 겹치지 않는다.

```bash
roslaunch asmc_camera camera.launch
roslaunch asmc_lidar lidar.launch
roslaunch asmc_planner autonomy_no_bridges_v2.launch jamming_planner:=local
```

LOCAL 목표 상한10km/h, 저속 회복 목표5km/h다. 정지하지 않으려면 검증 가능한 대체 경로가
필요하며 입력 무효·실제 충돌·물리적으로 불가능한 곡률에 속도만5로 덮어쓰지 않는다.
기본 재밍 방식은 camera로 유지한다. LOCAL에서만 새 planner/MPC/노면 입력을 시작한다.

판제: <http://localhost:8088/web/index_v2.html>. 자동 보기 또는 `LOCAL 경로·노면·fusion`.
LiDAR: <http://localhost:8091/>. 전체 실행에는 두 웹이 포함되고 GUI를 자동으로 열지 않는다.
LiDAR 단계별 상세 점군이 필요할 때만 전체 실행에 `lidar_debug:=true`를 추가한다.
기본 debug OFF에서도 판제 객체 입력과 웹 객체 표시는 유지된다.
전체 실행의 branch·revision·session/master를 판제 웹 상단에 표시한다. 관측 연결이 끊기면
과거 경로를 비우며, 상세 유효성·노면 age·MPC 실패 사유는 기존 패널에서 확인한다.

`jamming_planner`는 camera/local/localization을 지원한다. 현재 기본은 localization+LOCAL 복구이며 상세 검증 범위는 [06:50 체크포인트](../records/planning/2026-10-30-jamming-0650.md)를 따른다.
향후 같은 상호 배타 include와 selector 슬롯에 연결해 명령 소유권·watchdog·모드 인계를 재사용한다.
[비교·남은 실패](../records/planning/2026-10-29-local-first-comparison.md)와
[배포 검증](../records/planning/2026-10-29-local-deployment.md)을 함께 확인한다.

### 2026-10-30 비교 컨테이너 실행

`asmc-jamming-compare`의 기존 master11373·브릿지·인지·콘솔을 쓰는 경우:

```bash
roslaunch asmc_planner autonomy_no_bridges_v2.launch dashboard:=false
```

판제: http://localhost:8089/web/index_v2.html · LiDAR: http://localhost:8097/ .
새 단독 실행의 일반 웹 기본값8088/8091과 구분한다. 이미 실행 중인 autonomy와 중복 실행하지 않는다.
기본 localization 목표15km/h, A451부터 준비·A448에서 인계. 무정지 전체 수락은 검증 기록을 확인한다.
