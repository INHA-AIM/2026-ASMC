# Planning/Control 실행·검증

> 이 비교 브랜치의 세 방식 선택·포트·master·A448 정책은 [세 재밍 플래너 비교 실행](jamming-comparison.md)이 현재 정본입니다. 아래의 과거 개별 브랜치 안내와 구분합니다.

현재 패키지는 `asmc_planner`, `asmc_controller`, `planning_inputs`입니다.
bridge·카메라·LiDAR는 별도 실행합니다. LiDAR 정본은 `asmc_lidar`이며 구 Main은 고정 revision에서만 재현합니다. 전체 시작 순서는 [실행 명령](commands.md),
실제 인자/기본값은 [launch 목록](launch-reference.md)이 정본입니다.

## 빌드와 통합 실행

`asmc-ros-noetic`의 저장소 루트에서 실행합니다. 현행 실행은 outer 빌드와 setup만 사용합니다.

```bash
cd /root/ws
source /opt/ros/noetic/setup.bash
./scripts/build_ws.sh runtime -j2 -l2
source devel/setup.bash
# bridge·카메라·asmc_lidar를 별도 준비한 뒤
roslaunch asmc_planner autonomy_no_bridges_v2.launch
```

이 명령은 차량 제어를 시작합니다. 신호등 추론·발행은 카메라 담당의 `asmc_camera`만 소유하며,
판제 launch는 별도 ResNet을 시작하지 않습니다. `traffic_light:=false` 등 폐기 인자는 삭제합니다.
실행 중 중단/재개·주기는 카메라의 `set_mode`에서 제어합니다. 판제 신호 규칙·stale/invalid 처리와
구간별 소비 책임은 유지합니다. [소비·제어 안내](../handoffs/perception-to-planning.md)를 따릅니다.

통합 `autonomy_no_bridges_v2.launch`의 `highway_merge` 기본값은 `true`다. 명시적
`highway_merge:=false`로 고속도로 기능을 끌 수 있다. 개별 `planner_v2.launch` 기본값은
기존 `false`를 유지한다. 준비용 코어들은 검증된 정적 지도·경로를 재사용하며, 실행 상태·속도 이력은
각 코어에 남긴다. 원본 입력 변경·누락은 다시 로딩/검증하고 잘못된 입력은 시작을 거부한다.
[초기화 실측과 검증 한계](../records/repository/2026-10-29-planner-startup-ci.md)를 참고한다.

통합 실행은 대시보드 서버를 기본 실행하고 브라우저는 자동으로 열지 않는다
(`dashboard:=true`, `open_dashboard:=false`). 별도 `dashboard.launch`는 실행하지 않고,
터미널에 표시된 `http://localhost:8088/web/index_v2.html`을 눌러 판제 패널을 연다.
서버를 끄려면 `dashboard:=false`를 지정한다.
같은 HTTP 포트로 중복 실행하면 새 대시보드는 ROS 노드 등록 전에 종료해 기존 콘솔을 보존한다.
`dashboard_port` 변경은 포트 설정이며, 같은 ROS 노드 이름의 다중 콘솔 운영을 보장하지 않는다.

최신 main PR #109의 기본 `state_source:=competition`을 유지합니다. GPS/IMU의 map pose에
`competition_speed_state.yaml`이 지정한 `/Ego_topic` 종방향 속도를 결합하며, 센서 이름을 보고
다른 상태 입력으로 자동 전환하지 않습니다. `state_source:=gps_imu`는 GPS/IMU 추정만 사용하고
`external`은 외부 `/planning/vehicle/state` producer를 요구합니다.
객체 입력은 `/perception/lidar/planning_observations_map`의 공용 추적 관측으로 단일화했습니다. 속도/4단계 운동·원본 OBB·세대/시각/상태를 검사하고 인지 필터·classifier·JSON join을 다시 실행하지 않습니다. [연결 의미와 복구](../../src/asmc_lidar/docs/planning-runtime.md)를 따릅니다.

## Localization 음영 위치 추종 연구 옵션

`jamming_planner:=camera`가 기본값이며 최신 main의 팀원 재밍 처리를 유지합니다.
`jamming_planner:=localization`은 측위 기반 기존 Frenet 회피·추종 비교안을 선택합니다.
이 옵션에서는 정적 회피·관측 운동 대응·음영 차선 예외가 켜지고 속도 상한은 15km/h입니다.
일반 main의 속도 정책은 바꾸지 않습니다. 20km/h·회전 정차차량·모든 보행자 장면은
수락하지 않았습니다. [현재 체크포인트와 비교 조건](../records/planning/2026-10-29-localization-comparison.md)을 먼저 읽습니다.

재밍 방식 선택 인자는 `jamming_planner` 하나만 사용합니다. 호환 별칭은 제공하지 않습니다.
LOCAL PR #138의 `jamming_planner:=camera/local` 명칭과 맞춰 localization을 추가한 것입니다.

통합 기준 `fad73f82`에는 별도 LOCAL 선택이 아직 없습니다. 별도 작업트리의 개발 중인
LOCAL 코드를 가져오지 않았으며 `jamming_planner:=local` 같은 실행 별칭도 만들지 않았습니다.
이 PR의 내부 `src/local`은 Localization이 재사용하는 기반 구성요소로, 진행 중인 LOCAL 배포와 구분합니다.

| 구간 | 계산·제어 소유자 |
|---|---|
| NORMAL | 기존 main planner와 NORMAL MPC. Localization는 odometry/anchor를 준비하고 경로·MPC 계산은 대기 |
| JAMMING | Localization 위치 추종과 연구 MPC. NORMAL MPC 계산·발행 중지 |
| NORMAL 복귀 | 신선하고 검증된 GPS anchor + 새 정상 경로 2개 확인 후 selector 전환 |

두 옵션 모두 main의 `GpsModeSource`가 `/planning/mode`를 발행하고 selector 하나가
`/ctrl_cmd`를 발행합니다. Localization 선택 시 기존 `camera_bbox_*` 세 노드는 실행되지 않습니다.
검증되지 않은 anchor는 기본 경로의 4초 복귀 timeout으로 우회하지 않습니다.
LiDAR의 실제 GPS validity·터널 벽/천장 전처리와 카메라 추론은 변경하지 않습니다.

수동 실행은 컨테이너별 독립 master와 브릿지를 사용합니다. 설치된 대화형 profile은
자기 source/devel만 읽고, 노드나 master를 자동 시작하지 않습니다.

| 컨테이너 | 수동 master | 판제 | LiDAR |
|---|---|---|---|
| asmc-ros-noetic | 11311 | 8088 | 8091 |
| asmc-local2c-20261028 | 11348 | 8090 | 8092 |
| asmc-localization-compare | 11372 | 8089 | 8097 |

새 터미널에서 `docker exec -it asmc-localization-compare bash`로 접속한 뒤,
아래 명령을 각각 다른 터미널에서 실행합니다. 별도 source는 필요 없습니다.

```bash
roscore
roslaunch asmc_bridge bridge.launch
roslaunch asmc_camera camera.launch
roslaunch asmc_lidar lidar.launch debug:=true
roslaunch asmc_lidar view.launch
roslaunch asmc_planner autonomy_no_bridges_v2.launch jamming_planner:=localization
```

profile의 `roscore` 함수는 해당 master 포트를 선택합니다. `roslaunch` 함수는 판제·LiDAR
콘솔에 표의 포트를 적용하며, 명시한 `dashboard_port`가 있으면 그 값을 사용합니다.
사용 중인 HTTP 포트는 ROS 노드 등록 전에 거절합니다. 실제 MORAI UDP 브릿지는 한
컨테이너씩 교대하며, 공용 physical lease와 UDP 소켓 검사를 거칩니다. 교대 전 기존
자율주행과 브릿지를 원래 터미널에서 종료합니다. 다른 worktree의 코드를 바꾸지 않습니다.
[설치·복구 범위](../../tools/runtime/README.md)를 따릅니다.

자동 시험은 기존 `/root/localization-env.sh`를 명시해 private master11371을 사용합니다.
수동11372와 구분하며, 자동 bag 평가에서는 추가 독립 master를 사용합니다.
호스트 저장 관리용 `.storage/env.sh`는 컨테이너에서 source하지 않습니다.

GPS·IMU·Ego와 자체 LiDAR의 `/perception/lidar/tracks_local`,
`planning_observations_map`이 필요합니다. LiDAR 기본 융합과 실제 GPS 기반
터널 벽·천장·지면 처리는 유지하며, 추정 위치를 GPS 입력에 되먹이지 않습니다.
적분기는 하나이며 Localization의 상대 yaw는 유효한 원본 IMU quaternion의 변화로
측정합니다. 일반 LOCAL의 기본 gyro-rate 적분은 유지합니다. `shadow_motion_yaw_source:=rate`는
이전 연구 방식과 비교할 때만 지정합니다. 준비 구간에서만 다음 경로와 MPC를 잠시 예열합니다.
MolitComp03 연구 조합은 곡선 이동의 실측 오차를 속도 의존 항으로 보정합니다.
`shadow_motion_turn_gain_m2ps:=1.54`를 측정 속도(최소 1m/s)로 나눈 계수를
인접 IMU 방위 변화에 적용합니다. 속도 변화만으로 위치를 이동시키지 않으며,
물리적인 GPS 장착점이 바뀐다는 뜻도 아닙니다. native 실제 위치는 보정값 학습에 사용하지 않았습니다.
보정값은 `config/localization_motion_calibration.json`의 GNSS·IMU·속도 기록으로 재현합니다.
5·10km/h 정상 GPS 기록으로 선택했습니다. 현재 조합은 별도 실제 15km/h 3회와
±20cm·±2° 시작 오프셋 시험을 통과했습니다. 이는 장애물 없는 MolitComp03 조건이며,
반복 시험의 주행 화면은 800×666으로 줄였다가 회차마다 복원했습니다. 센서 영상은
앞 1280×720, 나머지 세 대 640×480 그대로인지 출발 전에 확인했습니다.
옛 고정 보정은 gain을 0으로 하고 `shadow_motion_reference_offset_m:=1.4`를 지정하면
재현됩니다. gyro-rate 비교도 gain·고정 보정 여부를 함께 명시합니다.
26.R1로의 일반화는 검증 전입니다. 독립 LOCAL의 기본 gain·고정 보정값은 모두 0입니다.
이전 반복에서 원본 IMU 시각과 커널 수신 모두 약 207ms 끊긴 회차는 실패로 보존합니다.
수신 지연으로 오인해 브릿지를 바꾸거나 150ms motion epoch·250ms watchdog을
완화하지 않았습니다. 작은 주행 화면이 모든 송신 공백을 없앤다는 뜻은 아닙니다.
전환 세대·입력·경로 시각이 맞는 후보 명령을 받은 뒤 selector가 제어를 넘깁니다.
정상 인계에는 일괄 정지 명령을 삽입하지 않으며, 무효 입력·준비 실패·250ms watchdog은 유지합니다.

초기 anchor는 최소 1초·10개 정렬 GPS를 요구합니다. 실측 5Hz 입력에서는 약 1.8초 이상입니다.
GPS 장착 높이 1.25m까지 정렬된 IMU roll/pitch/yaw로 보정합니다. 복구 때는 DR을
유지하며 1초간 정렬한 GPS 후보를 검증하고, 후보 오차가 25cm·2° 이내일 때만
1초 동안 연결합니다. 50cm 이상 원시 점프·불안정한 후보는 정지합니다. 원시 표본
오차와 검증 후보 오차는 trace의 `raw_gps_innovation_m`, `innovation_m`으로 구분합니다.
GPS가 계속 들어오는 동안에도 25cm를 넘는 단일 표본은 같은 후보 검증을 거칩니다.
직전 검증 위치를 유지하며, 1초 후보가 계속 25cm를 넘으면 정지합니다. 검증 중인
표본으로 120초·250m DR 한계를 갱신하지 않습니다. `recovery_verify`는 소실 전
잡음 검사에도 나타날 수 있습니다.
100ms 이내 역순 센서 표본은 사용하지 않고 폐기합니다. 마지막 유효 입력의 시각이나
freshness를 갱신하지 않으며, 큰 시각 역행·입력 단절·250ms 제어 watchdog은 유지합니다.
연구 MPC는 실제 속도와 가속도를 기존 가속 상한에 반영해 보호 정지 후 재출발의
속도 초과를 억제합니다. 이 설정들은 Localization에만 켜며 main·독립 LOCAL 기본값은 유지합니다.
trace는 `/planning/localization/trace`, 제어 진단은 `/planning/localization/control_trace`입니다.
파일 기록은 기본 꺼져 있으며, 저장 규약에 따라 등록한 고유 run과 출력 경로만
`shadow_run_id`, `shadow_journal_path`로 지정합니다.

## 개별 실행

통합 실행과 개별 실행을 동시에 켜지 않습니다. 각 항목은 다른 입력/소비자가 준비돼 있어야 합니다.

```bash
roslaunch planning_inputs vehicle_state.launch
roslaunch planning_inputs objects.launch
roslaunch asmc_planner planner_v2.launch   state_topic:=/planning/vehicle/state objects_input_type:=detected   behavior_objects_topic:=/planning/objects planning_objects_topic:=/planning/objects
roslaunch asmc_controller controller.launch state_topic:=/planning/vehicle/state
```

개별 명령 네 개에는 최종 제어 선택기가 포함되지 않습니다. 일반 주행은 통합 launch를 사용합니다.
`planner_v2.launch` 인자를 생략하면 `/vehicle/state`·MORAI Object 입력을 쓰는 개발 단독 설정입니다.
MPC 입력 누락 시 STOP은 fail-safe 확인이며 정상 추종 증거가 아닙니다.

```bash
# 대시보드는 통합 launch에 포함되므로 별도 실행하지 않음
rostopic info /ctrl_cmd
rostopic info /perception/camera/objects
rostopic hz /planning/vehicle/state
rostopic hz /planning/objects
rostopic hz /planning/trajectory
rostopic echo -n 1 /planning/frenet_feedback_v2
```

## 현재 topic·실패 처리

| Topic | 생산자→소비자 | 의미 |
|---|---|---|
| `/planning/vehicle/state` | planning_inputs→Planner/MPC | 최신 현재 상태 |
| `/planning/objects` | planning_inputs→Planner | authoritative OBB·ID·type·velocity |
| `/behavior/command_v2` | Planner→관찰/MPC | 내부 Behavior 기동 계약의 공개 관측 |
| `/planning/frenet_feedback_v2` | Planner→관찰 | execution/transaction 결과 |
| `/planning/trajectory` | Planner→MPC | source-time trajectory·ego provenance |
| `/ctrl_cmd/normal` | MPC→selector | 일반 구간 제어 |
| `/ctrl_cmd/jamming` | 음영 adapter→selector | 재밍 제어 |
| `/ctrl_cmd` | selector→bridge | 최종 제어, publisher 하나 |

state/trajectory stale·invalid·solver/reference 실패는 정지 경로입니다.
#89의 selector watchdog은 선택 입력250ms 단절 시20Hz 제동을 발행하며 선택되지 않은 입력으로 타이머를 갱신하지 않습니다.
source stamp·valid·failure code·OBB·실제 제동 응답도 확인하고 Hz만으로 수락하지 않습니다.
카메라3그룹이 숫자 enum에서는 일부 unknown으로 전달되므로 unknown을 객체 없음이나 정적으로 해석하지 않습니다.

## #89 설정과 검증 범위

차량 상태와 LiDAR의 `imu_yaw_is_true_enu=true`는 MORAI true ENU yaw를 UTM52N map 방향으로 보정합니다.
이미 보정된 producer는 양쪽 설정을 함께 false로 바꿉니다. 관련 패키지를 재빌드·재시작합니다.
GPS 위치 gain0.35, 속도별 gain, 조향0.15초 지연·29step×0.05초·속도별 변화 한계는
[planning_inputs](../../src/planning_inputs/README.md)와 [MPC](../../src/asmc_controller/README.md)에 설명합니다.

일반 KEEP 목표/상한58km/h와 지정7개 링크의 조건부 곡선 설정은 지도 원본 수정과 다릅니다.
신호/STOP/FOLLOW/회피/합류를 우선하며 실측 최고속도60km/h 미만을 별도 검사합니다.
부분 rollback은 `normal_cruise.enabled=false`, `keep_cruise.enabled=false`,
`cost.curve_contour_boost=0.0`이며 yaw 보정 한쪽만 되돌리지 않습니다.

[PR #89 원본 검증](https://github.com/INHAautonav/2026-ASMC/pull/89)은 직진 안정성3회와 속도 튜닝2회 완주를 보고합니다.
후자의 실측 최고57.72/58.04km/h, 전체 이동 평균30.85km/h이며 재밍 이후 평균40km/h는 미달성입니다.
속도 튜닝 후 p95 지도 중심선 오차가 증가한 trade-off와 rebase 이후 새 정량 Bag 미측정도 남습니다.
`roundabout_execution_test`의 `safe distant candidate cannot commit six metres before the stopline` 실패는
PR #89 당시 기록입니다. 이후 [PR #135 검증](https://github.com/INHAautonav/2026-ASMC/pull/135)에서는
해당 회귀 실행파일이 통과했습니다. 이는 합성 회귀 확인이며 새 전 구간 주행 성공을 의미하지 않습니다.

## 회귀와 시나리오

운영 `runtime` 빌드만으로 시험 실행파일이 모두 준비되지는 않습니다. 동일한 outer workspace에서
`full`로 시험·개발 도구를 빌드한 뒤 실행합니다. [빌드 모드 안내](build.md)를 참고합니다.

```bash
./scripts/build_ws.sh full -j2 -l2
source devel/setup.bash
catkin_make run_tests_asmc_planner run_tests_asmc_controller run_tests_planning_inputs
catkin_test_results build/test_results
```

등록 CTest와 명시적 ROS 시험의 실행·기준 실패는 package README와 CI 로그를 확인합니다.
과거 테스트 개수를 현재 숫자로 복사하지 않습니다. 전체 씬·단일 구간·충돌/정체 재현은
[scen_runner](../../tools/scen_runner/README.md)를 사용합니다. 개발용 gRPC/DLL 도구의 성공은 본선 허용 입력 수락과 별도입니다.
최종 목표·소비자 남은 작업은 [대회 로드맵](../architecture/consolidation-roadmap.md)을 따릅니다.

## MORAI 제어 프로토콜과 정지 순서

실주행 전 Network Settings → Ego Network에서 **Cmd Control = MoraiCmdController**,
UDP, local Host/Destination IP127.0.0.1, Host PORT9093·Destination PORT9094를 확인한다.
`TrajectoryTrackingCmd`는 다른 프로토콜이다. 포트 숫자가 같아도 `/ctrl_cmd`를 인코딩한
`#MoraiCtrlCmd$` 패킷의 수신·적용을 보장하지 않는다. 네트워크 파일을 다시 불러온 뒤에는
드롭다운의 실제 표시 이름과 연결 상태도 확인한다. sensor JSON·raw export의 release 차이는
runtime을 별도 프로토콜로 나누는 것과 다르며 옛 enum 숫자로 UI 이름을 추측하지 않는다.

2026-10-26 현장 진단: 잘못된 TrajectoryTrackingCmd에서는 Docker와 Windows에서 보낸
가속0·브레이크1 제어가 적용되지 않았다. MoraiCmdController·9093/9094로 재연결한 뒤
같은23바이트 제어의 D·브레이크1.0·조향0.1 적용을 확인했다. UDP send 성공/20Hz만으로
시뮬레이터가 명령을 수락했다고 표시하지 않는다.

추가로 **제어 종류의 불일치와 조향 부호 불일치는 별개 문제**였다. 현재 PC의 R1.H3·IONIQ5에서
기존 `invert_steer=false`는 좌회전 양수 MPC 명령을 실제 우회전으로 적용했다. 같은 경로 시작점과
새 LiDAR 입력을 유지한 제한 시험은3.67초·11.82m에서 횡오차0.8831m로 중단했다. ROS 명령과
차량 yaw 변화를 함께 확인했고, 제어기 알고리즘·지도 필터·장애물 임계값은 수정하지 않았다.

현재 PC의 수정은 bridge 송신 경계에서 **좌회전 양수 rad → 우회전 양수 정규값**으로 한 번
반전하는 것이다. **AIM06 R1.H3는 `invert_steer:=true`**, 공용 기본값은 기존 false를 유지한다.
Git `f84a62ce`(2026-09-02)에 팀원의 false 주행 확인 기록이 있어, 대회용 실행판도 같은 응답이라고
가정하지 않았다. 이후2026-10-26 사용자가 **대회용 MolitComp03에서 기본값 false로 새 LiDAR
판제 주행이 동작함을 직접 확인**했다. 같은 시점의 읽기 전용 조회에서 실행판
`Simulator_v.S4.251001.MolitComp03_Window`, bridge `invert_steer=false`·local UDP,
당시 `planning_objects_adapter/source_policy=asmc_lidar`와 `/perception/lidar/tracks_map` 입력을 확인했다.
B4의 현행 단일 입력은 [KF 품질 관측 정본](../../src/asmc_lidar/docs/planning-runtime.md)으로 변경됐다.
따라서 대회용 공용 기본값 false를 유지하고, 확인한26R1 개발 조합만 true를 명시한다.
`/ctrl_cmd/normal`, `/ctrl_cmd/jamming`과 MPC bicycle model의 좌회전 양수
계약은 유지한다. ROS WebSocket은 이 UDP 인코더를 거치지 않는다. 다른 PC의 정상 주행과 비교할 때
release·차량40도 조향 한계·통신 방식·bridge revision/설정을 함께 확인한다.
다른 PC도 동일 증상이라고 단정하거나 제어기 부호를 함께 뒤집지 않는다.

```bash
# 대회용 MolitComp03
./scripts/udp_bridge.sh
# 확인한 26R1 개발 조합
./scripts/udp_bridge.sh invert_steer:=true
```

임시 송신기에 동일 부호 수정을 적용한 후 첫 대조는5.30초·25.05m, 최대 횡오차0.0536m였다.
추가 계측 대조는16.00초·106.64m의 직선·곡선에서158개 조회 표본 모두 plan_valid=true,
최대 횡오차0.0972m, LiDAR source age 최대109.6ms·판제 객체 수신 age 최대105.2ms였다.
해당 구간의 전달 객체는 정상 빈 배열이므로 장애물 회피·추적 속도 품질을 검증한 시험은 아니다.
이 단계는 임시 송신기의 후보 대조이며 운영 bridge 적용 결과와 구분한다.

이후 사용자 승인으로 `/morai_udp_bridge`만 `invert_steer:=true`로 재시작하고, 임시 송신기의
추가 반전을 제거해 실제 bridge를 검증했다. 같은 시작점의16.00초·107.00m에서158개 조회 표본
모두 plan_valid=true, 최대 횡오차0.1235m, 최대 LiDAR source age114.5ms·판제 객체 수신
age109.5ms였다. 센서 bridge·카메라·LiDAR·master는 유지했고, 생성한 시험 판제만 종료한 뒤
Keyboard/P와 `/ctrl_cmd` 발행자0을 확인했다. 이26R1 대조의 직접 명령은 위 true override를 사용한다.

대회 버전의 사용자 직접 확인은 새 LiDAR 연결의 기능 확인이다. 구간·시간·장애물 조건을 계측한
전 미션 수락이나 LAN2LAN 시험으로 확대하지 않는다. 위 수치가 있는26R1 제한 대조와도 구분한다.

연장 대조 중 별도1회는2.85초에 `ego/object input is not fresh`로 정지했다. 이 실행은 당시
입력별 계측이 없어 차량 상태와 LiDAR 중 어느 입력인지 확정하지 못했다. 이후 계측 대조의 성공이
그 실패를 면제하지 않는다. 입력 중단 재현 시 `/planning/vehicle/state`와 `/perception/lidar/planning_observations_map`,
`/planning/objects`, 어댑터 diagnostics의 시각·유효성·탈락 사유를 함께 확인하며 timeout을 늘리거나
센서 무효를 정상 빈 결과로 바꾸지 않는다.

네트워크 파일의 release별 제어 선택은 [MORAI 설정표](../../config/morai/README.md#모드와-채널)가
정본이다. 26R1 생성본은 type786→771, import `egoCtrlMode`2→3으로 수정했고 MolitComp03의
type771은 유지하고 import mode는 설치 enum 확인 후3=AutoMode로 정정했다. 두 버전 모두
JSON1=Keyboard,2=GameWheel,3=AutoMode이며 UDP payload CtrlMode는 별도로2다. 현재 PC 저장본과
소스 template의 SHA를 구분해 계약에 보존했다. Windows SaveFile의 옛 복사본은 자동 갱신되지 않는다.

`/morai_udp_bridge/diagnostics`의 cmd 항목에 실제 로딩한 `cmd_ip`, `cmd_port`, `cmd_bytes`,
`max_steer_deg`, `invert_steer`가 포함된다. 이 값은 해당 프로세스가 사용하는 값이며,
`rosparam set`만으로 이미 초기화된 인코더가 바뀌지 않는다. 변경 적용에는 제어 bridge의 재시작이 필요하다.

정차·종료는 **autonomy/명령 발행부터 종료 → 키보드 제어·속도0·P 확인** 순서다.
브릿지는 각 Ctrl 패킷에 Auto2·D4를 포함하므로 발행자가 계속 살아 있으면 기어/모드만
수동 변경한 상태가 다음 명령으로 덮어써질 수 있다. camera/lidar/view는 구동 명령을 발행하지 않는다.
`Camera lane follower STOP: inactive_mode`는 NORMAL 모드에서 음영 follower가 선택되지
않았다는 진단이다. 실제 차를 정지시켰는지는 control selector·Frenet feedback·최종 `/ctrl_cmd`로 확인한다.

제어 채널을 복구한 뒤에도 `low-speed KEEP blocked by drivable-road boundary`가 나오면
객체 필터나 도로 경계 검사를 끄지 않는다. 현재 rear-axle 위치·방향·전역경로와 planner의 고정
도로 bundle을 대조한다. 같은 날 전역경로20번 표본 중심에서 새 판제 상태로 시작한 제한 대조는
약1.8초·약2m, 7개 조회 표본의 plan_valid가 모두 true였고 판제 횡오차 최대0.045m였다.
실제 throttle·normalized steer가 명령에 따라 바뀌는 것까지 확인했으며 전 코스·고속·미션
주행 수락이나 속도 GT 정확도 검증을 뜻하지 않는다. 시험 종료 후 명령 발행자부터 정리하고
키보드/P·정차를 조회로 확인했다. 위치 순간이동 후 남은 이전 estimator/planner 이력도 구분한다.
