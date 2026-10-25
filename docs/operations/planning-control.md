# Planning/Control 실행·검증

현재 패키지는 `asmc_planner`, `asmc_controller`, `planning_inputs`입니다.
bridge·카메라·LiDAR는 별도 실행합니다. 새 경로는 `asmc_lidar`, 구 Main은 인수 전 기존 실행/복구용입니다. 전체 시작 순서는 [실행 명령](commands.md),
실제 인자/기본값은 [launch 목록](launch-reference.md)이 정본입니다.

## 빌드와 통합 실행

`asmc-ros-noetic`의 저장소 루트에서 실행합니다. 새 경로는 outer 빌드만 필요하며 nested Main 빌드는 구 경로 실행/복구에 한정합니다.

```bash
cd /root/ws
source /opt/ros/noetic/setup.bash
./scripts/build_ws.sh -j2
source devel/setup.bash
# bridge·카메라·asmc_lidar를 별도 준비한 뒤
roslaunch asmc_planner autonomy_no_bridges_v2.launch lidar_source_policy:=asmc_lidar
```

이 명령은 차량 제어를 시작합니다. 신호등 추론·발행은 카메라 담당의 `asmc_camera`만 소유하며,
판제 launch는 별도 ResNet을 시작하지 않습니다. `traffic_light:=false` 등 폐기 인자는 삭제합니다.
실행 중 중단/재개·주기는 카메라의 `set_mode`에서 제어합니다. 판제 신호 규칙·stale/invalid 처리와
구간별 소비 책임은 유지합니다. [소비·제어 안내](../handoffs/perception-to-planning.md)를 따릅니다.

`state_source:=gps_imu`는 `/planning/vehicle/state` producer를 포함하고 `external`은 외부 producer를 요구합니다.
통합 객체 source는 `lidar`이며 새 선택 `lidar_source_policy:=asmc_lidar`는 `/perception/lidar/tracks_map`을 직접 소비합니다. 속도/4단계 운동·원본 OBB·세대/시각/상태를 확인하고 두 번째 인지 필터·JSON join을 실행하지 않습니다. 실제 인수 전 통합 기본은 구 `validated_lidar`입니다. 이 구 경로는 `/last_lidar`와 exact source stamp의 `/lidar/motion_debug`를 연결하며 `main_raw`는 기존 비교용입니다. [연결 의미와 복구](../../src/asmc_lidar/docs/planning-runtime.md)를 따릅니다.
통합 launch에는 제어 선택기와 `enable_control=true`인 재밍 어댑터가 포함됩니다.

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
# 관찰용 dashboard; 새 planner를 실행하지 않음
roslaunch asmc_planner dashboard.launch state_topic:=/planning/vehicle/state
rostopic info /ctrl_cmd
rostopic info /YOLO/BBox
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

차량 상태와 Main의 `imu_yaw_is_true_enu=true`는 MORAI true ENU yaw를 UTM52N map 방향으로 보정합니다.
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
`roundabout_execution_test`의 `safe distant candidate cannot commit six metres before the stopline` 실패가 보고됐습니다.
이 문서 갱신에서 새 주행 성공이나 회귀 전체 통과를 주장하지 않습니다.

## 회귀와 시나리오

```bash
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
