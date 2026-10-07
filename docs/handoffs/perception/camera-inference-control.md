# 모델별 카메라 추론 실행 상태

PR #68의 전방 YOLO opt-in 이전은 main `7aed612`에 병합됐다. 후속 작업은
`feat/camera-region-activation`에서 모델 실행 인터페이스와 교체 가능한 예시를 제공한다.
구간·결과 소비·주행 모드·제어권은 판제가 결정한다. 카메라 생산자는 GPS나 HD 구간을 읽지 않는다.

## 실행과 외부 요청

공유 컨테이너 `asmc-ros-noetic`의 `/root/ws`는 현재 카메라 작업 worktree다.
호스트의 각 터미널에서 직접 진입한 다음 같은 ROS master를 사용한다.

```bash
docker exec -it asmc-ros-noetic bash
```

```bash
cd /root/ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11311
export ROS_HOSTNAME=localhost
```

bridge 터미널에서는 기존 실행을 중복으로 띄우지 않고 아래 launch를 실행한다.
master가 없으면 roslaunch가 함께 시작한다. 실시간에는 `/use_sim_time=false`를 사용한다.

```bash
roslaunch asmc_bridge bridge.launch transport_profile:=local stream_profile:=competition
```

컨테이너·bridge와 rqt 창은 사용자가 별도로 실행한다. ROS 환경을 읽은 터미널에서
다음 프로필 하나를 선택한다. 카메라는 어느 구간을 주행 중인지 판단하지 않는다.

| 직접 launch | 모델 로드와 실행 | 시각화 |
|---|---|---|
| `roslaunch asmc_camera perception.launch` | idle: 모델·차선 downstream을 실행하지 않음, 서비스 없음 | 없음 |
| `roslaunch asmc_camera perception.launch profile:=planning` | 세 모델 로드·dummy 예열 후 STANDBY, 외부 요청 대기 | 꺼짐 |
| `roslaunch asmc_camera perception.launch profile:=debug` | 세 모델 ACTIVE, TwinLite 3뷰·YOLO 전방·ResNet camera4 | debug publisher 켜짐, rqt 별도 |
| `roslaunch asmc_camera perception.launch profile:=legacy` | 기존 차선 반복 추론, 모델 목표 Hz 제한 없음 | 꺼짐 |

인자 없는 idle launch는 노드가 없어 종료한다. 판제가 서비스를 호출하려면 planning으로
모델 노드를 먼저 띄운다. STANDBY는 GPU 모델을 유지하며 반복 추론을 중단하는 상태이고,
노드를 실행하지 않는 `start_*=false`와 다르다. 서비스 요청은 없는 노드를 생성하지 않는다.

```bash
rosparam set /use_sim_time false
# 판제용: 이후 모델별 서비스로 준비·활성화
roslaunch asmc_camera perception.launch profile:=planning
```

```bash
# 디버그용: 서비스 요청 없이 모든 모델 추론·시각화 시작
roslaunch asmc_camera perception.launch profile:=debug
```

```bash
# YOLO만 디버그: 차선·신호등 모델과 관련 시각화도 실행하지 않음
roslaunch asmc_camera perception.launch profile:=debug \
  start_camera_lane:=false start_traffic_light:=false
```

planning/debug의 `managed`는 true다. 목표는 TwinLite 10Hz **배치율**(각 뷰 10Hz),
YOLO 20Hz, 신호등 10Hz이며 새 입력만 처리한다. 이전 관측을 복제해 Hz를 채우지 않는다.
입력이나 연산 성능이 부족하면 목표보다 낮아질 수 있다. legacy는 입력·연산 성능이
허용하는 만큼 반복 처리하며 같은 목표 제한을 적용하지 않는다.

`start_camera_lane`, `start_camera_objects`, `start_traffic_light`는 프로필에서 선택한
모델을 개별 제외/추가하는 고급 옵션이다. `*_initial_mode`로 초기 모드,
`lane_target_hz`, `objects_target_hz`, `traffic_light_hz`로 모델별 목표를 덮어쓸 수 있다.
`start_visualization`은 시각화 publisher만 제어한다. planning은 차선 리뷰 marker도
기본 비활성이고, 운영 `LaneBoundaryFamily`는 차선 모델을 활성화하면 계속 제공한다.
개별 backend launch `camera_lane.launch`/`camera_objects.launch`도 기본 managed STANDBY이며,
상위 `perception.launch`를 일반 진입점으로 사용한다. 직접 backend만 반복 재현하려면
`managed:=false`를 명시한다. 새 기본값은 이 프로필 도입에서 변경된 것이며 과거 증거의
명령을 실행할 때도 당시 legacy/managed 프로필을 명시한다.

### 목표 Hz를 파일에서 바꾸기

상위 진입점의 기본값은 `src/asmc_camera/launch/perception.launch`의 다음 인자다.

```xml
<arg name="lane_target_hz" default="10.0"/>
<arg name="objects_target_hz" default="20.0"/>
<arg name="traffic_light_hz" default="10.0"/>
```

각각 TwinLite 배치, 전방 YOLO, camera4 ResNet의 목표다. 수정 후 카메라 launch를
다시 시작하면 반영되며 catkin 재빌드는 필요 없다. launch 명령의 같은 이름 인자가
파일 기본값을 덮어쓰고, 실행 중 판제 서비스의 양수 `target_hz` 요청이 다시 덮어쓴다.
`target_hz: 0`은 현재 값을 유지한다. rosparam만 바꿔서는 실행 중 주기 관리자가 갱신되지 않는다.

`camera_model_grid_max_hz`는 차선 model-grid 출력 제한이므로 추론 목표가 아니다.
YOLO 클래스·confidence YAML과 `config/runtime.yaml`의 설명 값을 바꿔서 목표 Hz를
조정하지 않는다. backend launch를 직접 실행할 때는 해당 파일의 `target_hz` 인자가 적용된다.
선택적 구간 예시를 쓰면 `config/runtime/camera_region_example.yaml`의 모델별 요청 Hz도
별도로 확인한다. 예시/판제가 요청하는 주기를 카메라 launch가 고정하지 않는다.

| 모델 | 서비스 | 상태 토픽 |
|---|---|---|
| TwinLite | `/camera_semantic_inference/set_inference_mode` | `/camera_semantic_inference/inference_status` |
| 전방 YOLO | `/camera_object_detector/set_inference_mode` | `/camera_object_detector/inference_status` |
| camera4 ResNet18 | `/traffic_light_infer/set_inference_mode` | `/traffic_light_infer/inference_status` |

타입은 `asmc_camera/SetInferenceMode`, `asmc_camera/InferenceStatus`다.
`mode`는 STANDBY=0, PREPARE=1, ACTIVE=2이며 `target_hz: 0`은 현재 설정 유지다.
서비스의 `accepted`는 요청 수락이고 준비 완료나 목표 주기 달성을 의미하지 않는다.
동일한 모드·주기를 재요청하면 처리 세대를 바꾸지 않는다. 모드·주기 변경은 세대를 올린다.

```bash
# 실제 새 영상으로 차선 내부 파이프라인 준비
rosservice call /camera_semantic_inference/set_inference_mode "{mode: 1, target_hz: 10.0}"
rostopic echo /camera_semantic_inference/inference_status

# prepared와 pipeline_ready를 확인한 뒤 새 차선 관측을 공급
rosservice call /camera_semantic_inference/set_inference_mode "{mode: 2, target_hz: 10.0}"

# YOLO·신호등도 독립적으로 준비 → 활성화
rosservice call /camera_object_detector/set_inference_mode "{mode: 1, target_hz: 20.0}"
rosservice call /camera_object_detector/set_inference_mode "{mode: 2, target_hz: 20.0}"
rosservice call /traffic_light_infer/set_inference_mode "{mode: 1, target_hz: 10.0}"
rosservice call /traffic_light_infer/set_inference_mode "{mode: 2, target_hz: 10.0}"

# 필요한 모델만 정지: GPU 모델은 유지
rosservice call /camera_semantic_inference/set_inference_mode "{mode: 0, target_hz: 0.0}"
```

각 ACTIVE 요청도 세대를 바꾸므로 이전 PREPARE의 진행 중 결과·대기 프레임을 새 준비
횟수에 포함하지 않는다. 새 source stamp의 연속 정상 처리 3회가 필요하다. 오류나 입력
중단은 준비를 해제한다. status는 wall time 0.2초 주기로 발행해 `/clock`이 멈춰도
0.35초 freshness 상실을 보고한다. 서비스가 없거나 status 수신 자체가 중단되면 소비자는
준비되지 않은 모델로 취급한다. 모델 로드 중에는 서비스가 아직 제공되지 않는다.

`prepared`는 추론 처리 성공, `last_output_valid`는 최근 의미 판정의 유효성이다.
빈 YOLO 결과·신호등 `not_detected`도 정상 추론일 수 있다. TwinLite의 `pipeline_ready`는
세대 cutoff 이후의 최신 유효 `LaneBoundaryFamily`까지 확인한다. `processing_hz`는 추론
배치율이며 뷰별·최종 차선 출력율과 구분한다. 결과를 복제하거나 source stamp를 바꾸지 않는다.

PREPARE에서 차선 downstream은 준비 확인을 위해 가동한다. **차선 토픽을 받았다는
사실만으로 제어를 열면 안 된다.** 소비자는 ACTIVE, generation/cutoff, status freshness와
자기 결과의 source stamp·valid를 함께 확인한다. YOLO·신호등은 ACTIVE 전에는 운영
판정을 발행하지 않는다. STANDBY에서는 이전 결과를 한 번 무효화하고 신호등 legacy는
`not_detected`로 초기화한다. 무효화 메시지는 알려진 원본 header를 유지한다.

managed 모드에서 모델·차선 노드 실패는 다른 모델을 종료시키지 않는다. 기존 legacy
launch의 required 설정은 유지한다. 차선 내부 입력도 cutoff 이전 결과를 배제한다.

## 시각화

`start_visualization`은 debug 영상 발행을 켠다. 창은 별도 터미널에서 직접 연다.
STANDBY에서는 새 추론 영상이 갱신되지 않으며 PREPARE/ACTIVE에서 확인한다.

```bash
rqt_image_view --clear-config /perception/camera/debug/mosaic/compressed
```

```bash
rqt_image_view --clear-config /perception/bev/debug/comparison/compressed
```

```bash
rqt_image_view --clear-config /perception/traffic_light/debug/image/compressed
```

YOLO 전용 overlay는 같은 source stamp·frame_id·영상 크기에만 박스를 그린다.
LiDAR·GPS·차선 추론이 필요하지 않으며 정상 빈 검출도 영상으로 확인할 수 있다.

```bash
rqt_image_view --clear-config /perception/camera/objects/debug/image/compressed
```

전체영상 YOLO와 기존 LiDAR fusion은 바깥 workspace를 source한 뒤 nested workspace를
source하고 직접 실행한다. 유효한 Ego 속도·GPS·IMU·LiDAR 입력도 필요하다.

```bash
source src/lidar+sensorfusion/devel/setup.bash
roslaunch Main main.launch start_yolo:=false rviz:=false
```

```bash
rqt_image_view --clear-config /fusion/debug_image
```

## 교체 가능한 HD 구간 예시

모델 생산자와 독립된 예시 노드는 기본적으로 실행되지 않는다. 수동 서비스 호출 또는
판제와 예시를 동시에 제어 주체로 사용하지 않는다. 예시는 차량 제어를 발행하지 않는다.

```bash
roslaunch asmc_camera camera_region_example.launch allow_example_control:=true
```

설정은 `src/asmc_camera/config/runtime/camera_region_example.yaml`이다. 모델별 초기
모드·zone ID·Hz·GPS 손실 요청과 준비·복귀 조건을 바꾸거나 노드를 제거해 판제가 대신
같은 서비스를 호출할 수 있다. 기본 예시는 TwinLite와 ResNet 10Hz, YOLO 20Hz 요청이다.
실제 입력보다 높은 목표 Hz를 요청해도 이전 관측을 재발행하지 않는다.

준비 lead는 `max(3초, 측정된 준비 p99+1초)`, 거리는 `max(20m, 속력×lead+5m)`이다.
현재 p99 설정은 측정값 주입 자리이며 대회 장비의 준비 시간을 보장하는 pin이 아니다.
GPS/HD context가 없으면 차선 ACTIVE를 유지한다. 복귀에는 새 유효 위치·차로·경로
0.5초와 음영 출구 5m 여유가 필요하다. 이는 예시 정책이며 카메라 서비스의 강제 규칙이 아니다.
음영 안의 위치를 추정하거나 마지막 위치를 새 위치로 발행하지 않는다.

`camera_model_grid_max_hz`는 model-grid 출력 상한이다. 실제 추론 주기는 managed
`target_hz`로 제어한다. managed 추론이 downstream 상한 이하이면 동일 관측을 독립된
주기 제한으로 다시 버리지 않는다. legacy 주기 제한은 유지한다.

## 검증과 수락 범위

단위 시험, 생성 메시지/실제 ROS 서비스 장애 시험, 선택적 예시 ROS 시험, 실제 GPU bag
시험과 MORAI 기록은 `config/baselines/perception_camera_inference_lifecycle_v1.yaml`에서 관리한다.
프로필 변경과 YOLO overlay의 추가 검증은 `config/baselines/perception_camera_launch_profiles_v1.yaml`에 기록한다.
기존 GPU 수락 증거는 당시 프로필 기준으로 보존하며 새 debug 동시 실행의 처리율 보장으로 해석하지 않는다.
GPU 시험은 동일 bag의 실패 실행도 보존한다. 세 모델을 함께 로드하고 한 모델씩 ACTIVE로
측정한 시험이며 세 모델 동시 추론 성능 보장은 아니다.

최종 코드의 반복 05/06/07 중 두 번은 전체 gate를 통과했다. 05에서는 우측 카메라의
ACTIVE 첫 1초 결과가 8개로 기준 9개에 미달했다. 이후 5초 창은 9.8Hz였지만 이를 근거로
첫 구간 실패를 면제하지 않는다. **10Hz 프로필 수락은 보류**한다. YOLO는 세 번 모두
19.88Hz 이상이며 별도 3,514프레임 비교는 header·클래스·순서·정수 좌표 모두 정확히 일치했다.
준비 시간 p99는 아직 측정하지 않았으므로 예시 lead를 장비 보장값으로 쓰지 않는다.

MORAI에서는 공식 시나리오 SHA·spawn, 텔레포트·카메라 준비·기존 21% crop preview를
확인했다. 제어 성공 응답에 비해 차량 이동이 거의 없고 출구 fixture에서 HD 차로 유효성이
확보되지 않아 **연속 주행·복귀 수락은 보류**한다. LiDAR live profile도 차량 속도 입력
부재를 별도 기록한다. 이 실패를 카메라·단위 시험 통과로 대체하지 않는다.

기존 `lane_boundary_adapter`·`jamming_lane_boundary_adapter` 코드와 기본 파라미터,
21% crop은 변경하지 않았다. 시험에서는 5km/h 등 실행 인자만 기록해 적용했다.
MORAI 테스트 도구는 actor 일괄 삭제를 사용하지 않고 종료 시 Ego pose·gear·제어 모드만
복원을 시도한다. 기존 실패 report의 `driven` 필드는 명령 전달 시도를 뜻하며 실제 이동
성공은 `moved_m`으로 판정한다. 기존 `restored`는 즉시 XY 확인만 사용했으므로 전체
복원을 뜻하지 않는다. 최종 재확인에서 원래 위치 오차는 0.011m 미만, gear·제어 모드는
일치했지만 방향 오차 1.347°가 남았다. 시뮬레이터는 일시정지해 두었으며 엄격한 pose
복원은 통과하지 않았다. 도구는 이제 안정된 위치·방향·gear·mode를 검사하고 미달을
실패로 기록한다. 전체 actor 복원·대회 주행 승인을 주장하지 않는다.

3카메라 YOLO, 동시 모델 성능, LiDAR/YOLO 10Hz 조합, 판제의 jamming/crop 통합,
YOLO26·학습, batch·공유 처리·TensorRT, 4070 SUPER 인수는 후속 독립 단계다.
