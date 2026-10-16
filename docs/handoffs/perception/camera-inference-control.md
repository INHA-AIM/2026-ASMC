# 모델별 카메라 추론 실행 상태

**현재 안내:** 기능별 모드 서비스와 `camera.launch`·`view.launch`는 [실행 계약](../../contracts/perception/camera-runtime-v1.md)을 따른다. 아래 PR #69의 옵션·실험 수치는 과거 기록이며 Main 독립 YOLO 복구 옵션은 제거됐다.


> 현재 실행·새 제어 경로는 [카메라 실행·3뷰 관측 계약](../../contracts/perception/camera-runtime-v1.md)이 정본입니다.
> 아래 기존 profile/서비스 설명과 과거 검증 결과는 호환·재현 기록으로 보존합니다.

**2026-10-14 실행 기본 갱신:** YOLO26l v11 E8·3뷰15Hz가 기본이며 사전학습 YOLO26l은
`objects_model_profile:=pretrained_coco7`로 선택합니다. 모델 교체는 종료·재시작 방식입니다.
기존 모드 서비스와 MD5는 유지합니다. [모델 파일·소비 계약](camera-mission16-deployment.md)을 먼저 확인하세요.
TwinLite와 ResNet의 기존10Hz는 이번에 유지하며, TwinLite15Hz는 후속 최적화 대조입니다.


PR #68의 전방 YOLO opt-in 이전은 main `7aed612`에 병합됐다. 후속 작업은
`feat/camera-region-activation`에서 모델 실행 인터페이스와 교체 가능한 예시를 제공한다.
구간·결과 소비·주행 모드·제어권은 판제가 결정한다. 카메라 생산자는 GPS나 HD 구간을 읽지 않는다.

**현재 차선 피팅·`LaneBoundaryFamily`는 음영구간 전용이다. 음영구간 밖의 차선 피팅은
구현·검증 범위에 포함되지 않으며 주행에 사용하면 안 된다.** `valid=true`나
`pipeline_ready=true`가 나와도 음영구간 밖의 사용을 허용하지 않는다.
`operating_domain=shadow_two_way_v1`은 알고리즘 적용 범위의 식별자이며 현재 위치를
검증하는 장치가 아니다. 판제가 구간 적용 조건과 결과 유효성을 함께 확인해야 한다.

`profile:=debug`의 전 모델 ACTIVE는 영상·연산 확인용이다. 음영구간 밖에서 TwinLite·BEV·
차선 화면을 볼 수 있지만 피팅 결과를 제어에 연결하지 않는다. 구간 밖의 무효 피팅을
해결하려고 수락 임계값을 완화하지 않는다. 차선 연산도 제외하려면
`profile:=debug start_camera_lane:=false`로 YOLO·신호등만 실행할 수 있다.

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
| `roslaunch asmc_camera perception.launch profile:=debug` | 세 모델 ACTIVE, TwinLite 3뷰·YOLO 3뷰·ResNet camera4 | debug publisher 켜짐, rqt 별도 |
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
YOLO 15Hz, 신호등 10Hz이며 새 입력만 처리한다. 이전 관측을 복제해 Hz를 채우지 않는다.
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

### 보넷 제외: semantic GT 윤곽 mask 기본 활성화

기본은 보넷 semantic GT에서 확인한 윤곽을 입력 복사본에 검게 가리는 방식이다.
원본 영상·TwinLite·ResNet 입력, 객체 원본 좌표·source header는 유지한다.
상위·개별 객체·시각화 launch의 `objects_hood_mask`는 **true**이고,
`objects_ego_hood_filter`는 **false**다. `profile:=idle`은 여전히 아무 모델도 실행하지 않는다.

```bash
roslaunch asmc_camera perception.launch profile:=debug
```

```bash
rqt_image_view --clear-config /perception/camera/objects/debug/image/compressed
```

전체영상 비교·롤백은 명시적으로 mask를 끈다.

```bash
roslaunch asmc_camera perception.launch profile:=debug objects_hood_mask:=false
```

실험용 후처리는 mask를 끄고 켠다. 두 방식 동시 활성화는 시작 오류다.

```bash
roslaunch asmc_camera perception.launch profile:=debug \
  objects_hood_mask:=false objects_ego_hood_filter:=true
```

후처리 규칙은 원본 라벨 `car`, 임시 정수 좌표 기준 너비 ≥60%, 중심 y ≥85%,
하단 y ≥95%를 모두 요구한다. 출력 소수 좌표·confidence·순서는 바꾸지 않는다.
GT287과 근접 시험39에서는 새 실제 객체 누락이 없었지만, 제외 박스에 옆 차량 일부가
함께 들어가는 사례를 보넷 전용으로 확정할 수 없어 기본 방식으로 선정하지 않았다.

선정 mask는 고정 GT287에서 기존 보행자 검출 1건을 놓친다. **2026-10-07 사용자 지시에
따라 그 특정 사례만 수락 예외로 기록**했다. 신규 누락을 일반적으로 허용하는 기준이 아니다.
실제 보넷 윤곽은 semantic GT와 원본 RGB에서 확인했다. 하단21% 직사각형은 사용하지 않는다.
이전 엄격한 실패 판정·자료는 수정하지 않고 [후속 선정 증거](../../../config/baselines/perception_camera_hood_policy_selection_v1.yaml)에 새 결정을 기록한다.

영역 파일은 `src/asmc_camera/config/camera/yolo_hood_mask_v1.yaml`이며,
1280×720에서 114,058픽셀(약12.38%)이다. 센서 pose/FOV/해상도 설정이 바뀌면 윤곽을 다시 확인한다.
`objects_hood_mask_config:=/절대/경로.yaml`로 변경 후 launch를 재시작한다.
후처리 설정은 `yolo_ego_hood_filter_v1.yaml`과 `objects_ego_hood_filter_config`다.
PREPARE와 ACTIVE는 같은 설정을 사용하고 STANDBY에서는 반복 추론을 멈춘다.
진단에는 방식·설정 SHA·단계 시간·후처리 전/후 개수가 포함된다.
mask는 입력 텐서 크기를 유지하므로 연산량 절감을 의미하지 않는다.

`profile:=debug`에서는 `/perception/camera/objects/debug/raw`도 발행한다.
이는 **해당 실행에서 YOLO가 반환한 결과**이며 mask 모드에서도 별도 전체영상 추론을 하지 않는다.
`objects_debug_raw:=false`로 끌 수 있고 planning·객체 단독 launch 기본은 false다.
운영 `/perception/camera/objects`는 정제 결과다. overlay는 정확히 같은 source stamp의
원본 영상·정제 결과·debug raw를 사용하고 유지 박스와 후처리 제외 박스를 구분한다.
잘못된 설정은 시작 실패, 프레임 오류는 원본 header/크기의 `valid=false`다.

### fusion 지표와 추적 연속성

`Fusion sync matched`는 LiDAR와 YOLO **프레임 쌍**을 50ms 안에서 연결해 fusion을
수행한 횟수다. 박스 수·검출 차량 수·새 track 수가 아니며 빈 YOLO 결과도 매칭될 수 있다.
재검출이나 track 재생성 자체로 이 카운트가 올라가지 않는다.
따라서 매칭 수 감소가 깜박임 감소를 뜻한다고 해석하지 않는다.

`tools/perception/analyze_track_continuity.py`는 별도로 track별 관측 공백·재등장·분류 상실을
기록한다. 새 ID 수를 성능 점수로 사용하지 않는다. 실행별 track ID만으로 실제 객체의
ID 분절을 확정하지 않으며, 그 판단은 독립 actor identity 또는 검토한 시공간 대응이 필요하다.
기존 프레임 동기화 수락 조건과 모델 준비·MORAI·fusion 보류는 유지한다.

### 목표 Hz를 파일에서 바꾸기

상위 진입점의 기본값은 `src/asmc_camera/launch/perception.launch`의 다음 인자다.

```xml
<arg name="lane_target_hz" default="10.0"/>
<arg name="objects_target_hz" default="15.0"/>
<arg name="traffic_light_hz" default="10.0"/>
```

각각 TwinLite 배치, 3뷰 YOLO, camera4 ResNet의 목표다. 수정 후 카메라 launch를
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
| 3뷰 YOLO | `/camera_object_detector/set_inference_mode` | `/camera_object_detector/inference_status` |
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
rosservice call /camera_object_detector/set_inference_mode "{mode: 1, target_hz: 15.0}"
rosservice call /camera_object_detector/set_inference_mode "{mode: 2, target_hz: 15.0}"
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
같은 서비스를 호출할 수 있다. 과거 예시는 TwinLite와 ResNet 10Hz, YOLO 20Hz 요청이다. 현재 YOLO 기본은15Hz이며 예시를 사용할 때15Hz로 지정한다.
실제 입력보다 높은 목표 Hz를 요청해도 이전 관측을 재발행하지 않는다.

준비 lead는 `max(3초, 측정된 준비 p99+1초)`, 거리는 `max(20m, 속력×lead+5m)`이다.
측정 p99는 개발 GPU·입력·동시 부하 조건과 함께 기록하며 대회 장비의 준비 시간을 보장하는 pin이 아니다.
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

이전 코드의 반복 05/06/07 중 두 번은 전체 gate를 통과했다. 05에서는 우측 카메라의
ACTIVE 첫 1초 결과가 8개로 기준 9개에 미달했다. 이후 5초 창은 9.8Hz였지만 이를 근거로
첫 구간 실패를 면제하지 않는다. **당시 10Hz 프로필 수락은 보류**했다. YOLO는 세 번 모두
19.88Hz 이상이며 별도 3,514프레임 비교는 header·클래스·순서·정수 좌표 모두 정확히 일치했다.
당시 준비 시간 p99는 미측정이었다. 후속 측정도 대회 장비의 보장값으로 쓰지 않는다.

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

### 이후 검증 순서

1. 모델별 준비 수락: 세대별 STANDBY→PREPARE→ACTIVE, 실제 입력 연속3회 준비,
   준비 p99·ACTIVE 첫1초 출력·지속10Hz·입력 중단·재진입을 측정한다.
   차선 피팅의 소비는 음영구간으로 제한하며 구간 판단과 결과 선택은 판제가 담당한다.
2. Fusion은 프레임 공급·시간 매칭과 객체 연속성을 분리한다. 처리 LiDAR stamp 집합,
   timeout/pending, source offset, 독립 actor별 검출 공백·분류 공백·ID 변경을 함께 제출한다.
   독립 객체 대응이 없는 track ID 증감을 재검출 성능으로 해석하지 않는다.
3. 동일 장면으로 세 모델 동시 실행, 전방/3카메라 YOLO, LiDAR 최대주기/10Hz와
   YOLO20/10Hz 조합을 독립 비교한다. 기존 50ms/200ms 정책 변경은 별도 실험으로 남긴다.
4. 선정 실행 조합에서 YOLO26s/m 출력·성능 비교 후, 해상도·FP16·동일 모델 batch·
   공통 영상 디코딩·모델별 TensorRT를 병목 근거에 따라 각각 검증한다.
5. RTX4070 SUPER에서 예열·동시 부하·장시간 안정성·fallback을 인수한다.

### PR #69 병합 범위 변경 (2026-10-07 사용자 결정)

현재 PR의 병합 조건은 모델별 준비·전환·출력 주기와 카메라→LiDAR fusion 검증이다.
음영구간 연속 자동주행·제어권 전환·출구 주행 복귀는 판제 통합 단계로 분리한다.
앞의 MORAI 이동·출구 fixture·방향 복원 실패는 당시 실행의 실패로 그대로 보존하며,
성공으로 재분류하거나 카메라의 출력 검증으로 대체하지 않는다. 이번 후속 검증은 bag과
외부 실행 상태 API를 사용하며 차량 제어·텔레포트를 실행하지 않는다.

`tools/perception/check_model_lifecycle.py`는 PREPARE의 고정 시간 대기를 없애고,
서비스가 반환한 세대의 `prepared`와 `pipeline_ready`를 확인한 뒤 ACTIVE를 요청한다.
선택적 private 파라미터 `~lifecycle_trace_path`는 요청 수락·입력 도착·처리 시작/완료·
발행·준비 상태를 wall monotonic 시각으로 메모리에 기록하고 종료할 때 JSONL로 저장한다.
미설정 시 추적 직렬화·파일 작업이 없다. 메시지·서비스 형식과 운영 토픽은 바꾸지 않는다.

첫 1초는 서버의 요청 수락 시각부터 측정한다. 추적 OFF 수락 실행은 RPC 호출 시작을
보수적인 기준으로 사용한다. 전환 cutoff 이전 관측은 새 출력으로 세지 않는다.
모델 로드·dummy 예열과 실제 영상 준비·차선 downstream 준비 시간을 분리하고,
첫 정상 추론과 유효한 차선/신호등 판정을 동일하게 취급하지 않는다.

보넷 방식 선정으로 카메라·fusion 수락을 대신하지 않는다. 조건 미달은 기록하고 명시적인
수락 결정 없이 면제하지 않는다. 2026-10-07 사용자가 아래 분류 차이 2건을 승인하고
PR #69 병합을 요청했다. 이 두 관측에 한해 수락 예외로 처리하며 다른 실패에 확대하지 않는다.

### 후속 측정 결과

`config/baselines/perception_camera_acceptance_followup_v1.yaml`과 연결된 원본 증거를 따른다.
준비 확인 후 활성화하는 traced 3회와 untraced 2회 모두 첫 1초·지속 주기·최대 공백 gate를
통과했다. 모델별 warm PREPARE 30회의 실제 영상 준비 p99는 TwinLite 0.371초, YOLO
0.239초, ResNet 0.307초이며 status/downstream 준비 p99는 각각 3.412/0.353/0.495초다.
TwinLite downstream 값에는 유효한 LaneBoundaryFamily가 나올 때까지의 대기가 포함된다.
이는 연산 예열 시간과 다르며, 유효 차선이 없는 장면에서 준비를 보장하는 상한이 아니다.
개발 예시 lead는 TwinLite 4.412초, YOLO·ResNet 3초로 갱신했다.

이전 우측 첫 1초 8개 실패에는 프레임별 server trace가 없어 정확한 역사적 원인을 확정하지
않는다. 준비 대기·측정 기준을 보강했고 새 실행은 통과했지만 GPU 스케줄링 결함을 고쳤다고
주장하지 않는다. 단위·ROS 장애 시험과 실제 모델의 입력/clock 중단·준비 실패·복구·ACTIVE
재진입을 별도로 확인했다. 처리 성공률과 유효 차선/신호등 판정 비율은 분리해 보고한다.

추적 OFF 주 bag은 구 507/509·신 511/511 매칭으로 수량 gate를 통과했지만, 안정된 공통
3,197개 관측에 분류 차이 2건이 있어 **원래 자동 판정은 실패**다. 사용자의 명시적인 승인으로
이 두 관측의 수락 보류를 해제했다. 음영 bag은 구 186/183·신
186/183, 안정 공통 1,484개 분류 차이 0으로 이번 실행 gate를 통과했다. 미대응 관측은
주/음영 각각 1,001/704개이며 성공으로 계산하지 않는다. 이전 실패도 그대로 보존한다.

별도 음영 trace 실행의 신 두 번째 실행은 처리 LiDAR 169개를 모두 매칭했다. 기준 구 첫
실행의 처리 183·매칭 181 대비 처리 -14, drop -1, pending -1이 매칭 -12를 설명한다.
timeout 손실은 0이고 실제 처리 생략 기록이 증가했다. 보넷 제거·추적 연속성 개선 때문에
카운트가 줄었다는 근거가 아니다. trace 비용을 포함한 진단 결과를 추적 OFF 수락으로
대체하지 않는다. 주·음영 trace 8개는 실제 C++ 연속 재현에서 매칭·분류 상태를 재현했다.

숫자 track ID 비교는 진단으로 보존하고, 분류 gate는 같은 LiDAR stamp·상호 유일한
3D 중심 1mm 이내로 대응한 안정 관측을 사용한다. 위치 대응이 안 되는 관측을 같은
객체로 단정하지 않으며, 개별 old/new 분류 차이도 원본 trace에 보존한다.

승인 대상은 source stamp `1790654794688000000`의 unknown/car 차이와
`1790654812792000000`의 car/unknown 차이다. 각 실패의 정확한 원인을 하드웨어 부하로
확정하지 않는다. 기존 facade 실패 1건도 남긴다. 테스트는 판제 v2 launch의
`traffic_light` 기본값 false를 기대하지만 실제 값은 true이며, 모델 추론 실패가 아니다.
카메라에서 신호등 생산자를 실행할 때는 판제 launch에 `traffic_light:=false`를 지정한다.
