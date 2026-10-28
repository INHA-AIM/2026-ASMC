# 카메라 실행·3뷰 관측 계약 v1

현재 실행 정본은 `asmc_camera/camera.launch`, 시각화 정본은 `view.launch`다.
음영 lane의 실행·수요와 최신 검증 상태는 [family 공급 최적화 인계](../records/camera/camera-family-supply-optimization.md)를 따른다.
E3(기본)·E8·사전학습 YOLO26l을 실행 시 선택하며 가중치 교체에는 노드 재시작이 필요하다.
기계 정본은 `config/contracts/camera_object_observation_v1.yaml`이다.

## 실행과 소유권

```bash
# 기본: 세 모델을 로드·예열하고 상시 ACTIVE, 추론·발행
roslaunch asmc_camera camera.launch
# 명시적인 대기 실행: 위 실행을 종료한 뒤 선택
roslaunch asmc_camera camera.launch mode:=standby
# 객체 인지 ACTIVE; 주행 제어는 시작하지 않는다
roslaunch asmc_camera camera.launch mode:=active models:=objects
# 사전학습 대조: 기존 실행을 종료한 뒤 시작
roslaunch asmc_camera camera.launch mode:=active models:=objects detector:=pretrained
# 세 모델 모두 ACTIVE
roslaunch asmc_camera camera.launch mode:=active
```

공개 옵션은 `mode:=off/standby/active`, `models:=all` 또는 쉼표로 연결한
`objects,lane,traffic_light`, `detector:=e3/e8/pretrained`, `confidence`, `nms_iou`다.
객체 기본은 collect v5 E3의15종 모델이다. `traffic_cone`을 제외하고 `stroller`는 모델 ID14다.
외부 소비에는 숫자 ID 대신 원본 이름과 기존 세 그룹을 전달한다.
현재 정본은 YOLO15Hz·TwinLite15Hz·ResNet10Hz, confidence0.40·동일 클래스 NMS0.50이다.
2026-10-15 사용자 요청에 따라 전방 tensor는 YOLO/TwinLite 모두544×960으로 변경했다.
현재 TwinLite 좌우480×640과 YOLO의 B1/B2 shape가 같다.
기존 좌우384×640은 `lane_input_side384.yaml`에 보존한다. 센서 RGB·원본 좌표·header·보정은 그대로다.
TwinLite는 한 checkpoint의 같은 크기 입력끼리 나눠 추론하며, 전방 확대와 YOLO 축소의
통합 효과는 [960 검증 기록](../records/camera/camera-front960-validation.md)에서 별도로 판정한다.
이전1280/640 결과를 새 입력의 성능 수락으로 사용하지 않는다.
YOLO/TwinLite의 완전한5초 창 최소10Hz와 목표15Hz의95% 달성은 별도로 보고한다.
입력이 끊기면 목표를15Hz로 높여도 결과 공급을 보장할 수 없다.
같은 목표 Hz는 같은 입력 프레임·발행 위상을 의미하지 않으며 소비자는 원본 stamp로 연결한다.
`off`는 모델과 차선 downstream을 만들지 않는다. 이미 다른 launch로 실행 중인 노드를
종료하는 명령은 아니다. `standby`는 모델 로드·dummy 예열 이후 입력 추론 없이 대기한다.
PREPARE는 서비스로 요청한다. ACTIVE는 새로운 영상만 처리하며 결과 재발행으로 Hz를 높이지 않는다.

`objects_runtime.yaml`, `lane_runtime.yaml`, `lane_input.yaml`, `traffic_light_runtime.yaml`과 모델 프로필 YAML이
mask·보정·입력·모델 경로·상세 설정을 소유한다. 공개 옵션으로 매번 지정할 필요가 없다.
ResNet은 camera4의 기존 PIL RGB·GPU resize·bfloat16 경로를 유지한다.
`lane_input.yaml`은 TwinLite와 model-grid 투영이 함께 읽는 단일 입력 크기 설정이다.
YOLO 배포 checkpoint2개·engine2개는 패키지 models의 실제 Git 파일이다.
[모델 배포 안내](../../src/asmc_camera/models/README.md)의 지원 GPU/runtime과 SHA를 따른다.
TwinLite Large v7 E8 checkpoint·전용 engine과 기존 Medium v6 복구본을 패키지에 포함한다. 2026-10-18 사용자 결정으로 기본 모델을 Large v7 E8로 전환했다. SUNNY freshness·공급 지연 제한과 자동 수락 실패는 보존한다.
기본 backend는 `lane_runtime.yaml`의 TensorRT
FP16이다. 같은 입력의 PyTorch 복구는 내부 `_runtime.launch`의 `lane_runtime_config`로
`lane_runtime_pytorch.yaml`을 명시적으로 선택한다. Medium 복구는 `lane_runtime_medium.yaml`이다.
[Large E8 배포 검증](../records/camera/2026-10-18-twinlite-large-deployment.md)에 선정·SHA·정차 시험과 한계를 기록한다.
Engine 오류에 자동 fallback하지 않는다.
기본 backend 선택은 전체 공급 수락을 뜻하지 않는다. 변환 품질·과거 후단 실패는
[TensorRT 검증 기록](../records/camera/camera-twinlite-tensorrt.md)을 따른다.
공유 GPU 입력은 내부 `_runtime.launch implementation:=shared shared_gpu_input:=true` 후보이며
기본은 모델별 프로세스다. 공유 후보에서도 camera4 ResNet은 별도 프로세스를 사용한다.
구형 `perception.launch`의 TwinLite384×640 호환 경로는 보존한다.
TwinLite·BEV·차선 피팅의 책임은 분리하며 차선 주행 소비는 음영 ODD에 한정한다.
원본 시각 IMU·공동 폭 피팅이 기본이며 근거 부족 시 전 뷰가 기존 M0로 복귀한다.
[보정 계약](../architecture/camera-imu-projection.md)과 [최종 검증](../records/camera/2026-10-18-joint-family-stabilization.md)을 따른다.
Header·표현·MD5는 유지하지만 설정 SHA는 바뀐다. 구 SHA를 고정한 판제 어댑터의 인수는
[인계 문서](../handoffs/camera-to-planning.md)에 따라 별도로 확인한다.
구간 판정과 결과 소비는 판제 소유다. 현재는 세 모델을 상시 ACTIVE로 유지하며
구간·GPS 유효 여부로 발행을 중단하지 않는다. Frenet 객체 freshness를 유지하고,
[구간별 소비 안내](../handoffs/perception-to-planning.md)에 따라 판제가 사용 범위를 결정한다.
A153 끝/A451 진입까지 차량 소비, A126 진입 전 family 준비를 구분한다.
구형 `perception.launch profile:=planning`과 낮은 수준의 managed STANDBY 기본은 유지한다.

## 제어와 호환

| 역할 | 새 서비스 / 상태 | 기존 alias 루트 | status.model |
|---|---|---|---|
| objects | `/perception/camera/objects/set_mode`, `/status` | `/camera_object_detector` | `yolo` |
| lane | `/perception/camera/lane/set_mode`, `/status` | `/camera_semantic_inference` | `twinlite` |
| traffic_light | `/perception/camera/traffic_light/set_mode`, `/status` | `/traffic_light_infer` | `resnet18` |

표의 `/status`는 각 역할의 새 루트 뒤에 붙인다. 기존 루트 뒤의
`/set_inference_mode`, `/inference_status`는 같은 제어 객체·세대의 alias다.
별도 중계 노드·상태 머신·추론 생산자는 생기지 않는다. Main의 YOLO 상태 구독과
차선 `LaneEpochGate`의 구독도 정본으로 전환한다. 정본은 `asmc_msgs/InferenceStatus`와
`asmc_msgs/SetInferenceMode`이며 상태에 `producer_session_id`를 추가했다. 기존 `asmc_camera`
타입의 필드·MD5는 구형 경로에 보존한다. 정본 상태는 별도 타입/MD5이므로 카메라·Main·소비자를
같은 변경으로 재빌드/재시작한다. 구형 타입을 쓰는 소비자는 구형 경로를 사용하거나 공용 타입으로
수정한다. [공용 제어 계약](../../config/contracts/camera_inference_control_v2.yaml)을 따른다.

다음은 명시적으로 STANDBY를 선택한 경우의 복구 절차다. 기본 실행에는 필요하지 않다.

```bash
rosservice call /perception/camera/objects/set_mode '{mode: 1, target_hz: 0.0}'
rostopic echo /perception/camera/objects/status
# 같은 generation에서 prepared=true 확인 후
rosservice call /perception/camera/objects/set_mode '{mode: 2, target_hz: 0.0}'
```

mode 0/1/2는 STANDBY/PREPARE/ACTIVE다. `target_hz: 0.0`은 현재 주기를 보존한다.
objects status의 processing_hz는 여러 뷰의 완료 수를 합친 값일 수 있다.
뷰별 15Hz 확인에는 observations 또는 diagnostics의 output_by_view를 사용한다.

PREPARE 응답의 accepted는 실제 영상 준비 완료가 아니다. 준비에는 각 뷰의 새 영상이
필요하며 차선은 `pipeline_ready`까지 확인한다. 한 뷰가 멈추면 객체 인지는 다른 뷰를
계속 처리할 수 있지만 전체 prepared는 false가 된다. TwinLite의 기존 3뷰 coherence는 유지한다.

기존 `perception.launch`와 `camera_mission16.launch`는 호환 경로다. 기존 profile·옵션과
시각화 시작 동작을 보존한다. 새 실행법과 동시에 켜지 않는다.

## 관측과 무효화

기존 `/perception/camera/objects` (`CameraObjectBoxArray`)는 유지한다.
같은 계산 결과에서 새 `/perception/camera/objects/observations`
(`CameraObjectObservation`)를 카메라별로 한 번 발행한다. 큰 영상 복제 토픽은 추가하지 않는다.

| 필드 | 의미 |
|---|---|
| `detections` | 원본 영상 좌표의 bbox 배열, 크기·header·valid·원본 라벨·그룹·confidence |
| `producer_session_id` | 객체 생산자 시작마다 새로운 UUID; 세대 값만으로 재시작을 구분하지 않음 |
| `generation` | 결과 발행과 같은 잠금 안에서 확정한 모델 모드 세대 |
| `calibration_id` | 해당 카메라의 보정 설정 및 센서 설정 SHA를 묶은 SHA-256 식별자 |
| `model_id` | 실제 engine/checkpoint SHA-256 |
| `config_id` | 시작 시 처리 설정과 mask 파일 SHA의 식별자 |

카메라 ID와 영상 시각은 `detections.header.frame_id/stamp`, 크기는 내부 배열이 정본이다.
객체 식별에 ROS publisher의 `header.seq`를 사용하지 않는다. top-level legacy 배열의 seq는
ROS가 발행 순번으로 할당하며, 새 관측의 내부 header는 발행 직전 원본 snapshot을 보존한다.
서비스로 바뀐 주기·모드는 status와 generation으로 확인한다. config_id는 시작 설정의 ID다.

- `valid=true, boxes=[]`: 새 입력의 정상 빈 검출. 이전 비어 있지 않은 결과로 되돌리지 않는다.
- `valid=false`: decode/추론 오류 또는 모드 전환에 따른 무효 통지. 새 정상 관측으로 세지 않는다.
- STANDBY/PREPARE 전환에는 이미 출력한 뷰를 새 세대로 무효화한다. 첫 입력 전에는 header를 만들지 않는다.
- 생산자 종료·입력 중단은 status/source freshness timeout으로 확인한다. 새 세션·세대는 과거 결과를 버리는 경계다.
- stamp 회귀는 기존 상태 머신이 세대를 바꾼다. 이전 세대의 진행 중 결과는 발행하지 않는다.

현재 LiDAR 버퍼의 키는 session·generation·카메라·stamp·calibration을 포함한다.
`asmc_lidar`의 기본은 3뷰 matching이며 동일 객체의 여러 뷰를 scan별로 통합해
분류 확인 횟수를 중복 계산하지 않는다. 구 전방 bbox adapter와 `/YOLO/BBox`는 폐기했다.
자세한 소비·검증 범위는 [3뷰 융합 문서](../../src/asmc_lidar/docs/three-view-fusion.md)를 따른다.

## 시각화

```bash
# 별도 터미널. view.launch는 추론 생산자를 시작하지 않는다.
roslaunch asmc_camera view.launch models:=objects
# 또 다른 터미널
rosrun rqt_image_view rqt_image_view --clear-config /perception/camera/objects/debug/mosaic/compressed
```

객체 시각화는 좌/전방/우를 한 노드에서 직접 합성하고 JPEG를 한 번 인코딩한다.
기존 개별/합성 토픽과 표시 순서를 유지하며 개별 뷰는 구독할 때만 추가 인코딩한다.
뷰별 최대8 JPEG·500ms 캐시와 기존 관측의 session·generation·카메라·stamp·설정 식별자로
정확한 영상 대응과 무효화를 관리한다. 합성 header는 디버그 발행 시각이며 동기 3뷰 관측이 아니다.
시각화 목표는10Hz이고 추론 목표15Hz와 별도다. 구독자가 없는 큰 영상의 구독·변환·인코딩을 중단한다.
최종 기본 전환과 GUI 수락 상태는 [시각화 검증 기록](../records/camera/camera-visualization-validation.md)을 따른다.

Family 선은 발행된 family를 그대로 그린다. RGB/semantic comparison은
[`CameraGroundProjection`](../../config/contracts/camera_ground_projection_v1.yaml)의 뷰별 지면을
공유한다. 원본 시각 IMU 조회는 family worker에서 한 번이며, 작은 metadata는 피팅 전에 전달해
영상 렌더링이 피팅 완료를 기다리지 않도록 한다. Comparison만 구독하면 별도의 고정 BEV 합성
경로를 구동하지 않는다. 원 JPEG와 같은 시각의 semantic이 없는 경우에는 대체하지 않고 건너뛴다.
기존 BEV 데이터 토픽·판제 좌표 계약은 변경하지 않는다.

## Timestamp의 출처

카메라는 bridge의 header stamp를 다시 만들지 않는다. 센서 촬영 시각이라는 보장은 없다.
`morai_sensor_bridge.timestamp_mode=receive`이면 bridge 수신 시각이다. `source`이면 유효한
패킷 sec/nsec를 사용하고, 형식 오류·역행 시 수신 시각으로 fallback하며 진단 카운터를 올린다.
이때 이전 값보다 작거나 같은 fallback은 1ns 증가시킨다. 실시간 측정에서는 실제 설정값과
fallback 진단을 함께 기록해야 한다. 지연 실측은 이 시각과 분리해 실제 입력 callback 진입부터
ROS 발행까지 monotonic wall 시간을 사용한다.

## 비교용 공유 프로세스

전방960 이후의 공유 입력 경계와 B1/B2 설계는 [공유 입력 설계안](../records/camera/camera-shared-input-design.md)에 있다.
GPU 입력·전송 공유 후보는 구현했으며, TensorRT 전환 후 T3 대조에서도 기본으로
채택하지 않았다. CPU 절감과 다른 모델의 지연 악화를 함께 판정하며 위 검증 기록에
실측과 보류 근거를 남긴다.

기본은 R1의 독립 프로세스다. 내부 `_runtime.launch implementation:=shared`는 R2 실험 경로다.
명시적인 모델별 Parameters namespace·독립 worker·모드·주기를 사용한다. 공유 입력은 토픽당
한 번 구독하고 최신 압축 영상을 전달한다. decode는 소비 시 수행하며 뷰당 두 슬롯의 lease
캐시를 사용한다. 슬롯이 모두 사용 중이면 처리 대상 프레임을 건너뛰고 입력 callback은 기다리지 않는다.
공유 RGB는 읽기 전용이며 YOLO mask는 작업 버퍼에만 적용한다.

치명적 CUDA 오류는 모든 모델을 STANDBY·무효 상태로 만들고 `runtime_restart_required`로 표시한다.
그 프로세스의 PREPARE는 거절된다. 재시작 후 새 세션과 PREPARE 확인이 필요하다.
R2는 CPU/p95 개선 및 다른 모델의 지연 기준을 통과하기 전 기본으로 승격하지 않는다.
실측과 중복 억제 판단은 [검증 기록](../records/camera/camera-runtime-structure-validation.md)을 따른다.
