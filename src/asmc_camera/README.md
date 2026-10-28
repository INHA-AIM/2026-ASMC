# asmc_camera

YOLO 객체 인지·TwinLite 차선 영상 처리·ResNet 신호등 인지를 소유합니다.
실행은 `camera.launch`, 시각화는 `view.launch`를 사용합니다.
기본 객체 모델은 **YOLO26l collect v5 E3·15종**입니다. 이전 v11 E8·16종과 사전학습 YOLO26l은 명시적으로 선택하는 비교·복구용입니다.
현재 정본 목표는 YOLO15Hz·TwinLite15Hz·ResNet10Hz이며 confidence0.40·NMS IoU0.50입니다.
현재 기본 tensor는 두 모델 모두 전방544×960·좌우480×640입니다. 모델별 독립 프로세스를 사용하며 입력 tensor를 공유하지 않습니다.
TwinLite 기본 모델은 **Large v7 E8·TensorRT FP16**입니다.
2026-10-18 사용자가 알려진 SUNNY freshness·공급 지연 제한을 확인한 뒤 기본 전환을 승인했습니다.
같은 Large의 PyTorch 복구와 기존 Medium v6 TensorRT 복구 설정을 보존합니다. 자동 수락 실패 기록과350ms 기준은 유지합니다.
[선정·동일 평가·실시간 검증](../../docs/records/camera/2026-10-18-twinlite-large-deployment.md)에 배포 SHA와 한계를 기록합니다. 변환 품질 검증은
[TensorRT 검증 기록](../../docs/records/camera/camera-twinlite-tensorrt.md)에서 관리합니다.
[음영 family 최적화](../../docs/records/camera/camera-shadow-family-optimization.md)는
family 전용 경로·수요 기반 BEV·GC 진단과 후속 수락 상태를 관리합니다.
이전 음영 반복12회와10분 연속 시험의 순간 family freshness/공백 실패는 과거 증거로 보존합니다.
현재 기본 전환과 전체 공급 수락은 별도 결정이며, 최신 결과·직접 시험 명령은
[family 공급 최적화](../../docs/records/camera/camera-family-supply-optimization.md)를 따릅니다.
2026-10-16 사용자 직접 주행·시각화 확인 후 현재 리팩토링의 main 병합 승인을 받았습니다.
자동 시험의 순간 공백/freshness는 알려진 제한으로 남기며, 병합을 전체 자동 수락으로 표시하지 않습니다.
같은 문서에 bus/truck의 간헐적 누락, 차로 앞 bicycle/dog, dog 하위 종류별 약세와 판제 요청 후 추가학습 결정을 기록했습니다.
YOLO/TwinLite의 최소 공급10Hz와 목표15Hz 달성은 별도로 검증합니다. 같은 목표 Hz가 같은 영상 시각을 보장하지는 않습니다.

```bash
# asmc-ros-noetic에서 빌드한 workspace 환경을 source한 뒤
roslaunch asmc_camera camera.launch                        # 세 모델 상시 ACTIVE
roslaunch asmc_camera camera.launch mode:=standby          # 명시적인 대기 실행; 둘 중 하나만 실행
roslaunch asmc_camera camera.launch mode:=active models:=objects
# 기존 실행을 종료한 뒤 사전학습 모델 대조
roslaunch asmc_camera camera.launch mode:=active models:=objects detector:=pretrained
```

배포 E3 객체 엔진은 RTX4060, Large E8 차선 엔진은 RTX4060Ti에서 생성했습니다.
다른 GPU에서는 동일 checkpoint로 로컬 엔진을 생성·검증합니다. 직접 `camera.launch`도
검증된 로컬 객체 엔진을 선택하며 배포 가중치·체크섬을 수정할 필요가 없습니다.
[GPU별 생성·선택·복구](../../docs/operations/local-lane-engine.md)를 따르세요.

별도 터미널:

```bash
roslaunch asmc_camera view.launch models:=objects
rosrun rqt_image_view rqt_image_view --clear-config /perception/camera/objects/debug/mosaic/compressed
```

Family 선 화면은 실제 family가 도착하면 즉시 갱신합니다. 신호등도 결과 도착 즉시 표시합니다.
큰 객체/RGB-semantic 합성과 RGB BEV 비교는 최대10Hz로 제한하고 최신 대기 결과 하나만 유지합니다.
구독자가 없으면 해당 영상 처리를 중단하며, 시각화가 추론/판제 발행을 기다리게 하지 않습니다.
현재 정본 `cameras_v3.yaml`의 지면은 MORAI VehicleSpec wheelRadius에 맞춘 **z=-0.36m**입니다.
카메라 장착 위치는 바꾸지 않았습니다.
[원본 시각 IMU 보정](../../docs/architecture/camera-imu-projection.md)과 관측 폭·공통 곡선의 공동 피팅이 기본입니다.
IMU roll/pitch는 model-grid의 지면 근거와 family xyz에 적용합니다. Comparison RGB·semantic BEV도
family의 같은 프레임별 보정값을 재사용하며 IMU 조회·추론·피팅을 다시 실행하지 않습니다.
평면 수학과 프레임별 선택은 공용 `runtime/ground_projection.py`에 있습니다.
BBox lane 어댑터도 같은 영상의 평면 metadata를 재사용합니다.
[공용화 범위와 소비 검증](../../docs/records/camera/2026-10-18-imu-bbox-projection.md)을 참고합니다.
기존 BEV 데이터 토픽은 고정 평면 계약을 유지합니다.
자세가 없거나 시각 대응이 불확실하면 묶음 전체를 기존 고정 평면으로 처리합니다.
[잔차·공급 검증](../../docs/records/camera/2026-10-18-joint-family-stabilization.md)에
동일 입력 비교·10분 수락과 남은 M0 전환 한계를 기록했습니다. 복구는 기존 실행을 종료한 뒤
`roslaunch asmc_camera _imu_compare.launch enabled:=false`이며 명목 거리는 그대로0.36m입니다.
[0.36m·표시 경로 검증](../../docs/records/camera/2026-10-18-nominal-ground-and-arrival-view.md)에
정차 지면 채택 근거와 표시 부하의 남은 제한을 기록합니다.

[실행·모드·관측 계약](../../docs/contracts/camera-runtime-v1.md)에 공개 옵션과
새 제어 경로·기존 alias·프로필별 원본 클래스/세 그룹·관측 freshness를 정리했습니다.
[모델 설치와 소비 규칙](../../docs/operations/camera-deployment.md),
[실측·후처리 선정](../../docs/records/camera/camera-runtime-structure-validation.md)을 함께 확인하세요.
[15Hz·confidence 후속 검증](../../docs/records/camera/camera-rate-confidence-followup.md)에 최신 결정과 이전 실패를 정리했습니다.
[시각화·readiness 검증](../../docs/records/camera/camera-visualization-validation.md)은 단일 렌더링 경로와 GUI 동시 부하 결과를 관리합니다.
단일 렌더러는 PR #85의 현재 실행 경로입니다. 이전 입력의30분 시험과 현재 입력의10분 시험은 조건별 증거로 보존하며 최신 차단 상태는 위 family 공급 최적화 문서를 따릅니다.
[전방960 검증](../../docs/records/camera/camera-front960-validation.md)은 별도 결과이며 이전 입력의 통과를 재사용하지 않습니다.
기본은 세 모델 상시 추론·발행입니다. 구간에 따라 생산자를 끄지 않으며 판제가 결과 소비를 결정합니다.
[구간별 소비 안내](../../docs/handoffs/perception-to-planning.md)에 A153 끝까지의 차량 구간,
A126 진입 전 family 준비와 신호등 범위를 정리했습니다. 차선의 주행 소비는 음영 ODD에 한정합니다.
신호등 추론·발행도 카메라 담당의 단일 생산자로 유지합니다. 판제 launch는 별도 모델을 시작하지 않아
`traffic_light:=false`가 필요하지 않습니다. 실행 중 중단·재개는 카메라의 기존 `set_mode`를 사용합니다.

세부 모델·입력·mask 설정은 `config/camera/*_runtime.yaml`과 선택한 모델 프로필 YAML에서 관리합니다.
현재/실험/호환 설정의 구분은 [config 안내](config/README.md), 실제 가중치와 로컬 연결은 [models 안내](models/README.md)를 참고하세요.
일반 실행에서 실험용 batch·모델 경로·디버그 옵션을 반복 지정할 필요가 없습니다.
기존 `perception.launch`·`camera_mission16.launch` 및 실험 launch는 호환·재현 목적으로 유지합니다.
새 실행과 동시에 켜서 생산자를 중복으로 만들지 않습니다.

학습·데이터 감사·수집 도구는 오프라인 경로입니다. 실행 구조는 데이터·차량 제어·
LiDAR fusion 정책을 바꾸지 않습니다. 현재 asmc_lidar는 3뷰 관측을 수신해 실제 matching·분류를 수행합니다.

LiDAR 연결은 [현재 인계](../../docs/handoffs/camera-to-lidar.md)와
[3뷰 계약](../asmc_lidar/docs/three-view-fusion.md)을 따릅니다. 구 Main은 폐기됐으며 별도 실행하지 않습니다.

정본 상태/모드 타입은 `asmc_msgs/InferenceStatus`, `asmc_msgs/SetInferenceMode`다.
상태의 session UUID는 객체 관측과 같으며 기존 `asmc_camera` 타입은 구형 노드 제어 경로에 보존한다.
정본 경로 소비자는 공용 타입으로 재빌드하고 카메라·LiDAR 소비자와 함께 재시작한다.

공식 BDD100K Large의 추가학습 전 결과는 [별도 추론 실험](docs/bdd-pretrained-experiment.md)으로
확인한다. `bdd_lane.launch`·`bdd_lane_view.launch`는 이진 drivable/lane 시각화만 제공하며
운영 family 입력과 상태 토픽에 결과를 섞지 않는다. 운영 기본 모델은 변경하지 않는다.

이후 Capture 전체를 사용하는 Large 추가학습은 [데이터 준비 안내](docs/training-data.md)를 따른다.
전방960×544·좌우640×480, 배치 계보별 train/val/test와 조건 균형을 준비하며
2026-10-17 승인 후 [Large 학습과 모니터링](docs/training-run.md)을 시작했다.
2026-10-18 완료된8epoch 중E8을 선정하고 품질·TensorRT·정차 통합 시험을 완료했다.
FOGGY 전체 GUI10분은 통과했으나 교차12회 중 Large SUNNY1회에서 readiness 저하3건이 남았다.
후속 사용자 결정으로 `lane_runtime.yaml`을 Large E8 기본으로 전환했다.
`lane_runtime_large.yaml`은 같은 모델의 명시적 선택이며 `lane_runtime_medium.yaml`은 복구 설정이다.
위 학습·선정 당시의 병합 전 기록과 달리 현재 main에는 Large E8 기본 설정이 반영되어 있다.
