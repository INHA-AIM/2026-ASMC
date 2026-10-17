# 카메라 설정 안내

`camera.launch`가 현재 실행의 시작점입니다. config 파일 전체를 순서대로 읽는 구조가
아니며, launch에서 선택한 파일만 사용합니다.
기본은 세 모델 상시 ACTIVE·목표15/15/10Hz입니다. 구간은 생산자 on/off 대신
[판제 소비 안내](../../../docs/handoffs/perception-to-planning.md)로 관리합니다.

| 설정 | 현재 역할 |
|---|---|
| `runtime.yaml` | 실행·소유권·공급 기준 안내 |
| `runtime/camera_region_example.yaml` | 기본 미실행인 제어 예제. 현재 설정은 GPS/구간과 무관하게 세 모델 ACTIVE 요청 |
| `camera/objects_runtime.yaml` | 두 객체 모델이 공유하는 mask·보정·발행 설정 |
| `camera/yolo26l_mission16_e8_three_view.yaml` | 기본 E8, 전방960·좌우640 |
| `camera/yolo26l_pretrained_mission_three_view.yaml` | 실행 시 선택하는 사전학습 COCO7, 전방960·좌우640 |
| `camera/object_classes_mission16_v1.yaml` | 원본 이름→세 소비 그룹 |
| `camera/lane_runtime.yaml` | Medium v6 기본 TensorRT FP16·GPU 전처리·수요 기반 출력·고정 engine manifest |
| `camera/lane_runtime_pytorch.yaml` | 같은 Medium·입력 크기의 명시적 PyTorch 복구 설정. 내부 `_runtime.launch lane_runtime_config:=...`로 선택 |
| `camera/lane_runtime_large.yaml` | Large v7 E8 TensorRT 명시적 시험 후보. SUNNY source freshness 실패로 기본 승격 보류 |
| `camera/lane_runtime_medium.yaml` | 기존 Medium v6 TensorRT의 명시적 복구 설정. 노드 종료 후 내부 `_runtime.launch lane_runtime_config:=...`로 선택 |
| `camera/lane_runtime_tensorrt.yaml` | 기존 T2 실험 재현 설정. 현재 실행 정본은 `lane_runtime.yaml` |
| `camera/lane_input.yaml` | TwinLite 전방544×960·좌우480×640 후보. 추론과 model-grid 투영이 함께 읽음 |
| `camera/lane_input_side384.yaml` | 전방960·기존 좌우384 비교/복구용 |
| `camera/twinlite_medium.yaml` | TwinLite backend와 구형 launch의 공통 기본값 |
| `camera/lane_bdd_large.yaml`, `camera/lane_input_bdd_official.yaml` | 공식 BDD Large의 이진 출력 실험. 운영 family와 분리한 `bdd_lane.launch` 전용 |
| `camera/traffic_light_runtime.yaml` | camera4 ResNet 설정 |
| `camera/visualization.yaml`, `bev/visualization*.yaml` | 시각화 설정 |
| `cameras_v3.yaml`, `bev/*` | 보정·BEV·차선 처리. 입력 tensor 변경으로 센서 보정을 바꾸지 않음 |

`camera/yoloe_l/`, `camera/yoloe_selection/`, M/L·detect/seg 비교 프로필,
`*_experiment.yaml`과 마스크의 v1/neutral_v2는 과거 비교·복구 경로입니다.
예를 들어 `yolo26l_detect_coco7_three_view.yaml`의 전방1280은 이전 비교 설정이며
현재 `detector:=pretrained`가 선택하는 파일은 `yolo26l_pretrained_mission_three_view.yaml`입니다.
YOLO11m은 구형 호환 경로이고 기본 모델이 아닙니다. `offline/`은 학습·평가용입니다.

과거 버전을 모두 복제하는 방식은 사용하지 않습니다. 일반적인 코드·설정 변경 이력은
Git으로 보존하고, 재현에 필요한 실험 프로필·SHA·실패 증거만 명시적으로 남깁니다.
이 저장소도 현재 설정, 선택 가능한 모델, 호환 경로, 재현용 실험이 공존합니다.
`v1/v2`는 각 파일의 규칙·보정·실험 버전이므로 가장 큰 번호가 항상 운영 기본은 아닙니다.
과거 설정을 이동·삭제하려면 인계 문서와 시험의 참조부터 함께 정리해야 합니다.

960 변경의 검증 상태와 기존 실패는
[실행 계약](../../../docs/contracts/camera-runtime-v1.md)과
[960 검증 기록](../../../docs/records/camera/camera-front960-validation.md)을 따릅니다.
