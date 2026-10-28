# 배포 모델

현재 패키지는 객체 검출 **YOLO26l E3·E8**, 차선 **TwinLite Large v7 E8**,
신호등 **ResNet `traffic_light/best.pt`**를 배포한다.

| 선택 | 패키지 파일 | 용도 |
|---|---|---|
| `detector:=e3` 기본 | `object_detection/yolo26l-mission15-e3.pt`, `.engine` | collect v5 추가학습 15종 |
| `detector:=e8` | `object_detection/yolo26l-finetuned_e8.pt`, `.engine` | 이전 16종 복구·대조 |
| TwinLite 기본 | `lane_segmentation/twinlite-large-v7-e8.pt`, `twinlite_large_v7_e8_{front,sides}.engine` | 전방 544×960 / 좌우 480×640 |
| 신호등 | `traffic_light/best.pt` | 별도 ResNet 생산자 |

객체 [배포 목록 v3](../../../config/models/camera_object_models_v3.json),
차선 [배포 목록](../../../config/models/camera_lane_large_v7_e8_v1.json),
[GPU별 engine 준비와 운영](../../../docs/operations/camera-deployment.md)을 따른다.
지원 GPU에서 checkpoint·engine의 SHA와 실행 환경을 확인하며, 다른 GPU에서는 해당 checkpoint로
engine을 다시 만든다. 개인 GPU용 `.runtime/` 결과는 Git 배포 파일과 구분한다.

ROS는 기본적으로 패키지 안의 SHA 고정 `.engine`을 사용합니다. 같은 모델의 검증된 개인 GPU 엔진은
[로컬 선택](../../../docs/operations/local-lane-engine.md)으로 사용할 수 있으며 배포 파일을 덮어쓰지 않습니다.
각 배포 engine은 생성본과 바이트가 같으며
dynamic profile이 전방544×960·좌우480×640과 batch1/2/3을 지원합니다.
내부 FP16 계산과 engine I/O dtype은 구분하며 실제 binding dtype을 따릅니다.
E3 배포 engine의 생성 환경은 Linux x86_64·RTX4060 8GB·TensorRT10.13.0.35 CUDA11·`asmc-ros-noetic`입니다.
PyTorch2.1.2+cu118·Ultralytics8.4.150을 사용하며, 기존 RTX5070 생성본을 같은 E3 checkpoint로 다시 변환합니다.
기존 E8 engine의 생성 환경은 RTX4060Ti 8GB입니다.
다른 GPU/runtime에서 호환성을 보장하지 않습니다. 불일치는 오류로 드러내고 PyTorch로
자동 전환하지 않습니다. 다른 환경에서는 패키지 `.pt`로 engine을 별도 생성·검증합니다.

TwinLite는 TensorRT 실행에서도 checkpoint SHA를 검사하므로 Large E8 `.pt`를 함께 유지한다.
`lane_runtime_pytorch.yaml`은 **동일 Large E8**의 PyTorch 복구 경로다.
학습 재개에는 D의 원본 full checkpoint를 사용하며 이 패키지의 추론 export로 대체하지 않는다.
Large의 고정 upstream 소스와 라이선스는 PyTorch·변환 경로에 계속 필요하다.

YOLO11m, COCO pretrained 비교, BDD 비교, Medium v6 배포본과 ONNX 중간 산출물은 사용자 요청으로
현재 Git tree에서 제거했다. 파일별 SHA·크기·복구할 main 커밋은
[Git 이력 자산 목록](../../../config/artifacts/git-history-assets-v1.json)에 있다.
`detector:=pretrained`는 현재 실행 선택에서 제외했다. 과거 실험 설정·평가 수치·source metadata는
재현 근거이며, 배포되지 않는 가중치를 현재 사용 가능하다고 뜻하지 않는다.
옛 BDD/COCO 실험을 재현하려면 [이력 복구 안내](../../../docs/operations/asset-history.md)에 따라
해당 커밋의 별도 작업트리를 사용한다. D의 원본·복구·학습 자료는 이번 정리에 포함하지 않았다.
