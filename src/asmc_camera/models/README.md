# 모델 파일

실행 기본은 YOLO26l E8이며 `camera.launch detector:=pretrained`로 사전학습 YOLO26l을
선택합니다. 사전학습 YOLO26l checkpoint는 COCO80이고 실행에서 원본7종만 선택합니다.

두 YOLO의 `.pt`·`.engine` 실제 배포 파일을 일반 Git에 포함합니다. 지원 GPU·컨테이너에서는
pull 후 별도 YOLO 다운로드 없이 실행합니다. [배포 목록 v2](../../../config/models/camera_object_models_v2.json)에
원본/배포 SHA·크기·E8 출처·engine 환경을 기록합니다. 원본 재개 checkpoint와 기존 release는
`/data/models`와 [원본 목록 v1](../../../config/models/camera_object_models_v1.json)에 보존합니다.

| 모델 | 보존한 원본 폴더 | 패키지 실제 배포 파일 |
|---|---|---|
| E8 추가학습 | `camera_mission16_v11_e8/yolo26l.pt`, `.engine` | `object_detection/yolo26l-finetuned_e8.pt`, `.engine` |
| 사전학습 COCO7 | `camera_coco7_v1/yolo26l.pt`, `.engine` | `object_detection/yolo26l-pretrained_coco7.pt`, `.engine` |

E8 `.pt`는 원본 EMA의 모든 parameter/buffer를 보존한53.07MB 배포본입니다.
optimizer·scaler·중복 training_model·scheduler_state를 제거하므로 학습 재개에는 원본
316MB checkpoint를 사용합니다. 사전학습 `.pt`는 COCO80 구조와 원본 바이트를 유지합니다.

ROS는 패키지 안의 SHA 고정 `.engine`을 사용합니다. 두 engine은 원본과 바이트가 같으며
dynamic profile이 전방544×960·좌우480×640과 batch1/2/3을 지원합니다.
내부 FP16 계산과 engine I/O dtype은 구분하며 실제 binding dtype을 따릅니다.
생성 환경은 Linux x86_64·RTX4060Ti8GB·TensorRT10.13.0.35·`asmc-ros-noetic`입니다.
다른 GPU/runtime에서 호환성을 보장하지 않습니다. 불일치는 오류로 드러내고 PyTorch로
자동 전환하지 않습니다. 다른 환경에서는 패키지 `.pt`로 engine을 별도 생성·검증합니다.

`tools/perception/package_camera_models.py`로 원본 SHA 확인·EMA 전용 직렬화·복사를
재현합니다. 기존 release 설치 도구로 이 파일을 심볼릭 링크로 바꾸지 않습니다.

`object_detection/yolo11m.pt`는 기존 호환용 파일입니다. `traffic_light/best.pt`는
ResNet 신호등 모델이며 YOLO 선택과 관계없습니다. TwinLite의 고정 upstream 소스는
PyTorch 복구 경로 및 export의 설치 의존성입니다. canonical `lane_runtime.yaml`은
동일한 SHA의 패키지 내부 checkpoint를 사용하며 원본 학습 산출물은 보존합니다.


## TwinLite TensorRT 후보

`lane_segmentation/`에는 Medium v6 원본 checkpoint, 전방544×960 B1 및 좌우480×640
B1–2 ONNX/FP16 engine, `manifest.json`을 둡니다. EMA·두 head·클래스 순서는 유지하며
attention을 근사하거나 제거하지 않습니다. Manifest의 checkpoint/engine SHA와 GPU,
TensorRT 버전을 시작 시 검사합니다. 불일치는 명시적으로 실패하며 fallback하지 않습니다.

이 파일은 실시간 수락을 위한 후보입니다. `lane_runtime.yaml`의 PyTorch 복구 경로와
고정 upstream 소스 의존성은 보존합니다. 변환에는 upstream 소스와 ONNX 도구가 필요하며,
TensorRT 실행 자체는 해당 upstream Python 모델을 로드하지 않습니다.
[변환·품질·실시간 검증](../../../docs/handoffs/perception/camera-twinlite-tensorrt.md)을 확인하세요.
