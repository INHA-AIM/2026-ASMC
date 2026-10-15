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
ResNet 신호등 모델이며 YOLO 선택과 관계없습니다. TwinLite checkpoint와 고정 upstream 소스는
기존 설치 의존성입니다. checkpoint는
`config/camera/lane_runtime.yaml`에 지정된 기존 학습 산출물 경로에 있습니다.
