# 모델 파일

실행 기본은 YOLO26l E8이며 `camera.launch detector:=pretrained`로 사전학습 YOLO26l을
선택합니다. 기본 YOLO26l checkpoint는 COCO80이고 실행에서 원본7종만 선택합니다.

큰 파일의 정본은 `${ASMC_MODELS_ROOT:-/data/models}`입니다. 컨테이너 `/data`는
호스트 workspace의 `data` 마운트입니다. Git에는 파일·SHA·배포 경로를 적은
[모델 목록](../../../config/models/camera_object_models_v1.json)과 설치 도구를 보관합니다.

| 모델 | 정본 폴더 | 패키지 안의 선택적 로컬 연결 |
|---|---|---|
| E8 추가학습 | `camera_mission16_v11_e8/yolo26l.pt`, `.engine` | `object_detection/yolo26l-finetuned_e8.pt`, `.engine` |
| 사전학습 COCO7 | `camera_coco7_v1/yolo26l.pt`, `.engine` | `object_detection/yolo26l-pretrained_coco7.pt`, `.engine` |

`.pt`는 학습·변환용 원본이며 현재 ROS 추론은 SHA가 고정된 `.engine`을 읽습니다.
960 입력은 기존 dynamic engine의 지원 범위 안이므로 모델 파일을 새로 만들지 않습니다.

로컬 연결은 중복 복사 없이 원본을 찾기 위한 상대 심볼릭 링크이며 Git에서 제외합니다.
설치 시 `tools/perception/install_camera_model_release.py --package-models
src/asmc_camera/models/object_detection` 옵션으로 만들 수 있습니다. 다른 기존 파일을
덮어쓰지 않으며 SHA를 먼저 확인합니다. 패키지만 복사하면 실제 가중치가 같이 이동하지
않으므로 팀원은 [모델 설치 안내](../../../docs/handoffs/perception/camera-mission16-deployment.md)를 따릅니다.

`object_detection/yolo11m.pt`는 기존 호환용 파일입니다. `traffic_light/best.pt`는
ResNet 신호등 모델이며 YOLO 선택과 관계없습니다. TwinLite checkpoint는
`config/camera/lane_runtime.yaml`에 지정된 기존 학습 산출물 경로에 있습니다.
