# asmc_camera

YOLO 객체 인지·TwinLite 차선 영상 처리·ResNet 신호등 인지를 소유합니다.
실행은 `camera.launch`, 시각화는 `view.launch`를 사용합니다.
기본 객체 모델은 YOLO26l v11 E8, 사전학습 YOLO26l도 실행 시 선택할 수 있습니다.
현재 정본 목표는 YOLO15Hz·TwinLite15Hz·ResNet10Hz이며 confidence0.40·NMS IoU0.50입니다.
전방 tensor는 두 모델 모두544×960입니다. 좌우 tensor는 두 모델 모두480×640인 PR #85 후보입니다.
TwinLite는 독립 PyTorch 경로를 유지하며, TensorRT 후보의 품질·공급 검증은
[TensorRT 검증 기록](../../docs/handoffs/perception/camera-twinlite-tensorrt.md)에서 관리합니다.
YOLO/TwinLite의 최소 공급10Hz와 목표15Hz 달성은 별도로 검증합니다. 같은 목표 Hz가 같은 영상 시각을 보장하지는 않습니다.

```bash
# asmc-ros-noetic에서 빌드한 workspace 환경을 source한 뒤
roslaunch asmc_camera camera.launch                        # 세 모델 STANDBY
roslaunch asmc_camera camera.launch mode:=active models:=objects
# 기존 실행을 종료한 뒤 사전학습 모델 대조
roslaunch asmc_camera camera.launch mode:=active models:=objects detector:=pretrained
```

별도 터미널:

```bash
roslaunch asmc_camera view.launch models:=objects
rosrun rqt_image_view rqt_image_view --clear-config /perception/camera/objects/debug/mosaic/compressed
```

[실행·모드·관측 계약](../../docs/contracts/perception/camera-runtime-v1.md)에 공개 옵션과
새 제어 경로·기존 alias·원본16종/세 그룹·관측 freshness를 정리했습니다.
[모델 설치와 소비 규칙](../../docs/handoffs/perception/camera-mission16-deployment.md),
[실측·후처리 선정](../../docs/handoffs/perception/camera-runtime-structure-validation.md)을 함께 확인하세요.
[15Hz·confidence 후속 검증](../../docs/handoffs/perception/camera-rate-confidence-followup.md)에 최신 결정과 이전 실패를 정리했습니다.
[시각화·readiness 검증](../../docs/handoffs/perception/camera-visualization-validation.md)은 단일 렌더링 경로와 GUI 동시 부하 결과를 관리합니다.
단일 렌더러는 PR #85의 후보 경로입니다. 이전 입력의30분 시험에서 차선·신호등 공백이 있어 main 승격과 PR Ready를 보류했습니다.
[전방960 검증](../../docs/handoffs/perception/camera-front960-validation.md)은 별도 결과이며 이전 입력의 통과를 재사용하지 않습니다.
판제는 구간·활성화·소비를 결정하며 차선의 주행 소비는 음영 ODD에 한정합니다.

세부 모델·입력·mask 설정은 `config/camera/*_runtime.yaml`과 선택한 모델 프로필 YAML에서 관리합니다.
현재/실험/호환 설정의 구분은 [config 안내](config/README.md), 실제 가중치와 로컬 연결은 [models 안내](models/README.md)를 참고하세요.
일반 실행에서 실험용 batch·모델 경로·디버그 옵션을 반복 지정할 필요가 없습니다.
기존 `perception.launch`·`camera_mission16.launch` 및 실험 launch는 호환·재현 목적으로 유지합니다.
새 실행과 동시에 켜서 생산자를 중복으로 만들지 않습니다.

학습·데이터 감사·수집 도구는 오프라인 경로입니다. 이 실행 변경은 데이터·가중치·차량 제어·
LiDAR fusion 정책을 바꾸지 않습니다. 3뷰 관측 메시지는 후속 LiDAR 인계를 위한 생산자 계약이며
현재 3뷰 LiDAR fusion이 연결됐다는 의미는 아닙니다.
