# BDD100K Large 사전학습 추론 실험

> 현재 패키지는 BDD 가중치를 배포하지 않습니다. 아래는 당시 실험 기록입니다. 재현은 [Git 이력 자산 안내](../../../docs/operations/asset-history.md)의 별도 checkout에서 진행합니다.

공식 TwinLiteNet+ Large 가중치를 추가학습 없이 MORAI 영상에서 확인하는 경로다.
운영 `camera.launch`의 Medium v6 TensorRT·family·YOLO·신호등 기본값은 유지한다.
결과 기록은 [BDD Large 비교](../../../docs/records/camera/2026-10-17-twinlite-bdd-large.md)를 따른다.

## 모델과 의미

- 공식 [TwinLiteNet+ README](https://github.com/chequanghuy/TwinLiteNetPlus)의 pretrained 모델 폴더에서 `large.pth`를 받았다.
- 원본은 로컬 `data/models/twinlite_bdd_large_v1/large.pth`에 보존한다.
- 패키지의 `models/lane_segmentation/twinlite-large-bdd100k.pt`는 정본 로더용 metadata만 추가했다. 408개 tensor는 원본과 정확히 같다.
- 출처·SHA·공식 source commit·head 정의는 옆의 `twinlite-large-bdd100k.source.json`에 기록한다.
- 출력은 `drivable` 2종(background/drivable), `lane` 2종(background/lane)이다. 흰 차선·황색 차선·정지선 구분이 없다. 시각화의 하늘색은 **모든 차선**이다.
- 이진 출력을 MORAI 4종으로 바꾸거나 family 입력으로 사용하지 않는다. 재학습·TensorRT export는 이번 실험에 포함하지 않았다.

공식 입력은 384×640이고 RGB `/255`, OpenCV LINEAR resize, padding 114를 사용한다.
실험 기본은 운영과 같은 전방544×960·좌우480×640이며 공식 LINEAR 보간을 유지한다.
`resolution:=official`은 모든 뷰를 384×640으로 letterbox한다. 원본 크기와 header는 보존한다.
PyTorch AMP·Large의 실시간 공급률은 운영 Medium TensorRT의 성능 수락과 별개다.

## 직접 주행·시각화

MORAI와 기존 UDP bridge를 실행하고 `asmc-ros-noetic`에서 아래 명령을 사용한다.
실험 실행은 한 세트만 사용한다. 이전 실험 launch/rqt를 종료한 후 다시 시작한다.
일반 `camera.launch`를 대신할 필요는 없으며 함께 실행할 경우 GPU 부하는 추가된다.
품질 확인만 할 때는 BDD 실험 단독 실행을 권장한다. 차량 제어는 시작하지 않는다.

```bash
source /opt/ros/noetic/setup.bash
source /root/ws/devel/setup.bash
roslaunch asmc_camera bdd_lane.launch
# 공식 해상도 대조는 위 launch를 종료한 뒤 실행
roslaunch asmc_camera bdd_lane.launch resolution:=official
```

별도 터미널에서 환경을 source하고 렌더러를 실행한다.

```bash
roslaunch asmc_camera bdd_lane_view.launch
```

별도 터미널에서 rqt 한 개를 실행한다.

```bash
rosrun rqt_image_view rqt_image_view --clear-config /experiments/twinlite_bdd/debug/mosaic/compressed
```

좌/전방/우 순서이며 각 열의 원본·drivable·binary lane을 확인한다. 구독자가 없으면
렌더러의 입력 구독·디코딩·인코딩과 원본 크기 확률 복원을 중단한다.
렌더러는 동일 camera/stamp의 원본과 확률만 대응시킨다.

## 토픽·제어 격리

| 대상 | 실험 경로 |
|---|---|
| 원본 RGB 입력 | 기존 `/image_jpeg/compressed`, `/cam_left/image_jpeg/compressed`, `/cam_right/image_jpeg/compressed` |
| 확률 결과 | `/experiments/twinlite_bdd/{front,left,right}/{drivable,lane}/probability` |
| 합성 시각화 | `/experiments/twinlite_bdd/debug/mosaic/compressed` |
| 공용 상태·모드 | `/experiments/twinlite_bdd/status`, `/experiments/twinlite_bdd/set_mode` |
| 진단 | `/experiments/twinlite_bdd/diagnostics` |

운영 `/perception/camera/lane/status`, family, model-grid road_marking, BEV를 발행하지 않는다.
호환 상태·서비스도 실험 namespace로 remap한다. `prepared`는 모델 입력 준비 상태이고,
`pipeline_ready=false`는 family가 연결되지 않은 실험의 의도된 상태다. 판제 준비 완료로 사용하지 않는다.
`/experiments/twinlite_bdd/unused_family`에는 생산자가 없으며 family feedback 격리용이다.

## 재현

원본 다운로드 URL과 SHA는 배포 source JSON에 있다. 공식 소스를 고정 commit으로 설치한
기존 카메라 환경에서 포장 도구를 실행한다. 출력이 이미 있으면 덮어쓰지 않는다.

```bash
python3 tools/perception/prepare_twinlite_bdd.py \
  --source /data/models/twinlite_bdd_large_v1/large.pth \
  --output /tmp/twinlite-large-bdd100k.pt
python3 tools/perception/evaluate_twinlite_bdd.py \
  --bdd-checkpoint src/asmc_camera/models/lane_segmentation/twinlite-large-bdd100k.pt \
  --medium-checkpoint src/asmc_camera/models/lane_segmentation/twinlite-medium-v6.pt \
  --output /tmp/twinlite-bdd-comparison
```

오프라인 도구는 기본 v6 val 1,404장과 같은 원본/GT를 사용한다. drivable은 객체 점유를
negative로 취급하는 MORAI GT, lane은 흰/황 차선의 합집합으로 비교한다. GT 정지선 픽셀은
binary lane 비교에서 제외한다. 학습 도메인·라벨 정의가 다른 두 모델의 개발 비교이며,
Large와 Medium 구조 자체의 우열 또는 추가학습 후 성능을 입증하는 실험이 아니다.
