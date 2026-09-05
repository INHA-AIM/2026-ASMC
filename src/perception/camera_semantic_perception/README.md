# Camera semantic perception

> 상태: Stage C1 — 모델 독립 live inference 기반 구현

MORAI의 전방 또는 전·좌·우 JPEG 카메라를 latest-only 방식으로 받아 TwinLiteNet+ 2-head
추론 결과를 표준 ROS probability 토픽으로 발행한다. 모델 로딩은 backend에 격리되어 이후
다른 segmentation 모델로 교체해도 ROS 입력·출력 배관을 유지한다.

## 현재 범위

- 선택 view: `front` 또는 `front,left,right`
- 학습과 동일한 RGB `[0,1]`, `384×640` letterbox
- front 1600×900 기준 상·하 12 px padding과 가변 원본 크기 자동 계산
- TwinLiteNet+ Medium, EMA 우선 체크포인트 로딩
- 체크포인트 SHA-256, epoch, upstream commit, weight 종류 시작 로그
- 원본 카메라 크기로 복원한 `mono8` 확률 출력
- latest-only buffer와 최대 4 ms 3-view batch coalescing
- 2초 단위 FPS와 단계별 평균 지연 로그(정밀 percentile 계측은 Stage C3)

## 실행 컨테이너

UDP 브릿지는 `asmc-ros-noetic`, GPU 추론은 `asmc-perception-train`에서 실행한다. 두
컨테이너는 Docker Desktop host network의 같은 ROS master `127.0.0.1:11311`을 사용한다.
학습용 컨테이너에는 공식 TwinLiteNet+ checkout이 `/opt/baselines/TwinLiteNetPlus`로
읽기 전용 마운트된다.

체크포인트와 학습 run은 Git에 포함되지 않는다. 기본 launch는 현재 로컬 후보 경로를
가리키므로 다른 PC에서는 공유받은 `best_mean.pt` 경로를 `checkpoint:=...`로 지정해야 한다.
체크포인트 내부 upstream commit과 외부 checkout commit이 다르면 안전하게 중단한다.

```bash
# WSL
./scripts/docker_ros_up.sh up
./scripts/docker_perception_up.sh up

# ROS 컨테이너: 최초 또는 패키지 변경 후
docker exec -it asmc-ros-noetic bash -lc './scripts/build_ws.sh'

# 별도 터미널: UDP 브릿지
docker exec -it asmc-ros-noetic bash
source devel/setup.bash
./scripts/udp_bridge.sh

# 별도 터미널: front-only GPU 추론
docker exec -it asmc-perception-train bash
source /root/ws/devel/setup.bash
roslaunch camera_semantic_perception front.launch
```

3-view 실행:

```bash
roslaunch camera_semantic_perception three_view.launch \
  checkpoint:=/root/ws/artifacts/perception_eval/runs/<run>/best_mean.pt
```

CPU 배관 시험은 `device:=cpu`로 덮을 수 있지만 실시간 성능 기준으로 사용하지 않는다.

## 2026-09-05 1차 live 실측

환경은 26.R1.H3, RTX GPU, 입력 1280×720, TwinLiteNet+ Medium epoch 18 EMA이다.

| 모드 | 결과 |
|---|---|
| front-only | lane/drivable 모두 약 20 Hz, GPU infer 약 16~19 ms/batch |
| three-view | view별 약 18.5~19 Hz, 총 약 54~58 view-frame/s |
| three-view GPU infer | 약 23~25 ms/batch |
| 출력 | 입력 stamp/frame ID를 보존한 원본 크기 1280×720 `mono8` |

이는 파이프라인 연결과 처리 가능성을 보는 1차 평균 로그다. 워밍업 구간을 제외한 p50/p95,
frame age, VRAM과 다른 노드 동시 실행 시 지연은 Stage C3에서 다시 측정한다.

## 입력과 출력

| View | 입력 |
|---|---|
| front | `/image_jpeg/compressed` |
| left | `/cam_left/image_jpeg/compressed` |
| right | `/cam_right/image_jpeg/compressed` |

각 선택 view에 다음 `sensor_msgs/Image`, `mono8` 토픽을 발행한다. 값 0~255는 foreground
confidence 0~1에 대응하며 입력 메시지의 stamp와 frame ID를 보존한다.

```text
/perception/camera/{view}/drivable/probability
/perception/camera/{view}/lane/probability
```

## 검증

```bash
PYTHONPATH=src/perception/camera_semantic_perception/src \
python3 -m unittest discover \
  -s src/perception/camera_semantic_perception/test -v

rostopic hz /perception/camera/front/drivable/probability
rostopic hz /perception/camera/front/lane/probability
rostopic echo /perception/camera/front/lane/probability -n1
```

## 다음 단계

1. 독립 visualizer에서 원본·drivable·lane·overlay panel과 3-view mosaic 발행
2. decode/preprocess/infer/postprocess/publish의 p50/p95, frame age, drop/stale와 VRAM 계측
3. 조건별 live gate로 front-only/3-view와 현 센서 해상도·추론 Hz 결정
4. 확률 마스크를 camera calibration으로 공통 ego-BEV에 투영
