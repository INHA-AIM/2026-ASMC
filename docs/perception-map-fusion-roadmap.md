# 카메라·MGeo·LiDAR 인지 융합 개발 로드맵

> **문서 역할:** Current Working Proposal — 인지 융합 방향과 단계별 결정 gate
> **문서 허브:** [README.md](./README.md)
> **상태:** Working Proposal v0.1 — 팀 검토와 실증 결과에 따라 변경 가능<br>
> **기준일:** 2026-09-05<br>
> **주요 참여:** 안승현(카메라 인지), 장원태(MGeo map prior 제안), 손재호(LiDAR 인지), planning/control 팀<br>
> **상세 문서:** [인지 GT·BEV·차로 구조화 계획](./perception-bev-lane-interface-plan.md) · [UDP↔ROS 네트워크](./network-udp-ros.md) · [v4 학습 평가](./perception-v4-training-evaluation.md)

이 문서는 팀 전체의 개발 방향을 맞추기 위한 제안서다. 토픽 이름, BEV 범위, 메시지
필드와 담당 경계는 아직 최종 계약이 아니다. 카메라 live 실증과 map bake pilot 결과를
확인한 뒤 `src/interfaces/`에서 별도 합의하고 변경 이력을 남긴다.

## 1. 목표와 우선순위

대회 우선 목표는 허용 센서만 사용해 K-City 경로를 안정적으로 완주하는 것이다. 하나의
인지 결과를 절대 기준으로 삼기보다 다음 세 계층을 분리하고, 신뢰도와 시간 정보를 가진
공통 ego-BEV에서 결합하는 방향을 우선 검토한다.

| 계층 | 주 역할 | 현재 방향 |
|---|---|---|
| MGeo map prior | 정적 도로 구조, route corridor, 차로 위상, 정지선 | 제공 데이터를 offline bake한 뒤 허용 측위로 local crop |
| Camera perception | 현재 보이는 lane·drivable 관측 | TwinLiteNet+ 2-head를 먼저 live 검증, 모델 교체 가능한 adapter 사용 |
| LiDAR perception | 동·정적 장애물의 위치·점유·추적 | 장애물 검출을 도로/차로 문맥과 연결하고 최종 safe area에서 우선 반영 |

현재 작업 순서는 **카메라 실시간 추론·시각화·계측 → 센서 profile 결정 → 공통 BEV
pilot → 차로 구조화와 LiDAR 연계**다. 정지선은 MGeo/전역경로 정지점을 1차 후보로 두고,
영상 stop-line head는 위치추정 또는 지도 오차가 실제 문제로 확인될 때 추가한다.

## 2. 대회 통신 경계

본선에서 MORAI와 팀 PC 사이에는 규정 v1.1의 허용 UDP만 사용한다. ROS는 팀 PC 내부
모듈 연결에만 사용한다.

```text
MORAI
  └─ allowed UDP
       ├─ morai_sensor_bridge ─ Camera/LiDAR/GPS/IMU ROS topics
       └─ morai_udp_bridge    ─ Competition Vehicle Status/Collision ROS topics

internal ROS modules
  └─ /ctrl_cmd ─ morai_udp_bridge ─ Ego Ctrl Cmd UDP ─ MORAI
```

배포 경로에서는 다음을 사용하지 않는다.

- MORAI ROS/rosbridge WebSocket 직접 연결
- Capture Mode gRPC, GetMGeo gRPC와 시나리오 제어 gRPC
- Semantic/Instance GT 카메라
- Object Info, 신호등 상태 등 규정에 없는 oracle 입력
- GPS blackout의 의도를 우회하는 비허용 전역 위치 필드

개발·학습 도구와 본선 실행 profile은 명시적으로 분리한다. 본선 제어는 Ego Ctrl Cmd
`cmd type 1`, `ctrl mode 2`를 유지하며, Competition Vehicle Status도 주최 측이 허용한
필드 범위를 대회판에서 다시 확인한다.

## 3. 제안 시스템 구조

```text
                         OFFLINE
provided MGeo/global path ── map baker ── versioned static map layers
                                                │
                         RUNTIME                ▼
GPS/IMU/allowed status ─ localization ─ local map crop + pose covariance
                                                │
Camera front/left/right ─ segmentation ─ camera probabilities
                                                │
LiDAR ─ detection/tracking/occupancy ───────────┤
                                                ▼
                                      common ego-BEV layers
                                                │
                    ┌───────────────────────────┼─────────────────────┐
                    ▼                           ▼                     ▼
             lane/topology              safe drivable        object↔lane context
                    └───────────────────────────┼─────────────────────┘
                                                ▼
                                      behavior / planner / MPC
```

### 3.1 MGeo map prior 후보 출력

- drivable surface와 route corridor
- lane boundary/centerline, lane width와 연결 관계
- 경로 선택지가 없는 구간의 nominal ego lane
- 좌·우 인접 차로와 차로 변경 가능성
- stop-line geometry와 전역경로상의 정지 후보점
- 속도 제한/예외 link 같은 정적 route attribute

정적 지도에는 NPC와 스폰 장애물을 굽지 않는다. 지도 산출물에는 map 버전, bake 설정,
전역경로 hash와 좌표계가 포함되어야 한다.

### 3.2 Camera perception 후보 출력

- view별 `drivable_probability`, `lane_probability`
- 원본 영상에서의 coverage와 유효 영역
- 모델/checkpoint/전처리 provenance
- timestamp, frame age와 추론 지연
- 추후 ego-BEV로 투영한 확률 및 관측 confidence

현재 18 epoch EMA 후보의 v4 validation은 1,404 sample에서 전체 drivable IoU `0.9842`,
lane IoU `0.8930`이다. front lane IoU는 `0.8015`로 좌·우보다 낮으며 교차로, 횡단보도,
그림자와 가는 차선이 우선 실증 대상이다. 이 validation은 model selection에 사용했으므로
독립 test와 live 결과 없이 최종 배포 신뢰도로 간주하지 않는다.

### 3.3 LiDAR와 도로 문맥의 결합

카메라 또는 MGeo로 도로와 차로가 구조화되면 LiDAR 검출물마다 다음 정보를 붙일 수 있다.

- `lane_relation`: `ego`, `left`, `right`, `other`, `outside`, `unknown`
- drivable/route corridor와 footprint가 겹치는 비율
- lane centerline 기준 종·횡방향 위치 후보
- ego 경로와의 충돌 가능성, 거리와 TTC
- map/camera 중 어떤 근거를 사용했는지와 confidence/age

이를 이용하면 ego lane 장애물은 직접 회피 대상으로, 인접 차로 차량은 차로 변경 gap
평가 대상으로, 도로 밖 구조물은 낮은 우선순위로 분류하기 쉽다. 그러나 map pose 오차나
camera segmentation 누락 때문에 `outside` 판정만으로 LiDAR 검출물을 즉시 삭제하지 않는다.
차량 주변 안전 envelope에 들어오거나 LiDAR 신뢰도가 높은 물체는 별도 collision 후보로
보존한다. 최종 `safe_drivable`은 정적 drivable에서 LiDAR occupancy를 차감해 만든다.

## 4. GPS blackout에서의 후보 운용

GPS가 정상일 때는 pose 기반 map crop과 camera-BEV를 정합해 카메라 횡방향 오차, heading
오차와 temporal jitter를 평가할 수 있다. GPS blackout에 진입하면 GPS crop을 계속 사용할
수 없으므로 다음 후보 구성이 필요하다.

```text
last reliable global pose
  + IMU yaw rate
  + allowed longitudinal speed/status
  + issued control
  = propagated pose + growing covariance

propagated pose
  + camera lane/drivable alignment
  + optional LiDAR map alignment
  = corrected local pose candidate
```

- map crop은 한 점의 확정 pose보다 불확실성 범위를 함께 취급한다.
- 완만한 steering angle/rate와 적절한 속도 영역을 반복 주행으로 찾는다.
- 안정적인 제어 영역은 drift를 줄이는 수단이지 localization을 대체하지 않는다.
- nominal path/feed-forward 제어에 camera lateral/heading feedback을 결합하는 안을 우선 검토한다.
- blackout 종료 후 GPS가 복귀할 때 pose가 급변하지 않도록 재결합 정책이 필요하다.

## 5. 코드 경계 제안

다음 경로는 구현 전 제안이며 담당자 pilot 후 조정할 수 있다.

```text
src/perception/
├── twinlite_morai/                 # 현재: 학습·평가와 TwinLite 전용 adapter
├── camera_semantic_perception/     # 제안: 모델 독립 live 추론·시각화·성능계측
├── map_prior/                      # 후보: offline bake 결과 로드와 local crop
├── drivable_bev/                   # 후보: 공통 grid, camera/map layer와 fusion
└── morai_3d_detection/             # 현재: 손재호 3D 인지 연구 코드

src/interfaces/                     # 실증 뒤 합의한 ROS message/topic 계약
src/data_collection/                # Capture GT와 추후 최소 rosbag profile
```

### 5.1 카메라 live 패키지 내부 제안

```text
camera_semantic_perception/
├── config/                         # camera, backend, visualization profile
├── launch/                         # front-only / three-view / debug
├── scripts/
│   ├── semantic_inference_node.py
│   └── semantic_visualizer_node.py
├── src/camera_semantic_perception/
│   ├── backends/                   # TwinLite+, 추후 모델 adapter
│   ├── inference_engine.py
│   ├── frame_buffer.py
│   ├── performance_monitor.py
│   └── visualization.py
└── test/
```

추론 노드는 영상 수신, 모델 실행, 표준 확률 출력과 성능계측만 담당한다. 시각화 노드는
모델/CUDA에 의존하지 않고 RGB와 표준 확률을 구독해 원본, head mask, overlay와 3-view
mosaic를 만든다. BEV는 이 확률을 소비하는 별도 단계로 둔다.

## 6. Working topic/interface proposal

아래 이름은 live 개발을 시작하기 위한 작업안이다. custom message와 최종 namespace는
카메라 성능·출력 범위를 확인한 뒤 파트 합의로 확정한다.

### 6.1 기존 입력

| View | Topic | Type |
|---|---|---|
| front | `/image_jpeg/compressed` | `sensor_msgs/CompressedImage` |
| left | `/cam_left/image_jpeg/compressed` | `sensor_msgs/CompressedImage` |
| right | `/cam_right/image_jpeg/compressed` | `sensor_msgs/CompressedImage` |

### 6.2 카메라 출력 후보

```text
/perception/camera/{view}/drivable/probability
/perception/camera/{view}/lane/probability
/perception/camera/{view}/debug/panel/compressed
/perception/camera/debug/mosaic/compressed
/perception/diagnostics
```

- probability는 초기에는 `sensor_msgs/Image` `mono8`의 0~255 confidence를 후보로 한다.
- hard mask와 overlay는 시각화 노드가 configurable threshold로 파생한다.
- float probability가 필요한지, 전송량이 적절한지는 BEV pilot에서 다시 결정한다.
- 새 모델은 같은 head 의미를 반환하는 backend adapter를 추가해 교체한다.
- bbox/object detection은 segmentation head와 의미가 달라 별도 계약으로 확장한다.

### 6.3 공통 BEV 출력 후보

```text
/perception/bev/map/drivable
/perception/bev/map/lane
/perception/bev/map/route_corridor
/perception/bev/map/stopline
/perception/bev/camera/drivable_probability
/perception/bev/camera/lane_probability
/perception/bev/lidar/occupancy
/perception/bev/fused/safe_drivable
/perception/bev/fused/lanes
```

초기 raster는 `nav_msgs/OccupancyGrid` 또는 `sensor_msgs/Image`, vector lane은
`nav_msgs/Path`/marker로 pilot할 수 있다. 최종 planning 소비 필드가 확인되면 versioned
custom message를 검토한다.

### 6.4 아직 확정하지 않을 항목

- BEV 전·후·좌·우 범위와 meter/pixel
- ego origin을 vehicle center/rear axle 중 어디에 둘지
- camera probability bit depth와 threshold
- lane ID와 left/right adjacency custom message
- fusion weight와 map-camera disagreement 처리
- confidence, covariance와 timeout의 최종 단위/범위

좌표계는 우선 REP-103에 맞춘 `base_link`의 x-forward, y-left, z-up을 후보로 하며 모든
메시지에 timestamp, frame ID, source, age/validity가 있어야 한다.

## 7. 카메라 live 개발과 평가 계획

### Stage C1 — 모델 독립 추론 기반

- TwinLiteNet+ backend와 18 epoch EMA checkpoint 검증
- `front` 또는 `front,left,right` 선택
- latest-only buffer와 3-view batch
- 학습과 동일한 `384x640` letterbox 및 원본 좌표 복원
- checkpoint SHA, upstream commit, head와 전처리 설정 기록

**2026-09-05 1차 구현 완료:** `camera_semantic_perception` 패키지에서 front-only 약
20 Hz, three-view 각 약 18.5~19 Hz를 확인했다. 현재 출력은 원본 1280×720 `mono8`
probability이며, 정밀 percentile·frame age·VRAM과 full-stack 동시 부하는 Stage C3에서
다시 측정한다.

### Stage C2 — 독립 시각화

- view별 원본, drivable/lane mask와 overlay
- front/left/right mosaic
- `rqt_image_view`에서 확인 가능한 compressed debug topic
- 날씨·시간·시나리오 condition marker 표시

### Stage C3 — 성능계측

- input/output FPS, drop/stale count와 camera timestamp skew
- JPEG decode, preprocess, GPU infer, postprocess, publish 시간
- processing latency와 frame age의 p50/p95
- GPU allocated/reserved/peak memory
- 1~2초 집계 로그, `/diagnostics`, 선택적 JSONL session summary

프레임마다 상세 로그를 출력하면 I/O가 측정값을 교란하므로 기본은 집계 로그로 하고
per-frame 로그와 CUDA 강제 동기화는 별도 profile mode에서만 사용한다.

### Stage C4 — 조건별 live gate

- 규정 조건 Sunny/Foggy × 11am/1pm/3pm
- 5pm은 규정 외 강건성 조건으로 별도 표기
- 직선, 곡선, 교차로, 횡단보도, 장애물과 GPS 음영구간
- front-only와 three-view, perception 단독과 LiDAR 동시 실행 비교
- 새 독립 Capture/test run으로 정량 평가

이 결과로 센서 native 해상도, 추론 Hz, AMP와 batch 방식을 고정한다. 모델 변경은 live 또는
독립 test에서 front lane 품질이나 full-stack latency가 부족한 경우에만 같은 조건으로
비교한다.

## 8. 공용 rosbag의 범위

팀 공용 재현 데이터는 유용하지만 카메라 live 완성을 지연시키는 범용 recorder 개발은
우선하지 않는다. ROS1의 `rosbag record`를 사용하고, 카메라 기능이 안정화된 뒤 기존
`data_collection`에 작은 profile/wrapper를 추가하는 안을 검토한다.

후보 profile:

- `camera_perception`: camera, probability, diagnostics, pose/status
- `localization_map`: GPS, IMU, 허용 status, pose/covariance, map crop
- `lidar_perception`: point cloud, detections/tracks, occupancy
- `full_stack`: 통합 재현용 raw+derived 핵심 토픽
- `competition_legal`: 허용 UDP에서 파생된 입력과 내부 결과만 포함

추론 노드가 직접 bag을 쓰지 않는다. 별도 프로세스로 기록해 디스크 I/O와 추론 지연을
분리한다. bag과 함께 git commit, checkpoint/sensor/map/path hash, 시나리오, 날씨/시간,
topic/rate와 특이사항을 sidecar metadata로 보존한다. GT/oracle이 포함된 개발 bag은 이름과
경로를 분리해 본선 profile에서 실수로 재생하거나 구독하지 않게 한다.

## 9. 26.R1.h3 개발판과 대회판 이식 전략

현재 26.R1.h3는 대회판보다 기능이 풍부하며 K-City 맵과 내용은 유사하지만 message 이름,
지원 기능, timestamp/주기, 렌더링과 풀밭·노면 색감 등이 다를 수 있다. 개발판에서 기능을
완성하는 것과 대회판 호환을 검증하는 일을 분리한다.

### 9월 — 기능 완성 및 기준선 동결

1. 26.R1.h3에서 camera live, map prior pilot과 LiDAR context 연결을 개발한다.
2. 내부 model/backend와 MORAI message adapter를 분리한다.
3. topic rate, image encoding/resolution, timestamp와 허용 message 필드를 기록한다.
4. 모델/checkpoint, sensor profile, map bake와 공통 좌표 후보를 version/tag로 동결한다.
5. 대회판에서 사용하지 않을 gRPC/GT/oracle 경로는 launch profile로 격리한다.

### 10월 — 대회판 호환·안정화 브랜치 후보

9월 말 검증된 `main` commit에 tag를 붙인 뒤 `release/competition-sim` 같은 한시적
안정화 브랜치를 만드는 안을 팀 Git 정책에 맞춰 검토한다. 이 브랜치에서는 새 기능보다
호환 adapter, 파라미터와 성능 튜닝만 수행한다.

1. UDP packet/port, morai_msgs schema와 topic 이름 diff 작성
2. bridge adapter와 launch/config profile로 차이를 흡수
3. 카메라 색감, 압축, 해상도/FOV와 timestamp 실측
4. LiDAR/GPS/IMU rate와 blackout 동작 확인
5. full-stack p95 latency, drop과 판정/녹화 프로그램 동시 부하 확인
6. 대회판에서 짧은 카메라 GT 데이터를 추가 수집할 수 있으면 domain-gap 평가
7. 보강 학습 시 기존 26.R1.h3 데이터를 섞고 대회판 run 단위 val/test를 별도 유지
8. 최종 competition-legal launch와 금지 토픽 subscriber 정적/실행 검사

호환 수정은 가능하면 `main`의 공통 알고리즘에도 반영하고 대회판 전용 차이는 config와
adapter에 국한한다. 장기적으로 두 구현이 갈라지는 것을 막기 위해 안정화 브랜치의 bugfix
반영 방향과 종료 시점을 함께 정한다.

## 10. 제안 로드맵과 결정 gate

| 순서 | 산출물 | 다음 단계 진입 조건 | 상태 |
|---:|---|---|---|
| 1 | 카메라 model-independent live inference | front/3-view 정상 출력, checkpoint provenance 확인 | 1차 구현·live 검증 완료 |
| 2 | 독립 visualizer와 runtime diagnostics | 조건별 화면, p50/p95/drop/VRAM 기록 | 다음 구현 |
| 3 | 독립 camera test와 full-stack profile | 품질·지연으로 현 모델/센서 profile 판단 가능 | 예정 |
| 4 | MGeo bake/local crop pilot | map/route hash, pose/covariance와 local layer 확인 | 장원태 제안 |
| 5 | 공통 BEV grid pilot | map과 camera가 같은 `base_link` grid에 정렬 | 공동 |
| 6 | lane topology + LiDAR association | ego/adjacent/unknown과 safe area 출력 | 공동 |
| 7 | GPS blackout localization/control 실험 | drift, lateral error와 복귀 안정성 기준 충족 | 공동 |
| 8 | 최소 rosbag profile | 필요한 raw/derived 재현과 metadata 확인 | 후순위 |
| 9 | 대회판 이식·보강 | competition-legal audit와 full rehearsal 통과 | 10월 후보 |
| 10 | 영상 stop-line head | map 기반 방식의 실제 실패가 확인될 때 재평가 | 후순위 |

## 11. 팀 합의가 필요한 열린 질문

다음 항목은 문서만으로 확정하지 않고 pilot 결과를 보고 회의/PR에서 결정한다.

1. map prior와 camera-BEV의 공통 범위·해상도·ego origin
2. GPS blackout에서 허용 status 필드와 localization source 우선순위
3. map-camera 정합 결과를 localization 보정에 직접 넣을지, 평가용으로 먼저 제한할지
4. LiDAR object association의 `outside/unknown` 보존 및 collision override 규칙
5. planning이 필요한 raster/vector 필드와 update rate
6. competition branch 이름, 유지 기간과 main backport 규칙
7. 대회판 추가 데이터의 확보 가능 범위와 train/val/test 분할
8. 공용 rosbag profile 담당과 저장 용량/보존 정책

변경 제안은 이 문서의 전제를 조용히 덮어쓰지 말고, 근거가 된 실증 run, metric 또는
인터페이스 소비 요구와 함께 반영한다.
