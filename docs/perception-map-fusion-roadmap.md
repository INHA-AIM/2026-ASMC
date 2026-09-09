# 카메라·MGeo·LiDAR 인지 융합 개발 로드맵

> **문서 역할:** Current Working Proposal — 인지 융합 방향과 단계별 결정 gate
> **문서 허브:** [README.md](./README.md)
> **상태:** Working Proposal v0.3 — 팀 검토와 실증 결과에 따라 변경 가능<br>
> **기준일:** 2026-09-09<br>
> **주요 참여:** 안승현(카메라 인지), 장원태(MGeo map prior 제안), 손재호(LiDAR 인지), planning/control 팀<br>
> **상세 문서:** [인지 GT·BEV·차로 구조화 계획](./perception-bev-lane-interface-plan.md) · [UDP↔ROS 네트워크](./network-udp-ros.md) · [v4 학습 평가](./perception-v4-training-evaluation.md)

이 문서는 팀 전체의 개발 방향을 맞추기 위한 제안서다. 차량·센서 기준 좌표는 규정과
센서셋에 맞춰 정본화하되, 토픽 이름, BEV raster 범위·해상도, 메시지 필드와 담당 경계는
아직 최종 계약이 아니다. 카메라 BEV 투영과 map bake pilot 결과를 확인한 뒤
`src/interfaces/`에서 별도 합의하고 변경 이력을 남긴다.

## 1. 목표와 우선순위

대회 우선 목표는 허용 센서만 사용해 K-City 경로를 안정적으로 완주하는 것이다. 하나의
인지 결과를 절대 기준으로 삼기보다 다음 세 계층을 분리하고, 신뢰도와 시간 정보를 가진
공통 ego-BEV에서 결합하는 방향을 우선 검토한다.

| 계층 | 주 역할 | 현재 방향 |
|---|---|---|
| MGeo map prior | 정적 도로 구조, route corridor, 차로 위상, 정지선 | 제공 데이터를 offline bake한 뒤 허용 측위로 local crop |
| Camera perception | 현재 보이는 drivable·white/yellow/stopline 관측 | TwinLiteNet+ 4-class road-marking 모델을 live 검증, 모델 교체 가능한 adapter 사용 |
| LiDAR perception | 동·정적 장애물의 위치·점유·추적 | 장애물 검출을 도로/차로 문맥과 연결하고 최종 safe area에서 우선 반영 |

현재 작업 순서는 **설정·문서 정본화 → 재현 가능한 raw bag 평가 기반 → Streaming GT
transport gate → 좌표·BEV 기준 확정 → 카메라 결과 구조화 → MGeo 결합 → LiDAR/판제 연계**다.
정지선은 MGeo/전역경로 정지점을 1차 후보로 두고 현재 영상 stopline class를 보조 검증과
GPS Blackout 대응 후보로 평가한다.

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

- view별 `drivable_probability`, `road_marking_probability`
- road marking class `background=0`, `white_lane=1`, `yellow_lane=2`, `stopline=3`
- 원본 영상에서의 coverage와 유효 영역
- 모델/checkpoint/전처리 provenance
- timestamp, frame age와 추론 지연
- 추후 ego-BEV로 투영한 확률 및 관측 confidence

18 epoch EMA v4 2-head validation은 1,404 sample에서 전체 drivable IoU `0.9842`, lane
IoU `0.8930`이었던 이력 기준선이다. 현재는 drivable + 4-class road-marking v6
체크포인트를 live 시각화까지 확인했으며, 같은 raw bag과 독립 test 없이 두 결과를 직접
우열 비교하지 않는다. 교차로, 횡단보도, 그림자와 가는 차선은 계속 우선 실증 대상이다.

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
보존한다. 정적 drivable에서 LiDAR occupancy를 차감한 `safe_drivable`은 유력한 후보지만,
그 생성 책임을 인지·퓨전·판제 중 어디에 둘지는 소비 인터페이스 합의 후 확정한다.

### 3.4 차량·센서 좌표계 계약

MORAI 차량 센서 좌표 원점은 후륜축 중앙으로 보고 공통 `base_link`로 사용한다. 실제
Ego/GPS 데이터를 이용해 원점과 yaw 부호를 확인하기 전에는 planner 동작값을 변경하지
않는다.

| 항목 | 기준값 |
|---|---:|
| 차량 길이 / 폭 / 높이 | 4.635 / 1.892 / 2.434 m |
| Wheelbase | 3.000 m |
| Front / Rear overhang | 0.845 / 0.790 m |
| GPS 안테나 | `(+0.350, 0, +1.250)m` |
| LiDAR | `(+0.800, 0, +1.450)m` |
| 후륜축→전륜축 / 전방 범퍼 | 3.000 / 3.845 m |
| 후방 범위 / 반폭 | 0.790 / 0.946 m |
| 후륜축→차량 형상 중심 | 1.5275 m |

GPS 안테나 좌표에서 후륜축 중앙을 복원할 때는 다음 lever-arm 보정을 적용한다.

```text
p_base = p_gps - Rz(yaw) · [0.350, 0, 1.250]
```

현재 Frenet collision footprint의 `4.5×1.9m`, Behavior의 `4.7×1.85m` 근사와 GPS
lever-arm 미보정 가능성은 구현 전 감사 항목이다. 신호 판정용 정지선 거리는 차량 형상
중심이 아니라 규정 판정점인 전륜축 `x=3.000m`도 함께 산출해야 한다. 기준은 MORAI의
[차량 후륜축 중앙 센서 좌표](https://help-morai-sim.scrollhelp.site/en/morai-sim-drive/23.R1.0/-7)와
2026-09-08 규정집 v1.1을 따른다.

### 3.5 Raster·vector·Frenet 역할

Raster는 지면을 일정한 m/cell 격자로 나누고 각 cell에 확률·class·occupancy를 저장한
표현이다. 센서 융합에는 편리하지만 해상도에 따른 메모리 비용과 얇은 차선의 aliasing이
있다. MGeo의 연결 관계와 정지선은 vector/graph로 보존하고, 공통 local raster는
`base_link`의 x-forward, y-left Cartesian 격자로 둔다.

Frenet은 선택한 전역경로를 따라간 누적거리 `s`와 횡방향 오프셋 `d`를 사용하는 판제
좌표계다. 센서 projection과 충돌 계산용 공통 raster를 Frenet으로 만들지 않고, BEV에서
추출한 vector geometry와 객체를 판제 경계에서 Frenet으로 변환한다. BEV 범위와 m/cell은
카메라 확률 투영과 HD맵 crop을 같은 화면에서 확인한 뒤 확정한다. 개념 근거는
[Frenet 기반 궤적 생성 연구](https://publikationen.bibliothek.kit.edu/1000022393)에 연결한다.

## 4. GPS blackout에서의 후보 운용

2026-09-08 규정집 v1.1은 GPS·IMU에 일반 Noise를 인가하지 않지만, GPS 음영 구역의 GPS
전체 Blackout은 유지한다. 정상 구간에서는 MGeo topology·정지선을 우선 사용하고
camera-BEV를 실제 노면 관측과 map-camera 정합 평가에 사용한다. GPS blackout에 진입하면
GPS crop을 그대로 신뢰할 수 없으므로 다음 후보 구성이 필요하다.

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
- map 우선, camera 우선, 상호 검증 방식은 정상/진입/진행/복귀 구간별 정량 결과로 비교한다.

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
/perception/camera/{view}/road_marking/probability
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
/perception/bev/camera/road_marking_probability
/perception/bev/lidar/occupancy
/perception/bev/fused/safe_drivable
/perception/bev/fused/lanes
```

초기 raster는 `nav_msgs/OccupancyGrid` 또는 `sensor_msgs/Image`, vector lane은
`nav_msgs/Path`/marker로 pilot할 수 있다. 최종 planning 소비 필드가 확인되면 versioned
custom message를 검토한다.

### 6.4 아직 확정하지 않을 항목

- BEV 전·후·좌·우 범위와 meter/pixel
- camera probability bit depth와 threshold
- lane ID와 left/right adjacency custom message
- fusion weight와 map-camera disagreement 처리
- confidence, covariance와 timeout의 최종 단위/범위

`base_link` 원점은 후륜축 중앙, 축은 REP-103에 맞춘 x-forward, y-left, z-up으로 한다.
모든 메시지에 timestamp, frame ID, source, age/validity가 있어야 한다. 이 좌표 계약은
1단계 문서 기준이며 공개 ROS topic이나 custom message 변경은 BEV pilot 이후 별도 합의한다.

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

- view별 원본, drivable 및 white/yellow/stopline mask와 overlay
- front/left/right mosaic
- `rqt_image_view`에서 확인 가능한 compressed debug topic
- 날씨·시간·시나리오 condition marker 표시

**2026-09-08 갱신:** RGB와 drivable/road-marking probability를 입력 stamp로 exact sync해
view별 2×2 panel을 만들고, 전·좌·우 최신 panel을 순서대로 묶은 mosaic를 발행한다. 실제
MORAI 1280×720 입력에서 front panel과 1920×360 3-view mosaic를 확인했다. debug JPEG는
추론과 분리해 최대 10 Hz로 제한했다. 현재 visualizer는 좌·전방·우 3열과
RGB/drivable/road-marking 3행으로 표시하며 road-marking class를 서로 다른 색으로
구분한다. condition label과 threshold는 runtime ROS parameter로 조정할 수 있다.

### Stage C3 — 성능계측

- input/output FPS, drop/stale count와 camera timestamp skew
- JPEG decode, preprocess, GPU infer, postprocess, publish 시간
- processing latency와 frame age의 p50/p95
- GPU allocated/reserved/peak memory
- 1~2초 집계 로그, `/diagnostics`, 선택적 JSONL session summary

프레임마다 상세 로그를 출력하면 I/O가 측정값을 교란하므로 기본은 집계 로그로 하고
per-frame 로그와 CUDA 강제 동기화는 별도 profile mode에서만 사용한다.

**2026-09-09 기반 구현 완료:** `/perception/camera/diagnostics`에 backend/checkpoint, view별
input/output/replaced frame, 오류 수, stage별 평균, processing FPS, queue/e2e p50·p95와
source/receive/result stamp를 발행한다. upstream bridge fallback도 view별로 전달한다. 동일
raw bag replay와 model별 result bag으로 조건별 수치를 채우는 실증은 Stage C4에서 수행한다.

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

**2026-09-09 구현 완료:** 공용 recorder는 새 직렬화 포맷을 만들지 않고 ROS1의 검증된
`rosbag record`를 allowlist profile과 metadata lifecycle로 감싼다. recorder는 추론 노드와
별도 프로세스로 실행하며 기록 중 resample이나 timestamp 정렬을 하지 않는다. message
header stamp와 bag 기록 시각을 보존하고 정렬은 offline auditor/evaluator가 담당한다.
실행법과 profile 계약은
[rosbag 평가·Streaming GT 가이드](./rosbag-evaluation-streaming-gt.md)를 정본으로 삼는다.

Raw 입력과 모델 결과는 다음처럼 분리한다.

```text
data/rosbags/raw/<run_id>/
  metadata.yaml  environment_timeline.jsonl  bags/*.bag

data/rosbags/results/<raw_run_id>/<model_id>/
  metadata.yaml  bags/*.bag

data/rosbags/audits/<run_or_comparison>/
  audit.json
```

- raw bag은 생성 후 불변으로 취급한다.
- 모델/checkpoint/threshold가 달라지면 result run을 새로 만든다.
- `full_raw`도 `rosbag record -a`가 아니라 원천 센서·상태 topic의 명시적 allowlist를 쓴다.
- 대용량 debug image와 중간 결과의 중복 기록은 기본 profile에서 제외한다.
- bag과 함께 git commit/dirty, sensor/transport/timestamp provenance, 시나리오, 날씨/시간,
  topic/type/rate, bridge diagnostics와 bag SHA-256을 남긴다.
- GT/oracle bag은 경로와 profile을 분리하고 competition-legal launch가 구독하지 않게 한다.

목적별 후보 profile은 다음과 같다.

| Profile | 기록 목적 |
|---|---|
| `camera_raw` | RGB×3, Ego와 선택적 GPS/IMU |
| `lidar_raw` | Intensity point cloud, GPS/IMU/Ego |
| `full_raw` | 통합 재현에 필요한 원천 입력 allowlist |
| `lidar_dev_gt_udp` | ID 5/11, Object와 차량 상태의 privileged source-time stream |
| `dev_oracle_udp/ros` | transport A/B를 위한 전체 privileged oracle |
| `camera_semantic_result` | replay 추론 결과와 구조화 diagnostics |

Real Time UDP bag은 실제 비동기 지연·drop·jitter 분석 기준이다. 같은 bag을 재생하면 모델
A/B 입력은 통제할 수 있지만 MORAI→UDP bridge 이전의 packet 손실까지 재현하지는 못하므로
최종 선택 후 live Real Time 검증을 다시 수행한다.

### 8.1 학습용 Capture와 Streaming GT

카메라 단일-frame 학습은 Real Time + Capture 1 Hz를 기준으로 둔다. 2026-09-09에
2/5/10 Hz 요청을 시험했지만 실제 저장은 약 1.4~1.7 Hz에 포화됐고 요청마다 시뮬레이션이
약 0.5초 정지했다. 따라서 Capture는 학습 GT, Real Time Connect+rosbag은 temporal 평가로
분리한다. 수동 휠 주행의 지연·drop 측정에도 Capture를 함께 켜지 않는다.

LiDAR GT 센서셋은 동일 위치의 두 센서를 사용한다.

| 용도 | ID | 타입 | 위치 | 주기 | UDP Dest |
|---|---:|---|---|---:|---:|
| 실제 입력 | 5 | Intensity (`intensityType=0`) | `(0.8, 0, 1.45)m` | 10 Hz | 9299 |
| 학습 정답 | 11 | Instance (`intensityType=2`) | `(0.8, 0, 1.45)m` | 10 Hz | 9311 |

학습 모델 입력은 ID 5의 실제 intensity만 사용하고, ID 11의 instance point와
`_instance.txt`/`_instance_8Points.txt`는 class·3D box·상대속도·unique ID 정답으로만
사용한다. GT intensity를 feature로 사용하지 않는다. Instance는 차량·보행자·장애물을
개체별로 분리하므로 현재 객체 검출·OBB·tracking 목적에 맞다. Semantic LiDAR는 향후
road/lane point segmentation 연구가 필요할 때 별도 sensor profile로 만든다.

기존 clustering/OBB/tracking의 파라미터 평가에는 raw bag이 우선이고, 학습형 3D detector를
도입할 때 Capture의 Intensity–Instance/3D box pair를 사용한다. UDP source-time 실측에서
ID 5/11은 최신 20초에서 8.03/7.43 Hz로 수신됐지만 nearest gap p95가 약 102 ms여서
Streaming GT를 자동 학습 pair로 사용하지 않는다. RGB–Semantic도 구간에 따라 p95가
3 ms에서 31~35 ms까지 달라졌다. 허용 gap을 넘는 frame은 offline에서 제외하며 train/val/test는
인접 frame이 아니라 scenario/run/object seed 단위로 분리한다.

현재 UDP GT bridge는 source timestamp와 독립 ID 11 stream을 구현했다. ROS GT 구현은
현재 범위에 포함하지 않으며, UDP만으로 해결할 수 없는 요구가 확인될 때 별도 실험으로
재검토한다. 대회 경로는 항상 UDP로 유지한다.

## 9. 26.R1.h3 개발판과 대회판 이식 전략

현재 26.R1.h3는 대회판 `25.S4.MolitComp03`보다 기능이 풍부하며 K-City 맵과 내용은
유사하지만 message 이름, 지원 기능, timestamp/주기, 렌더링과 풀밭·노면 색감이 다를 수
있다. 개발판에서 기능을 완성하는 것과 대회판 호환·camera domain shift를 검증하는 일을
분리한다.

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
6. 대회판 학습에 쓰지 않을 독립 test run을 먼저 수집
7. Sunny/Foggy × 11am/1pm/3pm, 중요 구간과 일부 완주 lap을 추가 수집
8. GT Capture가 가능하면 paired GT, 불가능하면 검수한 수동 라벨 또는 제한적 pseudo-label 생성
9. simulator provenance를 분리한 26.R1.h3+대회판 결합 데이터셋으로 현재 4-class checkpoint 장기 재학습
10. 필요하면 대회판 sample을 의도적으로 가중하되 대회판 test는 절대 학습에 사용하지 않음
11. 기존 개발판 성능 무회귀와 대회판 held-out 개선을 함께 확인
12. image normalization/compression, threshold, BEV homography와 live FPS/p95 latency 재검증
13. 최종 competition-legal launch와 금지 토픽 subscriber 정적/실행 검사

호환 수정은 가능하면 `main`의 공통 알고리즘에도 반영하고 대회판 전용 차이는 config와
adapter에 국한한다. 장기적으로 두 구현이 갈라지는 것을 막기 위해 안정화 브랜치의 bugfix
반영 방향과 종료 시점을 함께 정한다.

## 10. 제안 로드맵과 결정 gate

| 순서 | 산출물 | 다음 단계 진입 조건 | 상태 |
|---:|---|---|---|
| 1 | 센서셋·규정·수집 모드·차량 좌표계 정본 | JSON/문서 검증과 변경 범위 확인 | 완료 |
| 2 | transport-aware raw/result bag과 source-time GT bridge | UDP smoke, lifecycle와 replay/audit 검증 | **구현 완료** |
| 3 | actor GT와 UDP↔ROS 5분 gate | Object ID 대응 및 transport 선정 근거 | UI 실증 남음 |
| 4 | 센서 calibration과 공통 BEV 기준 | GPS lever arm 검증, map-camera overlay로 범위·해상도 확정 | 공동 |
| 5 | 3-camera BEV와 lane 구조화 | drivable/white/yellow/stopline vector·거리 출력 | 안승현 |
| 6 | MGeo crop과 source policy 평가 | 정상 GPS·Blackout의 map/camera 정합 지표 확보 | 장원태·안승현 |
| 7 | LiDAR association과 판제 인터페이스 | ego/adjacent/unknown, occupancy와 ownership 합의 | 공동 |
| 8 | 대회판 이식·추가 데이터·장기 재학습 | competition-only held-out 개선과 full rehearsal 통과 | 10월 후보 |

## 11. 팀 합의가 필요한 열린 질문

다음 항목은 문서만으로 확정하지 않고 pilot 결과를 보고 회의/PR에서 결정한다.

1. 실제 map-camera overlay에 따른 공통 BEV 전·후·좌·우 범위와 m/cell
2. GPS blackout에서 허용 status 필드와 localization source 우선순위
3. map-camera 정합 결과를 localization 보정에 직접 넣을지, 평가용으로 먼저 제한할지
4. LiDAR object association의 `outside/unknown` 보존 및 collision override 규칙
5. LiDAR occupancy 차감과 `safe_drivable` 생성을 인지·퓨전·판제 중 어디서 소유할지
6. planning이 필요한 raster/vector 필드와 update rate
7. competition branch 이름, 유지 기간과 main backport 규칙
8. 대회판 GT Capture 가능 범위와 train/val/test 분할
9. 공용 rosbag profile 담당과 저장 용량/보존 정책

변경 제안은 이 문서의 전제를 조용히 덮어쓰지 말고, 근거가 된 실증 run, metric 또는
인터페이스 소비 요구와 함께 반영한다.
