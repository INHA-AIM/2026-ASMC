# Perception 개발·통합 가이드

> **문서 역할:** Active Package-Group Guide — 현재 인지 코드 경계, 문서 권위와 Phase 4 이관 gate
>
> **현재 상태:** P4-C6 package 통합과 P5-A/P5-B topic·rqt 정리, P5-C runtime acceptance 완료

이 문서는 긴 인지 설계·실험 문서를 대체하거나 요약 폐기하기 위한 문서가 아니다. 현재 어떤
코드와 문서를 먼저 사용해야 하는지 안내하는 진입점이다. 날짜별 분석, 수치, 실패 원인과 후보
변화는 원래 문서에 그대로 보존한다.

## 1. 문서 권위와 읽는 순서

| 확인하려는 내용 | 우선 문서 | 역할 |
|---|---|---|
| 현재 코드 위치와 통합 전 경계 | 이 문서 | Active Package-Group Guide |
| 본선 허용 입력과 금지 oracle | [대회 환경](../../competition/README.md) | Canonical Summary |
| 데이터·label·split 재현 | [dataset contract](../../contracts/perception/dataset-labeling-split.md) | Canonical Data Contract |
| raw/result bag 기록·재생 | [rosbag guide](../../operations/rosbag-evaluation.md) | Active Operations Guide |
| 음영 camera 출력을 판제에서 소비 | [shadow camera handoff](../../handoffs/perception/shadow-camera.md) | Active Handoff Guide |
| camera–LiDAR 객체 분류 | [object detection handoff](../../handoffs/perception/camera-object-detection.md) | Active Handoff Guide |
| GT·BEV·lane 정책의 상세 근거 | [BEV/lane design note](./bev-lane-interface-plan.md) | Dated Detailed Design Note |
| camera/map/LiDAR 개발 과정과 열린 결정 | [fusion roadmap](./roadmap.md) | Working Proposal + Chronological Record |
| LiDAR 패키지의 현재 완성도와 개선 계획 | [LiDAR assessment](./lidar-assessment-and-roadmap.md) | Assessment Record + Working Proposal |
| 특정 checkpoint의 학습 결과 | [v4 evaluation](../../records/perception/twinlitenet-v4-evaluation.md) | Experiment Record |
| 현재 topic 소비·bag 사용과 P5 정리 gate | [P5-A topic audit](../../records/baselines/perception-p5a-topic-audit-v1.md) | Frozen Input Audit |
| 실행 중 node/topic/rate/stamp/frame/consumer 검증 | [P5-C runtime acceptance](../../records/baselines/perception-p5c-runtime-acceptance-v1.md) | Active Acceptance Gate |

충돌할 때는 공식 규정, 동결된 contract, 현재 package source와 launch, handoff, working
proposal, 날짜별 record 순으로 판단한다. roadmap의 과거 `candidate`, `미구현`,
`operational=false` 문장은 해당 시점의 기록이며 현재 완성된 HD Map 상태를 되돌리지 않는다.

## 2. 현재 구현 지도

| 역할 | 현재 경로 | 현재 경계 |
|---|---|---|
| camera semantic runtime·backend·config | `src/asmc_perception/` | canonical 1/3-camera 추론·시각화, `asmc_perception.camera` |
| camera 상세 기록 | `src/asmc_perception/docs/camera-semantic-history.md` | 이전 live 측정·실험 이력; 현재 실행 정본 아님 |
| camera BEV/lane runtime·수학·config | `src/asmc_perception/` | `base_link` raster, fusion, lane-vector, `asmc_perception.bev` |
| BEV/lane offline audit | `src/asmc_perception/src/asmc_perception/offline/bev/`, `tools/bev/` | 음영 lane-family replay·pseudo-GT·audit; runtime에서 미기동 |
| TwinLiteNet+ adapter | `src/asmc_perception/src/asmc_perception/training/twinlite/` | 학습·평가·checkpoint loader; runtime에서 미기동 |
| 폐기한 camera 3D 연구 | Git history의 `src/perception/morai_3d_detection/` | 실행·참조가 없어 P4-C6에서 제거 |
| LiDAR와 camera fusion | `src/lidar+sensorfusion/` | outer catkin에서 제외된 독립 nested workspace |
| traffic-light runtime·model | `src/asmc_perception/` | canonical producer와 `models/traffic_light/best.pt`; legacy String topic 병행 |
| recorder·bag profile·replay·Capture·dataset | `src/data_collection/` | 인지 runtime과 분리한 데이터 수집 package |
| 정적 도로·mission context | `src/asmc_hd_map/` | 완성된 별도 package, `/hd_map/*`를 통해 소비 |
| 통합 `asmc_perception` | `src/asmc_perception/` | P4-C6 단일 launch와 camera/BEV/lane/traffic runtime·offline·training 소유 |

현재 경로를 목표 경로처럼 부르거나 먼저 삭제하지 않는다. 특히 `src/lidar+sensorfusion/`에는
`Main`, 중복 `Msg`·`morai_msgs`와 vendor Velodyne package가 함께 있으므로 한 번에 이동할 수
없다. runtime consumer parity를 확인한 뒤 팀 소유 코드만 단계적으로 이식한다.

## 3. 현재 runtime 데이터 흐름

```text
allowed camera topics
  └─ asmc_perception.camera
       ├─ per-view drivable / road-marking
       └─ asmc_perception.bev
            ├─ fused camera BEV
            └─ provisional lane boundary observations ──> planning adapter

allowed VLP16
  └─ lidar+sensorfusion ──> object detection / tracking / camera class fusion

extra camera
  └─ asmc_perception traffic_light ──> typed detection + legacy state/detail

/hd_map/* ──> static lane, drivable area, checkpoint and mission context
```

Camera BEV와 HD Map 주행가능영역은 같은 정보가 아니다.

- HD Map은 `map` 절대좌표의 정적 도로 구조와 mission metadata를 제공한다.
- camera BEV는 `base_link`의 현재 관측 probability·coverage를 제공한다.
- LiDAR는 현재 장애물 점유·추적을 담당한다.
- 최종 safe corridor, 차로 선택, 장애물 회피와 복귀 결정은 planning/fusion consumer가 소유한다.

## 4. 현재 camera·BEV interface

Camera semantic 입력·출력의 정확한 목록과 실행 인자는
[asmc_perception package README](../../../src/asmc_perception/README.md)를 따른다.
주요 per-view 출력은 원본 timestamp를 보존한다.

```text
/perception/camera/{view}/drivable/probability
/perception/camera/{view}/road_marking/class_id
/perception/camera/{view}/road_marking/confidence
```

3-view metric BEV의 정식 camera 출력은 `frame_id=base_link`인 raster다.

```text
/perception/bev/camera/drivable_probability
/perception/bev/camera/road_marking/class_id
/perception/bev/camera/road_marking/confidence
/perception/bev/camera/coverage
/perception/bev/diagnostics
```

`/perception/bev/camera/source_count`, `/perception/bev/debug/source_view`와 per-view
projected coverage는 `publish_bev_debug_metadata:=true`인 평가·검수 실행에서만 발행한다.
frame별 상세 `/perception/lane/pipeline_audit`도
`lane_publish_pipeline_audit:=true`인 result-bag 평가에서만 발행한다. 일반 주행에서 유지되는
`/perception/lane/diagnostics`와 혼동하지 않는다.

P5-A 감사 결과 `/perception/bev/debug/{view}`의 drivable·road-marking 3종은 이름과 달리
fusion의 필수 내부 입력이다. `model_grid` 2종도 lane vectorizer 입력이다. 실제 default-off
후보와 bag 영향은 [topic policy](../../../config/contracts/perception_topic_policy_v1.yaml)를
따르며 debug prefix만 보고 필수 transport producer를 끄지 않는다.

음영구간 typed 후보는 다음 토픽이다.

```text
/perception/lane/boundary_observations
  asmc_msgs/LaneBoundaryObservationArray
```

이 observation에는 절대 차로 번호, `home_lane`이나 `opposing_lane` 의미가 없다. consumer는
공통 `reference_x_m`, `order_valid`, source time, support와 score를 확인한 뒤 route/HD Map
문맥을 결합해야 한다. 상세 필드, invalid 상태와 판제 변환법은
[shadow camera handoff](../../handoffs/perception/shadow-camera.md)가 우선한다. 날짜별 threshold와
평가 근거는 [BEV 개발 이력](../../../src/asmc_perception/docs/bev-development-history.md)에 보존돼 있다.

## 5. Traffic light와 LiDAR 경계

`asmc_perception`의 traffic-light producer는 `/cam_extra/image_jpeg/compressed`를 구독해 typed
detection과 기존 문자열 state·JSON detail을 병행 발행한다. `behavior_planner`는 typed 입력을
우선 사용하고 producer가 전혀 없는 구 배포에서만 String으로 fallback한다. K-City HD Map에는
신호등 필요 구간이 metadata로 들어 있지만, 그
구간에서만 inference process를 켜고 끄는 orchestration은 아직 구현됐다고 간주하지 않는다.
실행과 stale 처리 계약은 [runtime contract](../../contracts/perception/runtime-v1.md)를 따른다.

LiDAR runtime은 독립 nested catkin workspace다. outer workspace의 sensor bridge를 먼저 띄우고
별도로 build/run한다. current object topic, time matching, class hysteresis, GPU 세대별 실행과
RViz 절차는 [sensor-fusion guide](../../../src/lidar+sensorfusion/RUN_SENSOR_FUSION.md)가 정본이다.
본선 v3 VLP16 설정과 손재호 소비 loop는 15 Hz이고 실제 완성 cloud 약 12 Hz는 허용한다.
v2 10 Hz는 과거 비교·회귀 기준이므로 본선 입력과 혼동하지 않는다.

현재 파이프라인의 단계별 완성도, 증상별 원인, GT → 센서 전환에서 판제 쪽에도 필요한 변경과
개선 우선순위는 [LiDAR assessment](./lidar-assessment-and-roadmap.md)에 있다. 실행 절차가 아니라
평가와 제안이므로 `RUN_SENSOR_FUSION.md`의 정본 지위를 대체하지 않는다.

## 6. 개발 입력과 본선 입력을 분리한다

본선 runtime은 주최 측이 허용한 UDP camera, VLP16, GPS/IMU와 차량 상태만 입력으로 사용한다.
다음 입력은 학습·검증·scenario 자동화에는 쓸 수 있지만 배포 추론이나 제어의 oracle로 사용하지
않는다.

- Capture Mode와 gRPC
- Semantic/Instance GT camera·LiDAR
- Object Info와 simulator traffic-light state
- GPS blackout 의도를 우회하는 비허용 전역 위치
- GT에서 파생한 mask, pseudo-GT와 평가 전용 map overlay

개발 결과에는 입력 profile, bag parent, model/checkpoint SHA, config와 simulator version을 남긴다.
GT를 사용한 artifact를 일반 raw/result bag과 이름만으로 구분하지 않는다.

## 7. 좌표·시간·artifact 규칙

- camera metric BEV와 lane observations는 `base_link`, x-forward/y-left/z-up이다.
- HD Map vector와 polygon은 `map` 좌표다. consumer가 이미 `map`인 geometry에 다시
  `base_link` transform을 적용하면 안 된다.
- sensor result는 source image/point-cloud timestamp를 보존한다. callback wall time으로 바꿔
  camera–LiDAR matching이나 ego pose alignment를 하지 않는다.
- runtime weight와 대용량 evaluation artifact는 Git에 넣지 않는다. repository에는 config,
  manifest, source/checkpoint SHA와 재생성 명령을 둔다.
- `score`나 probability를 calibration된 안전 확률로 간주하지 않는다. coverage, freshness와
  invalid 상태를 함께 전달한다.

## 8. 실행·검증 진입점

| 작업 | 실행 문서 |
|---|---|
| camera live inference·mosaic | [asmc_perception README](../../../src/asmc_perception/README.md) |
| 3-camera BEV·lane pipeline·audit | [BEV 개발 이력](../../../src/asmc_perception/docs/bev-development-history.md) |
| 학습 adapter·checkpoint 연결 | [TwinLite training README](../../../src/asmc_perception/training/README.md) |
| raw/result bag·Capture·dataset | [data collection README](../../../src/data_collection/README.md) |
| LiDAR+camera fusion | [sensor-fusion guide](../../../src/lidar+sensorfusion/RUN_SENSOR_FUSION.md) |
| traffic light | [asmc_perception README](../../../src/asmc_perception/README.md) · [이전 상세 기록](../../../src/asmc_perception/docs/traffic-light-history.md) |

소스나 interface를 바꾸면 최소한 해당 unit test, raw bag replay, topic type/rate/frame/stamp,
invalid·stale 처리와 downstream adapter를 같이 확인한다. 모델 수치가 좋아졌다는 이유만으로
runtime 계약이나 deployment 허용 상태가 자동 승격되지는 않는다.

## 9. Phase 4 통합 gate

P4-A 기준선은 완료됐다. 기계 정본은
[`perception_runtime_baseline_v1.yaml`](../../../config/baselines/perception_runtime_baseline_v1.yaml),
사람이 검토할 결과는 [P4-A record](../../records/baselines/perception-p4a-runtime-v1.md)다.
이 기준선은 현 구조를 목표로 승인하지 않고 안전한 이관을 위한 비교점으로만 사용한다.

1. **완료:** 현재 package, launch, topic/type/frame/stamp와 model artifact baseline을 기록한다.
2. **완료:** camera, BEV, traffic-light runtime과 offline train/eval/curation을 분류한다.
3. 공용 message 변경은 `asmc_msgs` consumer와 MD5 영향을 먼저 검토한다.
4. **완료:** 새 namespace로 runtime을 이식하고 기존 launch와 bag replay를 병행 검증한다.
5. **완료:** 대표 raw/result bag에서 output parity, rate/latency와 invalid/stale 동작을 비교한다.
6. nested `Msg` consumer를 `asmc_msgs`로, 중복 `morai_msgs`를 pinned submodule로 전환한다.
7. **완료:** 학습 adapter와 offline BEV 분석을 `asmc_perception`의 명시적 비-runtime 경계로 옮기고 import 경계를 시험한다.
8. **완료:** planning adapter와 simulator smoke, package reference gate 후 기존 camera/BEV/traffic package를 제거한다.
9. **완료:** `learning_by_cheating`의 production과 무관한 privileged teacher/debug 기능만
   `tools/research/lbc_teacher_bev`로 이관하고 구 학습·제어 prototype을 제거했다.

Vendor Velodyne driver는 팀 인지 runtime과 같은 package로 합치지 않는다. 단일 package라는 목표가
실패 경계와 upstream ownership을 없앤다는 뜻은 아니다.

P4-B에서 canonical `asmc_perception` façade와 additive interface를 추가했다. 실행과 현재 제한은
[package README](../../../src/asmc_perception/README.md), typed topic의 frame/stamp/freshness 및
cutover gate는 [Perception runtime contract v1](../../contracts/perception/runtime-v1.md)이 정본이다.
traffic-light String과 LiDAR `Msg/TrackedObject*`는 그대로 유지된다. 신호등 typed topic은
P4-C1에서 source image header를 보존해 병행 발행하며 P4-C2에서 대표 bag의 기존/new
String·typed·stale·recovery parity를 통과했다. LiDAR typed topic은 아직
`declared_not_published_p4b`다.

P4-C는 camera/BEV/lane/traffic-light runtime을 한 단위씩 이식한다. P4-C1은
traffic-light가 실제 추론 입력 image header를 `TrafficLightDetection`에 보존해 legacy String과
병행 발행하고 GPU 컨테이너 실행·시각화 진입점을 고정했다. P4-C2는 producer parity를
`config/baselines/perception_traffic_light_parity_v1.yaml`로 고정했다. P4-C3는
`behavior_planner`를 typed 우선·legacy fallback으로 연결하고 invalid/stale fail-safe와 실제 ROS
연결을 검증했다. P4-C4는 production behavior node와 현재 지도/route를 쓰는 격리 ROS loop에서
`GREEN→KEEP→RED→STOP→GREEN→KEEP`을 확인하고 camera/BEV/lane node 구현을 물리 이식했다.
별도 guarded 시험은 Frenet/MPC/UDP를 거쳐 물리 정지선 5.21 m 전 정지와 녹색 재출발, 원래
Ego 상태 복원을 확인했다. live camera/BEV/lane도 검사했다. P4-C5는
runtime support/config를 canonical package로 이관하고 기존 import·config 경로를
호환 전달자와 SHA mirror로 고정했다. 동일 raw bag 재생에서 front class-ID, fused BEV와 lane
observation의 공통 source-stamp payload mismatch가 0임을 확인했다. 상세 근거는
`config/baselines/perception_traffic_light_vehicle_closed_loop_v1.yaml`과
`config/baselines/perception_runtime_move_smoke_v1.yaml`,
`config/baselines/perception_support_config_parity_v1.yaml`이다. P4-C6은 offline·training·model
자산을 역할별 하위 경계로 이관하고 `camera_semantic_perception`, `drivable_bev`, `traffic_light`,
`src/perception/morai_3d_detection` 및 빈 `src/learning` placeholder를 제거했다. topic 호환은
유지하며 과거 A/B parity 수치는 동결 baseline과 Git history로 보존한다.
`data_collection`은 recorder와 bag profile을 계속 소유하는 별도 package이며 P4-C6 제거 대상이 아니다.
담당 팀원이 개발 중인 `src/lidar+sensorfusion`과 그 내부 `Msg`·`morai_msgs`는 이 범위에서
변경하지 않는다. LiDAR 완료·handoff 뒤 canonical adapter와 message 전환을 별도 P4-D로 진행한다.

## 10. 이번 문서 통합에서 보존한 것

긴 roadmap, BEV 설계 노트, package README와 handoff·record의 본문을 줄이거나 이동하지 않았다.
새 guide는 현재 상태를 찾아가는 색인만 제공한다. 절별 행선지와 미변경 범위는
[D2 perception 통합 기록](../../structure/d2-perception-report.md)에서 확인한다. D2 records 감사에서도
날짜별 분석은 실행·I/O 문맥과 기존 anchor를 보존하기 위해 원래 package README에 유지했다.
현재 위치와 이후 이동 gate는 [records index](../../records/README.md#4-package에-남아-있는-날짜별-로그)를
따른다.
