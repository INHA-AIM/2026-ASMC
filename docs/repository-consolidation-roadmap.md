# 저장소 4-패키지 통합·HD Map 메타데이터·현장 배포 로드맵

> **문서 역할:** Working Proposal — 패키지 통합과 본선 배포를 위한 단계별 실행 계획
> **작성 기준:** 2026-09-20 저장소와 현장 점검 반출 자료의 read-only 감사
> **진행 상태:** 1단계 P1-A~P1-D 완료, 2단계 P2-A/P2-B compact geometry 완료
> **보호 범위:** `behavior_planner`, `frenet_planner`, `mpc_controller`,
> `planning_control/lane_boundary_adapter` 및 planning 소유 launch는 이 계획의 일방적 이동·삭제 대상이 아니다.

## 1. 결론

팀이 직접 유지하는 ROS 코드는 다음 네 패키지를 정본으로 수렴시킨다.

| 정본 패키지 | 책임 |
|---|---|
| `asmc_msgs` | 팀 내부 typed message의 유일한 공용 패키지 |
| `asmc_bridge` | MORAI UDP 수신·제어 송신, sensor decode, 외부→내부 상태 정규화 |
| `asmc_hd_map` | HD Map build/runtime, 차선·차로·주행가능영역, 체크포인트·미션 메타데이터 |
| `asmc_perception` | camera/LiDAR/traffic-light inference, BEV·후처리·fusion, runtime 기록 |

여기서 “네 패키지”는 **팀 소유 공용·인지 코드의 경계**다. 다음은 예외로 남긴다.

- 판제 소유 패키지는 현재 이름과 경로를 유지한다.
- `MORAI-ROS_morai_msgs`는 외부 wire contract이므로 `asmc_msgs`로 복사하지 않고 핀된
  dependency로 유지한다.
- Velodyne driver처럼 upstream ROS 패키지인 코드는 우리 perception 패키지로 합치지 않는다.
  apt dependency 또는 `vendor/`로 격리한다.
- 학습·dataset bake·일회성 분석은 catkin `src/`에서 빼되, runtime perception 코드와
  재현 가능한 설정은 보존한다.

패키지 이동보다 먼저 메시지·토픽·정적 데이터 계약을 동결한다. 넓은 rename과 삭제를 먼저
하면 현재 HD Map 검수 결과, 판제 consumer, UDP 현장 설정 중 무엇이 깨졌는지 분리하기 어렵다.

## 2. 현재 감사 결과

2026-09-19 기준 `src/`에는 `package.xml`이 24개 있다. 이 수에는 팀 패키지뿐 아니라 중복
`morai_msgs`, `Msg`, 여섯 Velodyne 패키지가 포함된다. 루트 `scripts/`에는 실행·학습·분석
스크립트 30개가 한 수준에 섞여 있다.

| 현재 영역 | 판단 | 처리 방향 |
|---|---|---|
| `asmc_msgs` | 공용 메시지 13개, 이미 정본 후보 | 유지·확장 |
| `behavior_planner/msg` | planning 소유이며 공용 메시지와 일부 중복 | 1단계에서 수정 금지. planning 협의 후 adapter 또는 별도 이관 |
| `lidar+sensorfusion/src/Msg` | `TrackedObject*`, `Yolo_bbox*` 4개가 별도 메시지 패키지 | 소비자 이관과 함께 `asmc_msgs`로 흡수 |
| `morai_udp_bridge` + `morai_sensor_bridge` + `vehicle_state` | 한 외부 경계가 세 패키지로 분산 | `asmc_bridge`로 통합 |
| `integration_launch` | planning 공동 소유 | 이동하지 않고 새 bridge launch를 include하는 호환 계층만 협의 |
| `asmc_hd_map` + `mgeo_toolkit` | runtime·builder는 통합 완료, 외부 toolkit은 provenance 입력 | geometry parity 후 toolkit 의존 축소 |
| `camera_semantic_perception` + `drivable_bev` + `traffic_light` | 한 camera pipeline이 세 패키지로 분산 | `asmc_perception`의 하위 모듈로 통합 |
| `morai_3d_detection` + `lidar+sensorfusion/src/Main` | 학습·검증 산출물·runtime·vendor가 혼재 | runtime만 `asmc_perception`으로 이식, offline은 `tools/perception`으로 분리 |
| `data_collection` | ROS runtime recorder와 dataset 도구가 혼재 | recorder는 `asmc_perception`, offline 도구는 `tools/perception` |
| `src/learning` | README 1개뿐인 논리 폴더 | 문서 링크 이관 후 삭제 |
| `src/learning_by_cheating` | catkin 패키지는 아니나 38개 tracked file과 실행 참조가 남음 | 즉시 삭제 금지. 유효 참조 제거 후 git 이력에만 보존하고 삭제 |
| `src/lidar+sensorfusion` 내부 `build/`, `devel/` | 중첩 workspace 산출물 | source 이식 확인 후 로컬 산출물 제거·ignore 강화 |

`learning_by_cheating`은 현재 배포 경로의 필수 패키지는 아니지만 바로 삭제할 수는 없다.
`tools/aim_scenario_runner`가 LBC visualizer를 실행하고, 진단 스크립트와 여러 문서가 경로를
직접 참조한다. `lbc_bev`의 map crop/renderer 기능이 새 HD Map·BEV 코드로 대체됐는지 테스트한
후 삭제해야 한다. 반면 비어 있는 `src/learning`은 참조 문서만 고치면 제거 가능하다.

## 3. 목표 디렉터리

```text
src/
├── asmc_msgs/                 # 팀 내부 message만
├── asmc_bridge/
│   ├── src/asmc_bridge/       # network, sensor, vehicle_state
│   ├── config/profiles/       # dev_local, competition_lan
│   ├── launch/
│   └── test/
├── asmc_hd_map/
│   ├── src/asmc_hd_map/       # build, geometry, runtime crop, validation
│   ├── config/kcity2025/      # curation + competition metadata
│   ├── maps/kcity2025/        # 작고 versioned인 최종 bundle/manifest
│   ├── launch/ · rviz/
│   └── test/
├── asmc_perception/
│   ├── src/asmc_perception/   # camera, lidar, traffic_light, bev, fusion
│   ├── config/ · launch/ · rviz/
│   ├── scripts/               # ROS node entrypoint만
│   └── test/
├── behavior_planner/          # 판제 소유, 현 위치 유지
├── frenet_planner/            # 판제 소유, 현 위치 유지
├── mpc_controller/            # 판제 소유, 현 위치 유지
├── integration_launch/        # 소유자 협의 전 유지
├── planning_control/          # 판제 adapter, 현 위치 유지
└── MORAI-ROS_morai_msgs/      # pinned external dependency

tools/
├── perception/                # train/eval/dataset/bake/analysis
├── hd_map/                    # 대용량 원본 검사·one-shot migration
└── field/                     # 현장 profile 검사·packet audit

vendor/                        # 소스 설치가 꼭 필요한 upstream만 격리
scripts/
└── asmc                       # 사람이 사용하는 단일 command entrypoint
```

`asmc_perception` 하나에 모든 연구 파일을 넣는다는 뜻은 아니다. `src/`에는 대회 runtime만
두고 학습과 분석은 `tools/perception`에서 같은 Python library를 import한다. 이 구분이 있어야
catkin package 하나를 유지하면서도 실행 이미지가 학습 산출물로 비대해지지 않는다.

## 4. 개발 단계

| 단계 | 목표 | 주요 산출물 | 삭제 허용 |
|---|---|---|---|
| **1** | 계약·메타데이터 동결과 이동 안전망 | `asmc_msgs` v1, topic contract, 체크포인트/미션 schema, baseline manifest/tests | 없음 |
| **2** | `asmc_hd_map` 제품화 | 최종 lane/lane-area/drivable bundle, global/local publisher, checkpoint·mission RViz | `link_node`는 parity 통과 뒤 가능 |
| **3** | `asmc_bridge` 통합 | UDP·sensor·vehicle state 단일 package, dev/competition profile | 구 bridge는 field replay 통과 뒤 가능 |
| **4** | `asmc_perception` 통합 | camera/LiDAR/traffic-light/BEV runtime 하나, offline tool 분리 | `learning*`, 중복 `Msg`·`morai_msgs`는 gate 뒤 가능 |
| **5** | 실행·시각화 정리 | `scripts/asmc`, 통합 launch/RViz/rqt, 이후 read-only perception web console | 호환 wrapper 만료 뒤 가능 |
| **6** | 현장 LAN2LAN·Docker 동결 | competition image, network profile, rehearsal report, 당일 runbook | 본선 직전 기능 삭제 금지 |

각 단계는 별도 PR로 진행한다. rename과 동작 변경을 같은 PR에 넣지 않으며, 이전 경로를
지우는 PR은 새 경로의 bag replay·launch smoke 결과를 첨부한 뒤 연다.

## 5. 1단계 상세 계획 — 계약·메타데이터 동결

### 5.1 목표

1단계는 폴더를 대량 이동하지 않는다. 네 정본 패키지가 공유할 **메시지, 토픽, 정적 map
metadata, 검증 기준**을 먼저 고정한다. 이 단계가 끝나면 이후 작업은 파일 위치가 아니라
동일한 계약을 기준으로 병렬 이식할 수 있다.

### 5.2 작업 단위

#### A. 현재 runtime baseline 고정

**완료.** 기계 판독 manifest는
[`config/baselines/runtime_baseline_v1.yaml`](../config/baselines/runtime_baseline_v1.yaml), 사람이
읽는 결과는 [P1-A runtime baseline](./baselines/runtime-baseline-v1.md), 재생성 도구는
`tools/audit/build_runtime_baseline.py`다.

- 현재 catkin package 24개, launch 51개와 node 선언 56개, package dependency, executable과
  정적 topic 참조를 추출했다.
- P1-A 당시 팀/runtime message 22개를 동결했고, P1-B의 additive 타입 11개를 포함하도록 현재
  manifest를 33개로 재생성했다. 기존 13개 `asmc_msgs` MD5는 P1-A 값과 동일하다.
- HD Map stage14의 build ID, file SHA와 geometry-only SHA를 남겼다. 기준은 lane marking
  121개(white 94, yellow 27, solid 90, broken 31), stopline 7개다.
- 같은 시나리오의 대표 raw/result bag에서 topic 이름·rate·message count·type·MD5를 ROS CLI
  없이 bag v2 index로 읽고, bag 자체는 복사하지 않은 채 `$ASMC_DATA` 상대 경로·크기·SHA만
  저장했다.
- Autonav v2 센서의 현재 기준은 VLP16 10 Hz다. 손재호 LiDAR runtime을 위한 15 Hz 변경은
  3단계 승인 요구사항으로 따로 기록했으며 아직 센서 JSON을 바꾸지 않았다.
- planning 소유 경로, 센서·network config, HD Map geometry에는 변경을 만들지 않았다.

#### B. `asmc_msgs` v1 동결

**완료.** 기존 13개 메시지의 ROS1 MD5를 유지하고 `asmc_msgs` 1.0.0에 다음 11개 타입을
추가했다. 기계 판독 정본은
[`config/contracts/hd_map_topic_contract_v1.yaml`](../config/contracts/hd_map_topic_contract_v1.yaml),
사람이 읽는 규약은 [HD Map topic contract v1](./contracts/hd-map-topic-contract-v1.md)이다.

| 메시지 계열 | 필수 정보 |
|---|---|
| `HdMapMeta` | schema/build/map ID, frame, source hash, 생성 시각, global/local 여부 |
| `LaneMarking` / `LaneMarkingArray` | geometry, white/yellow, solid/broken, 폭, physical/logical 역할, source provenance |
| `Lane` / `LaneArray` | lane ID·차로 번호, centerline·polygon, left/right boundary ID, adjacency, 진행 방향 |
| `DrivableArea` / `DrivableAreaArray` | polygon ID, lane/junction 여부, source, validity |
| `CompetitionCheckpoint` / `...Array` | 순서, pose, 공식 yaw, 3 m 반경, route station, link ID |
| `MissionZone` / `MissionZoneArray` | mission type, start/end station·link, official/provisional 상태, 규칙 payload |

래스터는 새 custom message를 만들지 않고 `nav_msgs/OccupancyGrid`를 쓴다. 정적 geometry는
latched publish하고 local crop은 ego pose가 들어올 때 갱신한다.

동결한 토픽은 다음과 같다.

```text
/hd_map/meta
/hd_map/global/lane_markings
/hd_map/global/lanes
/hd_map/global/drivable_areas
/hd_map/global/drivable_grid
/hd_map/global/checkpoints
/hd_map/global/mission_zones
/hd_map/local/lanes
/hd_map/local/drivable_grid
/hd_map/debug/markers
```

메시지에 `lane_number`를 두되 도로 전체에서 고정된 숫자로 가정하지 않는다. 같은 진행 방향의
단면별 왼쪽→오른쪽 순서와 유효 station interval을 함께 저장해 합류·분기에서 번호가 바뀌는
문제를 피한다. 차선 도색(`LaneMarking`)과 통행 가능한 차로 영역(`Lane`)도 분리한다.

`tools/audit/validate_p1b_contract.py`는 ROS 설치 없이 기존/신규 message MD5, 필수 필드,
CMake 등록, package version, topic latch/scope를 검사한다. ROS Noetic build 후에는
`src/asmc_msgs/test/test_message_serialization.py`로 generated type의 MD5와
serialize/deserialize 왕복을 검사한다.
P1-B에서는 publisher와 geometry를 추가하지 않았고 P1-C metadata 구현이 다음 작업이다.

#### C. 체크포인트 정본화

**완료.** source 설정은
[`kcity2025_competition_metadata_source.yaml`](../src/asmc_hd_map/config/kcity2025_competition_metadata_source.yaml),
생성 bundle은
[`kcity2025_competition_metadata.yaml`](../src/asmc_hd_map/config/kcity2025_competition_metadata.yaml),
사람이 읽는 정본은
[K-City competition metadata v1](./contracts/kcity2025-competition-metadata-v1.md)이다. 공식
checkpoint/path, current link set, reviewed route decomposition과 stage14 map SHA를 모두 고정하며
입력이 바뀌면 `--write`도 실패한다.

공식 입력은
`$ASMC_DATA/R_KR_PG_K-city_2025/2026_molit_comp_checkpoints.json`으로 두고, runtime bundle에는
source SHA와 필요한 필드만 복제한다. organizer yaw는 degree 원문을 보존하고 quaternion은
publish 시 생성한다. START/END는 폐곡선의 station `0`과 `2184.61 m`가 같은 위치인 특수점으로
취급한다.

현재 공식 path와 2025 link centerline에 투영한 결과는 아래와 같다. 모든 checkpoint의 공식
좌표는 global path 위에 있으며 XY 투영 오차는 표시 정밀도에서 `0.000 m`다.

| CP | route s (m) | current link | link s (m) |
|---:|---:|---|---:|
| START/END | 0.00 | `A2256W000748` | 0.00 |
| 1 | 72.63 | `A2256W000748` | 72.63 |
| 2 | 186.29 | `A2256W000236` | 26.28 |
| 3 | 237.42 | `A2256W000215` | 28.96 |
| 4 | 346.74 | `A2256W000222` | 30.84 |
| 5 | 471.09 | `A2256W000205` | 0.00 |
| 6 | 580.87 | `A2256W000318` | 30.25 |
| 7 | 635.11 | `A2256W000083` | 25.24 |
| 8 | 858.50 | `A2256W000304` | 223.39 |
| 9 | 930.60 | `A2256W000866` | 15.76 |
| 10 | 1118.74 | `A2256W000846` | 134.03 |
| 11 | 1286.15 | `A2256W000411` | 167.41 |
| 12 | 1593.59 | `A2256W000445` | 75.34 |
| 13 | 1741.72 | `A2256W000153` | 148.13 |
| 14 | 1859.64 | `A2256W000446` | 78.08 |
| 15 | 1996.73 | `A2256W000126` | 118.99 |

검증기는 총 16개, 순서 중복 없음, 반경 3.0 m, path 투영 오차, link 존재, yaw 유한값을
검사한다. 디버그 marker는 반경 3 m 원/원통, heading 화살표, ID·station text를 표시한다.

#### D. 미션 구간의 확정도 분리

규정 문장, 공식 그림, 팀 map-review 해석을 같은 등급으로 저장하지 않는다.

| 구간 | 현재 경계 | 상태 |
|---|---|---|
| 출발 성공 | 전체 2184.61 m의 5%인 `s=109.23 m`, `A2256W000329` 내부 | 공식 규칙에서 계산 가능 |
| 기본 속도 제한 | `A748` 시작→`A846` 종료 및 `A451` 시작→`A751` 종료 60 kph | 공식 규칙을 link 경계에 투영 |
| 고주로 속도 예외 | `A2256W000411` 시작 `s=1118.74` → `A2256W000153` 끝 `s=1741.72` | 공식 link 경계. 별도 제한값은 규정에 없으므로 `null/exempt` |
| GPS 음영 | `A2256W000448` 다음 노드부터 `A2256W000126` 중심 구간 후보 | 사용자 map review·기존 config와 일치, 공식 수치 경계 재확인 필요 |
| 정적 장애물 | `A2256W000213` 종료 뒤부터 `A2256W000220` 시작 전까지의 `A2256W000222` 교차로 영역 | 팀 질의응답·상황 검토에 따른 비공식 경계 |
| 동적 장애물 | `A2256W000728→599→207→205` | 팀 질의응답·상황 검토에 따른 비공식 경계 |
| 회전교차로 | `A2256W000148→146→151` 주변 후보 | 공식 그림을 link에 매핑한 후보 |
| 로터리 후 고주로 이전 | `A2256W000866→054→846` | 별도 미션 없음, 전역경로 추종 및 60 kph 구간 |
| 주변 차량 인지 | `A2256W000411→420→408→445→153` 후보 | 공식 그림과 속도 예외 corridor가 일치 |
| 신호등 | `A2256W000236→…→A2256W000308` 인지 활성화 + 검수 정지선 기반 경계 5개 | 활성화 policy와 정지 경계를 분리하며 구간 밖 노드 중지 가능, 공식 수치 경계 재확인 필요 |

후보 구간은 RViz에서 공식 그림, checkpoint, link ID를 한 화면에 놓고 사람이 승인하기 전까지
`provisional`로 publish한다. 판제가 `provisional` mission을 강제 규칙으로 소비하지 않도록
메시지에 상태를 필수 필드로 둔다.

P1-C bundle에는 A236→A308 신호등 인지 활성화 policy를 포함한 authored zone 10개와
traffic-light ID가 있는 검수 정지선에서 파생한 신호등 경계 5개, 총 15개가 들어간다. 상태는
`derived_official=4`, `provisional=11`이며
모든 provisional zone은 `planning_enforcement_allowed=false`다. 신호등과 팀 link 해석 후보는
P1-D에서 표시하지만 사람 승인 전 판제 강제 규칙으로 사용할 수 없다.

#### E. 1단계 테스트와 완료 조건

- `asmc_msgs` 단독 catkin build와 기존 consumer build가 통과한다.
- 기존 message MD5와 topic은 1단계에서 바뀌지 않는다.
- 16개 checkpoint가 global path 3 m corridor 안에 있고 route order가 단조 증가한다.
- `START_END` 폐곡선 처리와 5% station 계산 단위 테스트가 있다.
- mission zone은 `official`, `derived_official`, `provisional` 중 하나가 없으면 schema validation에 실패한다.
- HD Map baseline hash/count와 대표 topic manifest가 CI artifact로 생성된다.
- planning 소유 파일 diff가 0이다.

P1-A 자체 완료 조건인 manifest/report byte-for-byte 재생성 검증은
`python3 tools/audit/build_runtime_baseline.py --check`로 통과했다. ROS Noetic 컨테이너에서는
`asmc_msgs` 24종 생성, 기존 13종의 실제 `rosmsg md5`, generated-message serialization과
P2-A workspace 회귀, source 비교와 local crop 계약을 포함해 `asmc_hd_map` 전체 106개
테스트를 통과했다.

P1-C 기준으로 checkpoint 16개의 route/link XY 투영 오차는 모두 `0.0 m`, route station은
단조 증가하며 START/END station 0과 5% station `109.230586167 m` 단위 테스트가 통과했다.
P1-D publisher를 실제 ROS master에 실행한 뒤 늦게 접속한 validator가 latched 전역 토픽과
1 Hz non-latched MarkerArray를 구독했다. marking 128개, checkpoint 16개, mission/policy/speed zone
15개, debug marker 205개와 geometry SHA가 모두 기준값과 일치했다. 속도 profile의 전 구간
coverage와 A054·A846 미션 제외도 runtime validator가 확인한다.

### 5.3 1단계 PR 분할

1. **P1-A audit (완료):** package/topic/message/sensor/bag/HD Map baseline manifest와 dependency report
2. **P1-B interface (완료):** `asmc_msgs` HD Map·competition metadata message, frozen topic contract와 MD5/serialization test
3. **P1-C metadata (완료):** checkpoint source import·projection validator·권위별 mission-zone bundle
4. **P1-D debug (완료):** SHA-gated read-only publisher, runtime topic validator와 RViz preset

P1-C/P1-D와 P2-A/P2-B 구현은 `asmc_hd_map` 한 패키지로 이동했다. 최종 차선, metadata,
compact vector/grid publisher의 토픽 계약은 유지하며 패키지 이동 전후 geometry parity를
회귀 검사한다.

## 6. 2단계 이후 구현 메모

### 6.1 `asmc_hd_map`과 주행가능영역

- **P2-A 완료:**
  [`kcity2025_drivable_area_input_audit.yaml`](../src/asmc_hd_map/config/kcity2025_drivable_area_input_audit.yaml)에
  39개 전역경로 링크의 lane-clip 후보와 `intersection_guarded` 보호구간을 동결했다. 기존 H5
  road는 진짜 vector road mesh가 아니라 Node/Link 중심선 buffer에 실측 보정을 더한 raster임을
  명시했으며, 16개 checkpoint 중심 포함·폐곡선 topology·입력 SHA·stage14 geometry hash를
  검증했다. 이 단계에서는 geometry와 publisher를 변경하지 않았다.
- **P2-B v5 완료:** 전역경로 39개, 같은 방향 usable 차로, 명시 장애물 link, A134 로터리와
  음영구간 반대차로 A131을 선택한다. 70개 Lane polygon은 endpoint node 법선의 flat cap으로 끝나며,
  41개 Lane/74개 측면은 신뢰 가능한 reviewed paint 횡거리로 보정한다. MGeo 인접차로와
  A126/A131 사이에 vector corridor fill 18개를 추가한다. 정적·동적 교차로는 선택 link
  envelope 안에서만 legacy H5 road를 gap patch로 사용하여 단일 연결·내부 hole 0개를
  보장한다. A134는 2 m 확장 envelope에서 원본 H5 포장면을 복원하고 중앙 섬을 제외한다.
  총 DrivableArea는 101개이며 `OccupancyGrid`는 0.2 m 파생 raster다. 면적은
  14,905.16 m²(legacy 전체의 16.59%)이고 checkpoint 16개를 모두 포함한다. A179·A427과
  명시적 갓길·버스·미사용 toll lane은 제외한다.
- **P2-C 통합 완료:** stage14 차선 결과를 geometry regression 기준으로 삼아 builder,
  runtime publisher, 검증기와 RViz를 `asmc_hd_map` 한 패키지로 통합했다. 원태의 이전
  standalone 도구/H5는 `development/won_tae_link_node/`에 read-only provenance 자료로
  격리했고 top-level package는 제거했다.
- **P2-D consumer handoff 완료:** global cell에 정렬된 120 m/0.2 m local grid와 같은 범위의
  Lane subset을 ego update마다 최대 5 Hz로 발행한다. local geometry와 ID는 global `map`
  namespace를 유지하고, 실제 ROS smoke에서 global/local lane 선택·grid cell parity를 검사한다.
- 최종 검토 차선·구 맵 origin 정합 차선·원태 H5 vector를 각각 독립 토픽으로 보는
  `lane_source_comparison.launch`를 개발 전용으로 유지한다. 운영 `/hd_map/global/*`과 섞지 않는다.
- 주행가능영역 원본은 Node/Link road mesh를 사용하되, 양쪽 물리 차선이 신뢰 가능한 일반
  road segment만 차선 안쪽으로 clip한다.
- junction/교차로, boundary가 없는 구간, 합류·분기 전이부에는 `intersection_guard`를 두어
  road mesh를 유지한다. 차선이 없다는 이유로 교차로를 비주행 영역으로 잘라내지 않는다.
- vector polygon과 global/local `OccupancyGrid`를 함께 제공한다.
- 검증 항목은 route 연결성, checkpoint 3 m disk 포함, 교차로 면적 보존, self-intersection,
  lane 번호/adjacency, 기존 차선 geometry hash다.
- 원태 H5는 현재 compact 교차로 gap patch 재생성의 provenance 입력이다. 최종 제품은 이를
  런타임에 읽지 않으며, 별도 artifact 보관 정책과 재생성 parity가 정해지기 전까지
  `development/won_tae_link_node`를 삭제하지 않는다.

### 6.2 `asmc_bridge`

- UDP network/sensor decode와 `vehicle_state` 정규화를 한 패키지에 둔다.
- simulator profile과 transport profile을 분리한다. 예:
  `25s4_molitcomp03 + competition_lan`, `26r1_h3 + dev_local`.
- 외부 `morai_msgs`는 wire adapter에서만 보이고, 내부 node는 가능한 한 `asmc_msgs`를 쓴다.
- privileged Object/GT 토픽은 개발 profile에서만 허용하며 competition launch에서 시작하지 않는다.
- 개발과 현장의 **포트와 채널은 동일하게** 만들고 IP만 profile로 바꾼다. 목표 포트는 Ctrl
  `9093/9094`(mode 2), Competition Vehicle Status `7603/7502`, CollisionData `9091/9092`,
  v2 sensor `9290~9303`이다. 기존 개발값 Ctrl `9111/9112`, Ego Status `909`는 3단계 전환이
  검증된 뒤 폐기한다.
- 현행 v2/제출 Autonav의 VLP16 10 Hz를 손재호 LiDAR runtime 요구에 따라 15 Hz로 바꾼다.
  JSON 수치만 추측해 고치지 않고 MORAI UI에서 15 Hz 설정 후 재저장해 `sensorPeriod`와
  `frequencyIndex`를 확인한다. 30초 이상 `/velodyne_points` 평균 14~16 Hz, packet loss,
  latency/CPU와 tracking 결과를 10 Hz baseline과 비교해야 한다.
- planning 공동 소유 `integration_launch`는 새 launch를 include하도록 별도 승인받고, 그 전에는
  변경하지 않는다.

### 6.3 `asmc_perception`과 `learning*` 정리

- camera semantic, metric BEV/lane family, traffic light, LiDAR runtime, fusion을 하위 Python/C++
  namespace로 이식한다.
- model weight는 repo 밖 artifact로 유지하고 package에는 manifest/SHA/config만 둔다.
- train/eval/bake/curation 스크립트는 `tools/perception`으로 옮긴다.
- `Msg/TrackedObject*`의 실제 consumer를 `asmc_msgs`로 바꾼 뒤 중복 `Msg` 패키지를 제거한다.
- 중복 `lidar+sensorfusion/src/morai_msgs`를 제거하고 pinned submodule 하나만 사용한다.
- LBC visualizer를 새 HD Map/BEV debug node로 교체하고 scenario runner·문서·진단 스크립트의
  참조가 0이 된 뒤 `src/learning`과 `src/learning_by_cheating`을 삭제한다. 별도 `legacy/`
  폴더로 다시 복사하지 않고 git tag/이력으로 보존한다.

### 6.4 스크립트와 시각화

루트에는 사람이 기억할 진입점 하나만 둔다.

```bash
./scripts/asmc build
./scripts/asmc sim up --profile dev_local
./scripts/asmc bridge run --profile competition_lan
./scripts/asmc map run
./scripts/asmc perception run
./scripts/asmc debug rviz
./scripts/asmc verify field
```

기존 30개 스크립트는 `container`, `runtime`, `data`, `train/eval`, `diagnostic`으로 분류한다.
package node는 각 package `scripts/`, offline job은 `tools/`, 사용 중단된 rosbridge/LBC 경로는
호환 기간 뒤 삭제한다. 기존 명령은 한 단계 동안 deprecation warning을 내는 wrapper로만 둔다.

시각화는 먼저 RViz/rqt에 통합한다. raw PV, semantic result, metric BEV, lane fitting, HD Map
lane/area, checkpoint 3 m radius, active mission, topic rate/staleness를 한 launch로 확인한다.
그 토픽 계약이 안정된 뒤 perception 전용 read-only web console을 만든다. 웹에서 차량 제어·
파라미터 변경은 하지 않는다.

## 7. 현장 LAN2LAN·Docker 단계

### 7.1 반출 자료에서 확인된 사실

`$ASMC_DATA/On-site inspection`에는 다음이 있다.

- MORAI `25.S4.MolitComp03` simulator/launcher 전체본: 압축 약 2.7 GB, 해제본 약 3.2 GB
- launcher version: `v.S3.20260714.95`
- 현장 network/sensor/scenario save file
- 시뮬 PC `192.168.0.1` → 참가팀 PC `192.168.0.10`인 LAN profile

현장 센서 폴더에는 다른 팀 설정도 섞여 있다. 우리 팀은 **Autonav**이며,
`0918sensors.json`을 Autonav 설정 근거로 사용하지 않는다. 현재 실행 정본은
`config/ASMC_sensor_set_v2.json`, 현장 `Autonav_sensor.json`은 제출 이력 확인용이다.

두 Autonav 파일의 sensor 구성·주기·UDP 포트는 다음과 같이 동일하다.

| 입력 | 시뮬 host | 팀 PC destination | 주기 |
|---|---:|---:|---:|
| Camera 1/2/3/4 | 9290/9292/9294/9296 | 9291/9293/9295/9297 | 20/20/20/10 Hz |
| VLP16 | 9298 | 9299 | 10 Hz(현재), 15 Hz(3단계 목표) |
| GPS | 9300 | 9301 | 5 Hz |
| IMU | 9302 | 9303 | 50 Hz |

semantic 차이는 자유 카메라 4번 pitch가 현재 v2 `328°`, 제출본 `340°`인 한 항목뿐이다.
현재 실행 기준은 v2 `328°`를 유지한다.

현장 `2026_molit_comp_network.json`의 Ctrl profile에는 Host `192.168.0.1:9093`, Destination
`192.168.0.10:9094`가 들어 있고, Competition Vehicle Status `7603/7502`, Collision
`9091/9092` 등 non-loopback port pair가 저장돼 있다. Ctrl은 팀 PC에서 시뮬 Host로 보내는
채널이므로 JSON 필드의 나열 순서를 패킷 진행 방향으로 해석하지 않는다. PUBSUB enum과 UI
이름은 실제 simulator 화면에서 다시 대조한다.

3단계에서 개발도 이 포트를 사용하도록 맞춘다. `dev_local`은 양쪽 IP가 `127.0.0.1`,
`competition_lan`은 시뮬 `192.168.0.1`과 팀 PC `192.168.0.10`이며 **IP만 다르다**. 이 변경은
P1-A 범위가 아니므로 현재 bridge·sensor/network config에는 아직 적용하지 않았다.

### 7.2 배포 원칙

- `dev_local`과 `competition_lan`을 명시적으로 선택하며 자동 IP 추측을 금지한다.
- 3단계에서 생성한 두 profile은 포트·채널·Ctrl mode가 같고 Host/Destination IP만 달라야 한다.
- 본선 팀 PC에서는 native Linux Docker `network_mode: host`를 우선 후보로 검증한다.
  WSL/Docker Desktop host networking의 localhost 실측을 LAN ingress 보장으로 간주하지 않는다.
- NIC 고정 IP·netmask·route는 운영 요원 확인값을 사용하고 Wi-Fi/다중 route 간섭을 제거한다.
- 필요한 UDP만 host firewall에 허용하고, bind 충돌은 시작 전에 `ss`로 검사한다.
- 이미지 tag, image digest, git commit, map bundle hash, model hash, network profile hash를 한
  deployment manifest에 기록한다.
- 인터넷 없이 이미지 load→container start→topic health→주행 시작까지 복구 리허설한다.

### 7.3 현장 검증 게이트

1. 케이블 연결 후 양방향 IP reachability와 route 확인
2. 센서별 UDP packet count/size/rate 확인
3. bridge topic rate와 staleness 확인
4. Ctrl mode 2 명령의 목적지·응답 확인
5. Competition Vehicle Status와 CollisionData를 판정 연습 프로그램과 함께 확인
6. 15분 이상 full stack 주행, CPU/GPU/RAM/packet drop 기록
7. bridge 또는 perception node 단독 재시작 후 복구 확인
8. 네트워크 단절·시뮬 reset·checkpoint respawn 절차 확인
9. VLP16을 30초 이상 측정해 `/velodyne_points` 평균 14~16 Hz, packet loss와 손재호 tracking
   결과를 확인하고 10 Hz baseline과 비교

결과가 통과하면 `network-udp-ros.md`의 competition 절과 UDP 치트시트를 현장 profile로
갱신한다. 개발 profile의 localhost 값은 별도 절에 보존한다.

## 8. 삭제·완료 게이트

| 제거 대상 | 선행 조건 |
|---|---|
| `src/learning` | 문서 index와 README 참조 0 |
| `src/learning_by_cheating` | scenario runner·진단·문서 참조 0, BEV/map 기능 parity |
| `src/asmc_hd_map/development/won_tae_link_node` | H5 provenance를 외부 immutable artifact로 보관하고 compact map 재생성 parity 통과 |
| `morai_udp_bridge`, `morai_sensor_bridge`, `vehicle_state` | `asmc_bridge` bag replay + 현장 profile smoke 통과 |
| `camera_semantic_perception`, `drivable_bev`, `traffic_light`, `data_collection` | 새 launch topic/latency/output parity 통과 |
| 중복 `Msg`, 중복 `morai_msgs` | 모든 consumer include/import 교체 및 clean build |
| 루트 legacy scripts | `scripts/asmc` 대응 명령·문서·CI가 존재하고 호환 기간 종료 |

삭제는 `git rm` 전에 대상 파일 목록과 대체 경로를 PR 본문에 적는다. 대용량 simulator,
rosbag, model checkpoint와 주최 배포 원본은 repo로 옮기지 않고 `$ASMC_DATA`에 유지한다.

## 9. 다음 착수점

**P1-A~P1-D와 P2-A~P2-D를 포함한 2단계는 완료됐다.** `asmc_hd_map`은 최종 차선,
주행가능영역, metadata, global/local runtime publisher, 검증기와 개발 source 비교 화면을
소유한다. 다음 작업은 3단계 `asmc_bridge` 통합과 dev/competition 네트워크·센서 profile이다.
이때 현장 동일 포트와 LiDAR 15 Hz 요구를 적용하되, 기존 bridge는 bag replay와 packet test를
통과하기 전까지 삭제하지 않는다. LAN2LAN 실제 두 PC 리허설은 6단계 gate에서 진행한다.
