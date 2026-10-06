# asmc_hd_map

AutoNav K-City 2025 HD맵의 deterministic builder, 운영 publisher, local cache와 개발 검수
도구를 소유하는 ROS1 package다. Phase 2 통합이 완료됐으며 운영 제품은
`kcity2025_static_junction_widened_drivable_v6`다.

> **문서 역할:** Active Package Guide — deterministic builder, runtime publisher와 개발 검수

- topic·message 의미: [HD Map contract](../../docs/contracts/hd-map/topic-v1.md)
- 판제 소비와 RViz: [runtime contract](../../docs/contracts/hd-map/runtime-v1.md)
- checkpoint·mission: [competition metadata](../../docs/contracts/competition/kcity2025-metadata-v1.md)
- 최종 주행가능영역 기준: [P2-B baseline](../../docs/records/baselines/kcity2025-compact-drivable-v1.md)

message/topic 의미는 HD Map contract, 실행 토픽과 필드 해석은 runtime contract, 공식
checkpoint·mission 값은 competition metadata가 우선한다. 이 README는 패키지를 개발하고
산출물을 재검증하는 방법을 설명한다.

이 패키지는 다음을 함께 소유한다.

- 검토 완료 차선·정지선과 현재 Node/Link topology
- 공식 체크포인트, route station, 속도·미션·인지 활성 metadata
- route-compact Lane과 vector/raster 주행가능영역
- global latched publisher, local 파생 cache와 현재 차로 association
- 통합 RViz와 최종/legacy/원태 source 비교 도구
- 입력 SHA, geometry hash, feature count와 deterministic 회귀 검사

## 빠른 실행

ROS Noetic workspace를 build/source한 뒤 판제용 runtime을 실행한다.

```bash
roslaunch asmc_hd_map competition_map.launch
rosrun asmc_hd_map hd_map_consumer_example.py
rosrun asmc_hd_map verify_competition_map_topics.py
```

RViz를 포함한 검수:

```bash
roslaunch asmc_hd_map competition_map_debug.launch
```

RViz는 기본으로 ego 차량 박스(`base_link`)를 따라간다. 전체 경로 보기로 띄우려면 `follow_ego:=false`를
준다. 기본 표시는 가벼운 `lite`(차선·주행가능영역·체크포인트·미션)다. HD맵 제작 검수용 차로·링크
레이어까지 보려면 `debug_marker_detail:=full`을 준다.

```bash
roslaunch asmc_hd_map competition_map_debug.launch                       # ego 따라가기, lite
roslaunch asmc_hd_map competition_map_debug.launch follow_ego:=false \
  debug_marker_detail:=full                                                # 전체 경로, 제작 검수
```

박스·색·TF 의미는 [runtime contract §4.0](../../docs/contracts/hd-map/runtime-v1.md#40-ego-차량-박스와-base_link-추적)을 본다.

미션 구간은 종류별 namespace(`mission_<type>`, 점선 `mission_surface_<type>`, full 전용 `mission_actors_<type>`)로
나뉘어 RViz에서 따로 켜고 끌 수 있다. 범례는 [runtime contract §4](../../docs/contracts/hd-map/runtime-v1.md#4-rviz-표시)를 보고,
구간별 회피 차로·actor 후보와 샘플링 API는 [미션 시나리오 spec](../../docs/handoffs/hd-map/mission-scenario-spec.md)을 본다.

최종/legacy/원태 source와 최신/과거 주행가능영역 비교:

```bash
roslaunch asmc_hd_map hd_map_source_comparison.launch repo_root:=/root/ws
```

source comparison은 review only다. `/asmc_hd_map/development/*`를 운영
`/hd_map/global/*`과 합치거나 판제 입력으로 사용하지 않는다.

publisher는 입력 SHA나 build 조합이 맞지 않으면 일부 토픽을 내보내지 않고 시작을 거부한다.
판제는 `/hd_map/global/*` latched 배열을 시작 시 한 번 받고 `map` 좌표 Ego로 자체 공간 조회하는
방식을 기본으로 한다. `/hd_map/local/*`은 최적화·시각화를 위한 파생 cache다.

## 현재 제품

| 항목 | 개수·기준 |
|---|---|
| 물리 차선·정지선 | 121 + 7 = 128 `LaneMarking` |
| 차로 | 70 `Lane` |
| vector 주행가능영역 | 102 `DrivableArea` = Lane 70 + corridor fill 18 + surface patch 14 |
| 체크포인트 | 15 (규정집 v1.2), 통과 반경 3 m |
| mission/policy/speed zone | 15 |
| raster | 0.2 m/cell, vector product의 파생 lookup |

판제는 latched global vector/topic을 정본으로 사용한다. local lane/grid는 map frame과 global ID를
유지하는 선택적 cache이며 planner horizon이나 별도 정본이 아니다. 현재 차로 association은
GPS 안테나가 아니라 후륜축 중심 `base_link`의 map 좌표를 기준으로 한다.
기본 local 창은 heading 기준 전방 90 m·후방 30 m·좌우 각 30 m(0.2 m, 600×300, 최대 20 Hz)다.
현재 차로는 pose callback에서 즉시 발행한다. `hd_map_consumer_example.py`의 `~layers`와
`SemanticMapIndex`로 필요한 vector/미션 레이어만 받고 제한속도·구간·신호등/정지선을 조회한다.
링크별 유효 최대속도 39개는 속도 zone payload로 전달한다. 상세 설정과 좌표 변환은
[runtime 계약](../../docs/contracts/hd-map/runtime-v1.md#22-선택-구독과-규칙-조회)을 따른다.

수치만으로 artifact를 동일하다고 판단하지 않는다. `/hd_map/meta`의 `build_id`, `map_id`,
`geometry_sha256`와 `manifest_sha256`를 함께 확인한다.

## 권위와 데이터 흐름

```text
공식 path/checkpoint + 현재 Node/Link + stage14 reviewed markings
                         │
                         ├─ competition metadata builder
                         ├─ drivable input audit
                         └─ compact drivable builder
                                      │
                                      v
       committed YAML/JSON/NPZ bundle ──> runtime publisher ──> /hd_map/*
```

| 자료 | 역할 | 운영 runtime에서의 취급 |
|---|---|---|
| 현재 K-City Node/Link | topology, link ID와 진행 관계 | 정본 입력 |
| 공식 global path/checkpoint | route station과 통과 기준 | 정본 입력 |
| route curation stage14 | 최종 물리 차선 121·정지선 7 | 검토 완료 입력 |
| legacy origin-aligned map | 구간별 geometry/paint provenance | builder가 승인한 부분만 반영 |
| 원태 H5/vector | 관측과 포장면 provenance | 승인된 patch·검수에 한정 |
| source-comparison topic | 사람 검수 | 운영 토픽과 절대 혼합하지 않음 |

전역경로 중심선은 route 기준이지 차선 경계 GT가 아니다. legacy ID도 현재 link의 foreign key로
직접 사용하지 않는다. 모든 이식 geometry는 현재 topology 귀속과 source SHA를 유지한다.

## 읽기 전용 artifact 검사

저장소 루트에서 `$ASMC_DATA`를 설정한 뒤 실행한다.

```bash
python3 src/asmc_hd_map/scripts/rebuild_hd_map_artifacts.py --check   # 아래 전부와 pin 일치

python3 src/asmc_hd_map/scripts/build_competition_metadata.py --check
python3 src/asmc_hd_map/scripts/build_runtime_pins.py --check

PYTHONPATH=src/asmc_hd_map/src \
  python3 src/asmc_hd_map/scripts/build_drivable_area_input_audit.py --check

PYTHONPATH=src/asmc_hd_map/src \
  python3 src/asmc_hd_map/scripts/build_compact_drivable_map.py \
  --repo-root "$PWD" --check
```

| policy/source | committed result |
|---|---|
| `kcity2025_competition_metadata_source.yaml` | `kcity2025_competition_metadata.yaml` |
| reviewed map + committed metadata | `config/baselines/hd_map_runtime_pins_v1.yaml` |
| `kcity2025_drivable_area_audit_source.yaml` | `kcity2025_drivable_area_input_audit.yaml` |
| `kcity2025_compact_drivable_source.yaml` | `kcity2025_compact_drivable_map.json`, `...grid.npz` |

generated YAML/JSON/NPZ를 손으로 고치지 않는다. 입력과 source hash를 변경한 뒤 builder로
재생성하고 geometry/count/hash, RViz와 consumer 회귀를 함께 리뷰한다.

`--check`는 파일을 고치지 않고 입력 hash와 재생성 결과가 committed artifact와 같은지 검사한다.
입력을 의도적으로 변경할 때만 builder의 write 경로를 실행하고 source·output·사람 검수 결과를
같은 PR에 포함한다.

## 구조

```text
src/asmc_hd_map/
├── config/                    source policy와 committed runtime bundle
├── launch/                    운영·debug·source comparison 진입점
├── rviz/                      통합 및 개발 검수 preset
├── scripts/                   builder, publisher, verifier
├── src/asmc_hd_map/           ROS 비의존 geometry·metadata core
├── test/                      deterministic·topology·runtime 회귀
├── reports/                   후보 생성 당시 수치와 provenance artifact
├── development/won_tae_link_node/
│                               이전 H5/bake 도구의 read-only provenance
└── schemas/                   candidate map schema
```

`reports/`의 날짜별 후보는 운영 입력 목록이 아니다. 현재 runtime은 committed metadata,
stage14 reviewed geometry와 compact product만 읽으며 시작 전에 SHA를 검사한다.

## Local pose와 현재 차로

기본 local pose는 `/gps`+`/imu`를 WGS84→UTM52N으로 변환한 뒤 맵 원점
`(302595, 4124145) m`과 GPS 전방 lever arm 0.35 m를 적용한다. `/Ego_topic.position=(0,0)`을
map XY로 사용하지 않는다. 정규화 pose가 있으면 다음처럼 지정한다.

```bash
roslaunch asmc_hd_map competition_map.launch \
  local_pose_source:=pose_stamped \
  local_pose_topic:=/localization/pose
```

local geometry도 `base_link` 좌표가 아니라 global ID를 유지하는 `map` frame subset이다.
현재 차로 기준점은 후륜축 중심이다. 공유 경계에서는 이전 차로를 유지하고 인접 polygon 안쪽
0.30 m pose가 2회 연속 들어온 뒤 전환한다. 최대 2 m의 작은 이탈은 `NEARBY`, 0.5초 동안
pose가 없으면 `STALE/valid=false`다.

```bash
rostopic echo /hd_map/local/current_lane \
  --filter 'm.valid' | grep -E 'state:|link_id:|lane_number:|signed_lateral|polygon_clearance|transition_'
```

paint gap은 `LaneMarking`에 그대로 남아도 `Lane` polygon과 `DrivableArea` union은 연속 주행면을
전달한다. 판제가 polyline 점 간격으로 solid/broken 또는 주행가능성을 추정해서는 안 된다.

## 개발용 source 비교

최종 차선, origin-aligned legacy, 원태 H5 vector, 최신 compact 면과 과거 H5 bake를 독립
레이어로 비교한다.

```bash
roslaunch asmc_hd_map hd_map_source_comparison.launch repo_root:=/root/ws
```

| 레이어 | 토픽 |
|---|---|
| 최종 차선·정지선 | `/asmc_hd_map/development/hd_map_sources/lanes/final_curated` |
| legacy origin-aligned | `/asmc_hd_map/development/hd_map_sources/lanes/legacy_origin_aligned` |
| 원태 H5 vector | `/asmc_hd_map/development/hd_map_sources/lanes/won_tae_h5` |
| 최신 compact 면 | `/asmc_hd_map/development/hd_map_sources/drivable/latest` |
| 과거 H5 road bake | `/asmc_hd_map/development/hd_map_sources/drivable/legacy_h5_bake` |

기본 pose source는 `/gps`+`/imu`다. `pose_source:=ego_status`는 Ego message에 실제 map XY를 넣는
별도 bridge가 있을 때만 사용한다. 개발 레이어는 review only이고 `/hd_map/global/*` 산출물에
자동 합성되지 않는다.

## 변경 절차와 회귀 gate

미션 구간·속도·체크포인트·주행가능영역은 판제 연동 결과에 따라 계속 바뀔 수 있다. `team_final`도 팀 현재
기준값일 뿐 수정 금지 상태가 아니다. source를 고친 뒤 생성물 연쇄는 한 번에 다시 만든다.

```bash
python3 src/asmc_hd_map/scripts/rebuild_hd_map_artifacts.py          # 다시 생성
python3 src/asmc_hd_map/scripts/rebuild_hd_map_artifacts.py --check  # stale 단계만 보고
```

1. 바꾸려는 대상이 물리 도색, topology, metadata 또는 drivable surface 중 무엇인지 구분한다.
2. source policy와 provenance를 수정하고 generated output 직접 편집은 피한다.
3. builder `--check`, 관련 unit test와 feature count/hash 차이를 검토한다.
4. `competition_map_debug.launch`에서 전체 route, checkpoint, mission과 면 연결을 확인한다.
5. 실제 ROS topic verifier와 영향을 받는 planner consumer를 검증한다.
6. source comparison은 근거로만 쓰고 선택 결과만 운영 bundle로 승격한다.

특히 다음은 함께 확인한다.

- checkpoint 15개와 route 순서·3 m 포함
- speed profile 전 구간 coverage, `route_speed_limits`의 min(MGeo, 공식 상한), provisional enforcement 금지와 `team_final` authority 태그
- Lane/DrivableArea 연결성, hole·seam과 반대방향/미사용 차로 제외
- `build_id`, source SHA, geometry hash와 feature count
- global/local cell parity와 `LaneAssociation` hysteresis

ROS 비의존 회귀는 다음과 같이 실행한다.

```bash
PYTHONPATH=src/asmc_hd_map/src \
  python3 -m unittest discover -s src/asmc_hd_map/test -p 'test_*.py'
```

ROS 환경에서는 `catkin_make --pkg asmc_msgs asmc_hd_map`과 실제 topic verifier를 추가한다.

## 과거 후보와 근거 위치

| 과거 README 절 | 보존 위치 |
|---|---|
| legacy alignment 첫 audit | `reports/kcity2025_legacy_alignment_20260916/` |
| current-only corridor | `reports/kcity2025_current_corridor_20260916/` |
| 원태 관측 calibration | `reports/kcity2025_observed_calibration_20260916/` |
| 3-source candidate review | `reports/kcity2025_candidate_review_20260916/` |
| legacy adoption | `reports/kcity2025_legacy_adoption_20260916/` |
| hybrid completion | `reports/kcity2025_hybrid_completion_20260916/` |
| operational candidate | `reports/kcity2025_operational_map_20260916/`과 config/source SHA |
| route curation 1~14 | [route curation 기록](../../docs/records/hd-map/kcity2025-route-curation-20260917.md) |
| P2-A input audit | [P2-A baseline](../../docs/records/baselines/kcity2025-drivable-input-v1.md) |
| P2-B compact product | [P2-B baseline](../../docs/records/baselines/kcity2025-compact-drivable-v1.md) |

원태 `development/won_tae_link_node`는 현재 제품 재생성에 필요한 provenance를 외부 immutable
artifact로 보존하고 parity를 확인하기 전까지 삭제하지 않는다.
