# asmc_hd_map

AutoNav K-City 2025 HD맵의 deterministic builder, 운영 publisher, local cache와 개발 검수
도구를 소유하는 ROS1 package다. Phase 2 통합이 완료됐으며 운영 제품은
`kcity2025_roundabout_extended_drivable_v5`다.

- 개발·재생성·검증: [package guide](../../docs/packages/asmc_hd_map/README.md)
- topic·message 의미: [HD Map contract](../../docs/contracts/hd-map/topic-v1.md)
- 판제 소비와 RViz: [runtime contract](../../docs/contracts/hd-map/runtime-v1.md)
- checkpoint·mission: [competition metadata](../../docs/contracts/competition/kcity2025-metadata-v1.md)
- 최종 주행가능영역 기준: [P2-B baseline](../../docs/records/baselines/kcity2025-compact-drivable-v1.md)

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

최종/legacy/원태 source와 최신/과거 주행가능영역 비교:

```bash
roslaunch asmc_hd_map hd_map_source_comparison.launch repo_root:=/root/ws
```

source comparison은 review only다. `/asmc_hd_map/development/*`를 운영
`/hd_map/global/*`과 합치거나 판제 입력으로 사용하지 않는다.

## 현재 제품

| 항목 | 개수·기준 |
|---|---|
| 물리 차선·정지선 | 121 + 7 = 128 `LaneMarking` |
| 차로 | 70 `Lane` |
| vector 주행가능영역 | 102 `DrivableArea` |
| 체크포인트 | 16, 통과 반경 3 m |
| mission/policy/speed zone | 15 |
| raster | 0.2 m/cell, vector product의 파생 lookup |

판제는 latched global vector/topic을 정본으로 사용한다. local lane/grid는 map frame과 global ID를
유지하는 선택적 cache이며 planner horizon이나 별도 정본이 아니다. 현재 차로 association은
GPS 안테나가 아니라 후륜축 중심 `base_link`의 map 좌표를 기준으로 한다.

## 읽기 전용 artifact 검사

저장소 루트에서 `$ASMC_DATA`를 설정한 뒤 실행한다.

```bash
python3 src/asmc_hd_map/scripts/build_competition_metadata.py --check
PYTHONPATH=src/asmc_hd_map/src \
  python3 src/asmc_hd_map/scripts/build_drivable_area_input_audit.py --check
PYTHONPATH=src/asmc_hd_map/src \
  python3 src/asmc_hd_map/scripts/build_compact_drivable_map.py \
  --repo-root "$PWD" --check
```

generated YAML/JSON/NPZ를 손으로 고치지 않는다. 입력과 source hash를 변경한 뒤 builder로
재생성하고 geometry/count/hash, RViz와 consumer 회귀를 함께 리뷰한다.

## 구조

```text
config/                    source policy와 committed runtime bundle
launch/, rviz/             운영·debug·source comparison
scripts/                   builder, publisher, verifier
src/asmc_hd_map/           ROS 비의존 geometry·metadata core
test/                      deterministic·topology·runtime 회귀
reports/                   날짜별 후보와 정량 audit 기록
development/won_tae_link_node/
                           이전 H5/bake 도구의 read-only provenance
schemas/                   candidate schema
```

과거 candidate 수치와 14단계 route curation의 행선지는 package guide의
[과거 후보와 근거 위치](../../docs/packages/asmc_hd_map/README.md#9-과거-후보와-근거-위치)에
정리돼 있다. `reports/`의 candidate를 현재 runtime 입력으로 해석하지 않는다.
