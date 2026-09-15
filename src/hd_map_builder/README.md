# hd_map_builder

현재 K-City 2025 `link/node`를 topology 정본으로 유지하면서, 구 KATRI의
`lane_boundary`, `road_mesh`, 노면표시·신호·표지 geometry를 **구간별 후보**로 정합하는
패키지다. 정합·audit core는 ROS와 독립적이고, 주행 검수용 ROS visualizer만 선택적으로
제공한다. 기존 `src/link_node`와 그 H5/PNG는 읽기만 하며 수정하지 않는다.

주최측 제공 대회 전역경로
`R_KR_PG_KATRI/2026_molit_comp_global_path.txt`를 주행 corridor와 누적 거리 `s`의 기준으로
사용한다. 전역경로는 한 개의 주행 중심선이므로 차선 경계·차로폭 GT로 간주하지 않는다.
인접 차로는 경로 좌우의 현재 link를 별도로 탐색하고, lane geometry는 관측 자료와 다시
검증한다.

## 왜 MGeo JSON을 그대로 새 정본으로 쓰지 않는가

구 MGeo의 `points`, `lane_color`, `lane_shape`, `lane_type`, 좌우 lane-mark 관계는 유용한
입력 형식이다. 반면 구/신 맵의 `idx` 집합은 다르고 도로 확장·분할도 존재하므로 구 ID를
현재 topology의 foreign key로 쓰면 안 된다.

새 정본 후보는 MGeo 속성을 `attributes.mgeo` 아래 보존하면서 다음을 추가한다.

- 현재 `map` frame과 origin
- 현재 link에 대한 many-to-many 대응
- source 파일 SHA-256과 원본 feature ID
- 거리·방향·구간 coverage 검증값
- `candidate/reviewed/operational/rejected` 상태

스키마는 `schemas/canonical-hd-map-v1.schema.json`에 있다. audit가 만드는
`canonical_candidate_example.json`은 구조 예시일 뿐 운영 맵이 아니다.

## 구조

```text
hd_map_builder/
├── config/
│   ├── kcity2025_competition_corridor.yaml
│   └── kcity2025_legacy_transfer.yaml
├── schemas/canonical-hd-map-v1.schema.json
├── scripts/build_current_corridor.py
├── scripts/audit_legacy_alignment.py
├── scripts/legacy_origin_overlay_node.py
├── launch/legacy_origin_overlay.launch
├── rviz/legacy_origin_overlay.rviz
├── src/hd_map_builder/
│   ├── frames.py              origin/CRS 계약
│   ├── geometry.py            polyline sampling/offset
│   ├── spatial.py             결정론적 spatial index
│   ├── correspondence.py      ID 비의존 geometry 대응
│   ├── io_mgeo.py             MGeo read-only adapter와 hash
│   ├── schema.py              canonical feature 계약
│   ├── map_overlay.py         origin-only local review geometry
│   ├── corridor.py            current route/corridor/topology
│   ├── nominal.py             명목 경계·정지선 후보
│   ├── current_builder.py     current-only canonical 후보 생성
│   └── audit.py               route/lane/road/부가 layer audit
├── reports/                   작은 정량 결과와 후보 index
└── test/
```

## 실행

저장소 루트에서 실행한다.

현재 Node/Link만 사용하는 1단계 명목 기준선:

```bash
python3 src/hd_map_builder/scripts/build_current_corridor.py
```

출력은 `reports/kcity2025_current_corridor_20260916/`에 생성된다. 이 빌드는 설정의
source key를 current global/link/node, route link와 주최측 전역경로로 제한한다.
관측선·구맵 경로가 추가되면 시작을 실패하므로 명목 기준선 provenance가 섞이지 않는다.

구맵 전이 가능성 audit:

```bash
python3 src/hd_map_builder/scripts/audit_legacy_alignment.py
```

기본 출력:

```text
src/hd_map_builder/reports/kcity2025_legacy_alignment_20260916/
├── summary.json
├── route_intervals.csv
├── link_correspondence.csv
├── lane_transfer_candidates.csv
├── observed_lane_alignment.csv
├── auxiliary_transfer_candidates.csv
├── high_speed_candidate_index.json
└── canonical_candidate_example.json
```

로컬 `road_mesh_out_line.json`이 없으면 나머지 audit는 계속 수행하고 road mesh 항목만
`available=false`로 기록한다. source 경로와 실제 hash가 모두 report에 남으므로
`global_info.mgeo_file_hash` 문자열을 입력 정체성으로 신뢰하지 않는다.

## 승격 원칙

1. 현재 link/node가 topology·route·속도 제한의 정본이다.
2. 주최측 전역경로는 route station과 주행 corridor의 정본이며 lane GT는 아니다.
3. 구맵은 선언된 origin translation만 적용하고 전역 ICP 회전·scale을 맞추지 않는다.
4. road, lane, stopline, traffic asset은 서로 다른 검증 gate를 사용한다.
5. 구 feature는 `candidate`로 시작하고 현재 Semantic/LiDAR/RGB 검증 없이는
   `operational`이 될 수 없다.
6. 기존 `link_node_map.h5`에는 검토가 끝나기 전 어떤 후보도 합치지 않는다.
7. 파란 차선은 보존 자료에는 남길 수 있지만 현재 대회 운영 lane layer에서는 제외한다.

첫 승격 후보는 구/신 geometry와 현재 Semantic LiDAR가 함께 맞는 고주로다. 음영구간과
차로 확장·교차로 연결부는 현재 관측을 우선한다.

## 2026-09-16 첫 audit 결과

`kcity2025_legacy_transfer.yaml`의 고정 기준으로 실행한 결과는
`reports/kcity2025_legacy_alignment_20260916/README.md`에 요약했다.

- 선언 origin만 적용한 변환: `(-135.058, -1509.463, +28.991)m`
- 대회 경로 2,184.6 m 중 link geometry 재사용 후보: 89.70%
- origin 변환 후 안정 경로 XY 잔차 중앙값: `(-0.039, -0.059)m`, norm p95 `0.575m`
- legacy lane 전체 A/B/D: `449/115/1015`
- 고주로 15 m corridor lane 후보: 64개(A 56, B 8; white 49, yellow 15)
- 고주로 legacy road mesh와 current link-width proxy IoU: 0.942

이 수치는 candidate 선별값이다. 특히 lane의 비교 대상은 실제 GT가 아니라 현재
`width=3.5m` link proxy이므로, 승격에는 현재 Semantic/LiDAR/RGB overlay 검수가 추가로
필요하다.

## 2026-09-16 current-only corridor 기준선

`kcity2025_competition_corridor.yaml`로 생성한 결과는
`reports/kcity2025_current_corridor_20260916/README.md`에 정리했다.

- 공식 route link 39개와 주최측 전역경로가 2,184.612m 전 구간에서 정확히 일치
- route·고주로·음영·정적 장애물 context를 합친 current link 70개
- 고주로 평행 link 32개를 실제 방향 기준으로 같은 방향 30개, 반대 방향 2개로 구분
- 명목 lane boundary 85개와 route stop-line node 후보 13개 생성
- 고주로만 포함되는 link geometry는 해당 corridor 구간으로 clip하되 source link ID와
  topology는 보존

모든 경계 폭은 현재 link의 일률적인 `3.5m`에서 나온 후보이며 실제 도색 GT가 아니다.
교차로·짧은 link는 자동 경계 생성을 건너뛰었고, 합류·분기 후보는
`review_required`로 남겼다. 관측선·구맵 보정 전까지 map과 모든 feature는
`operational=false`, `candidate` 상태다.

## 구맵 origin-only 주행 검수

UDP bridge가 `/Ego_topic`을 발행하는 상태에서 다음을 실행한다.

```bash
roslaunch hd_map_builder legacy_origin_overlay.launch
```

별도 `map → base_link` TF는 필요 없다. 노드는 Ego 위치·heading으로 주변 60m geometry를
직접 `base_link`로 변환한다. 구→신 맵에는 선언된 origin translation만 적용하며 fitted
yaw·scale·ICP는 적용하지 않는다.

| RViz 토픽 | 의미 |
|---|---|
| `/hd_map_builder/debug/legacy_lanes` | 구 MGeo lane. white/yellow/blue 원색 표시 |
| `/hd_map_builder/debug/legacy_link_centers` | 구 link 중심선. 기본 비활성 |
| `/hd_map_builder/debug/current_link_centers` | 현재 link 중심선. 차선 경계가 아님 |
| `/hd_map_builder/debug/competition_route` | 주최측 제공 전역경로 |
| `/hd_map_builder/debug/ego_context` | 후륜축 기준 차량 footprint와 전방 방향 |

구 link까지 비교하려면 다음처럼 켠다.

```bash
roslaunch hd_map_builder legacy_origin_overlay.launch publish_legacy_links:=true
```

소스 워크스페이스가 아닌 설치 공간에서 실행한다면 데이터가 있는 저장소 루트를
`repo_root:=/root/ws`처럼 명시한다.

이 화면은 위치·방향·구간 변경을 빠르게 검수하기 위한 것이며 실제 노면 RGB가 아니다.
물리 도색과의 픽셀 단위 검수는 후보 구간을 고른 뒤 RGB-BEV overlay에서 별도로 수행한다.
