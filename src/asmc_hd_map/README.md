# asmc_hd_map

AutoNav의 K-City 2025 HD맵 런타임, 결정적 생성기, 회귀 검사와 개발용 source 비교 도구를
한곳에 둔 패키지다. 판제는 `/hd_map/global/*`의 latched 정본 토픽만 소비한다. 개발용
overlay는 `/asmc_hd_map/development/*`로 완전히 분리되며 운영 토픽을 쓰거나 수정하지 않는다.

현재 K-City 2025 `link/node`를 topology 정본으로 유지하면서, 구 KATRI의
`lane_boundary`, `road_mesh`, 노면표시·신호·표지 geometry를 **구간별 후보**로 정합한다.
정합·audit core는 ROS와 독립적이다. 원태의 이전 `link_node` 도구와 H5/PNG는
`development/won_tae_link_node/`에 provenance 자료로 통합했으며 개발 시 읽기만 한다.

## 운영과 개발 진입점

운영 publisher는 원태 H5나 구맵 파일을 런타임에 다시 계산하지 않고, 동결된 최종 차선과
compact 주행가능영역만 읽는다.

```bash
roslaunch asmc_hd_map competition_map.launch
rosrun asmc_hd_map hd_map_consumer_example.py
rosrun asmc_hd_map verify_competition_map_topics.py
```

운영 launch는 global 정본과 함께 `/hd_map/local/lanes`,
`/hd_map/local/drivable_grid`를 기본 활성화한다. local crop은 map frame·global ID를 유지하며
기본 120 m 정사각형, 0.2 m/600×600 grid, 최대 5 Hz다. 정규화 pose를 사용할 때는
`local_pose_source:=pose_stamped local_pose_topic:=/localization/pose`를 지정한다.
기본 위치원은 `/gps`와 `/imu`이며 WGS84→UTM52N 변환 후 맵 원점
`(302595, 4124145)m`과 GPS 전방 lever arm 0.35 m를 적용한다. 현재 대회판의
`/Ego_topic`은 `frame_id=base_link`, `position=(0,0)`으로 들어오므로 map XY로 쓰지 않는다.
센서의 `eastOffset/northOffset`이 맵 원점과 1 m 넘게 다르면 local crop 발행을 거부한다.

판제의 권장 방식은 69개 Lane, 97개 DrivableArea, 128개 LaneMarking과 15개 MissionZone을
latched global 토픽에서 시작 시 한 번 받고, `map` 좌표의 Ego 위치로 자체 공간 조회하는
것이다. local lane/grid는 선택적 lookup·시각화 캐시이며 planner horizon이나 별도의 정본이
아니다. 따라서 local 120 m 정사각형이 곡선 경로 전후방과 정확히 맞지 않아도 global 기반
판제에는 정보 손실이 없다. 미션·체크포인트·차선 도색은 local로 중복 발행하지 않으며 현재
link 또는 route station을 global metadata와 대조한다. 자세한 소비 우선순위와 의도적으로 빈
필드는 [runtime contract](../../docs/contracts/hd-map-runtime-visualization-v1.md)를 따른다.

최종 차선·구 맵 origin 정합 차선·원태 H5 vector를 각각 켜서 비교하는 개발 화면은 다음과
같다. 세 레이어는 서로 다른 토픽이며 자동 혼합되지 않는다.

```bash
roslaunch asmc_hd_map lane_source_comparison.launch repo_root:=/root/ws
```

이 비교 launch도 기본적으로 `/gps`+`/imu`를 사용한다. `pose_source:=ego_status`는
`EgoVehicleStatus.position`에 실제 map XY가 들어오는 별도 브리지에서만 허용하며, `(0,0)`이나
competition corridor에서 20 m 넘게 벗어난 좌표는 잘못된 origin 진단을 막기 위해 거부한다.

| 개발 레이어 | 토픽 |
|---|---|
| 최종 검토 차선 121개와 정지선 7개 | `/asmc_hd_map/development/lane_sources/final_curated` |
| 구 맵 origin 정합 corridor 차선 | `/asmc_hd_map/development/lane_sources/legacy_origin_aligned` |
| 원태 H5에 저장된 원본 vector | `/asmc_hd_map/development/lane_sources/won_tae_h5` |

RViz에서는 기본적으로 최종 차선만 켜며, Displays에서 나머지 두 레이어를 독립적으로
활성화할 수 있다. 이 launch는 개발 전용이며 판제 배포 launch에 포함하지 않는다.

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
asmc_hd_map/
├── config/                    생성·검수·runtime·개발 비교 설정
├── launch/
│   ├── competition_map.launch             판제용 운영 진입점
│   ├── competition_map_debug.launch       통합 RViz 검수
│   └── lane_source_comparison.launch      3종 원천 개발 비교
├── rviz/
│   ├── competition_map_debug.rviz
│   └── lane_source_comparison.rviz
├── scripts/                   결정적 builder, publisher, verifier
├── src/asmc_hd_map/           ROS 비의존 geometry·metadata core
├── reports/                   동결 산출물과 정량 audit
├── development/
│   └── won_tae_link_node/     원태 H5·도구 보존본(운영 제외)
├── schemas/                   canonical map schema
└── test/                      geometry·runtime·source 분리 회귀
```

## 실행

저장소 루트에서 실행한다.

현재 Node/Link만 사용하는 1단계 명목 기준선:

```bash
python3 src/asmc_hd_map/scripts/build_current_corridor.py
```

출력은 `reports/kcity2025_current_corridor_20260916/`에 생성된다. 이 빌드는 설정의
source key를 current global/link/node, route link와 주최측 전역경로로 제한한다.
관측선·구맵 경로가 추가되면 시작을 실패하므로 명목 기준선 provenance가 섞이지 않는다.

원태 관측선으로 경계 후보를 보정하는 2단계:

```bash
python3 src/asmc_hd_map/scripts/calibrate_observed_lanes.py
```

2단계 설정은 1단계 `nominal_candidate_map.json`의 SHA-256을 고정 입력으로 검사한다.
19개 CSV의 직접 관측만 fit support로 쓰며 `generated`, `interpolated`, manual bridge와
기존 raster용 `inward_offset_m=0.03`은 보정 geometry에 적용하지 않는다. CSV의
`LEFT/RIGHT`는 수집 차량 기준 audit 값일 뿐 canonical 좌우 역할로 복사하지 않는다.
출력은 `reports/kcity2025_observed_calibration_20260916/`에 생성된다.

구맵 전이 가능성 audit:

```bash
python3 src/asmc_hd_map/scripts/audit_legacy_alignment.py
```

기본 출력:

```text
src/asmc_hd_map/reports/kcity2025_legacy_alignment_20260916/
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

## 2026-09-16 원태 관측선 기반 경계 보정 후보

`kcity2025_observed_calibration.yaml`의 고정 계약으로 19개 CSV 26,511행을 감사했다.

- 직접 관측 25,414행만 사용: Semantic LiDAR 13,679행, GPS Direct 11,735행
- 생성 958행과 시간 보간 139행은 audit-only
- 직접 관측 60개 segment를 exact polyline projection한 뒤 current topology 안에서만
  경계 전환을 허용
- 자동 gate를 통과한 match run 76개, 검토 필요 74개, 기각 47개, 미대응 15개
- source conflict가 없는 3,734개 1m profile bin을 적용
- 85개 parent 경계 중 50개에 60개 `observed_calibrated` interval 생성
- 양쪽 경계가 모두 통과한 구간의 폭 sample 3,143개: 중앙값 3.468m,
  p05/p95 3.276/3.803m

보정 interval 밖은 원래 `link_nominal` geometry로 남고, 보정/명목 station interval은
서로 길이 중복 없이 parent 전체를 구성한다. 합류 taper에서 폭이 좁아지는 후보도 있으므로
폭 profile은 관측 감사값이며 current link의 `width_start/end=3.5m`를 수정하지 않는다.
정량 결과와 review overlay는
[`observed calibration report`](reports/kcity2025_observed_calibration_20260916/README.md)에
있다. 이 map도 여전히 `candidate`, `operational=false`다.

## 2026-09-16 3-source 검수와 고주로 차로 보존

현재 centerline·명목 경계, 원태 관측 보정 경계, origin-only 구맵 A/B 후보를 한 화면에
그리는 review bundle을 추가했다.

```bash
source devel/setup.bash
python3 src/asmc_hd_map/scripts/build_candidate_review.py
```

고주로 current centerline 32개(동일 방향 30, 반대 방향 context 2)를 모두 유지한다. 공식
경로에서 좌우 인접 관계로 도달 가능한 25개 차선변경 link도 모두 포함돼 있다. 그중
21개에는 원태 관측 보정 interval이 있고, 4개는 아직 명목 경계다. 고주로 구맵 A/B
후보는 64개(A 56, B 8)다.

원태 보정선과 동일 current link의 구맵 경계를 비교한 대응 sample 거리 p50/p95는
0.105/0.515m였다. 따라서 변경되지 않은 고주로의 Tier A legacy geometry·색상·선 형태는
우선 이식 후보로 적합하다. 다만 현재 observed candidate map에는 아직 구맵을 합치지
않았으며 current node/link가 topology 정본이다. 상세 수치·구간별 PNG는
[`candidate review report`](reports/kcity2025_candidate_review_20260916/README.md)에 있다.

주행 중 세 source를 RViz에서 비교하려면 다음을 실행한다.

```bash
roslaunch asmc_hd_map candidate_map_overlay.launch repo_root:=/root/ws
```

기본 화면은 current/nominal/observed를 표시하고 구맵 A/B, 원태 raw observation과 고주로
link label은 RViz에서 선택적으로 켠다. pose는 대회/26.R1 공통 `/gps`·`/imu`를 사용하고
candidate map UTM origin과 GPS lever arm을 적용한다. 대회판에서 위치가 0으로 들어온
`/Ego_topic`은 기본 pose source로 쓰지 않는다. 이 launch의 모든 출력은 review-only다.
RViz 격자 원점과 흰 점은 후륜축 중앙 `base_link`이며 빨간 화살표는 차량 전방 `+x`,
초록 화살표는 차량 좌측 `+y`다. 차체 박스 중심은 차량 형상에 맞춰 원점보다 1.5275m
앞에 있으므로, 박스 중심이 격자 원점과 일치하지 않는 것이 정상이다.

### 세 정본 후보를 섞지 않는 주행 비교

구간별 source 선택을 다시 하기 위한 전용 화면은 다음 launch를 사용한다.

```bash
roslaunch asmc_hd_map three_source_map_compare.launch repo_root:=/root/ws
```

이 launch는 operational map과 이후 수동 수정 결과를 입력으로 읽지 않는다. nominal/observed
문서는 공통 좌표 frame과 current 70-link corridor allowlist를 만드는 데만 사용하며 geometry를
표시하거나 세 source에 합치지 않는다. 다음 세 source와 차량 footprint만 별도 토픽으로
발행한다.

| RViz display/topic | 정확한 입력 |
|---|---|
| `1 Pre-plan completed map` / `/asmc_hd_map/compare/preplan_hybrid` | 원태 관측 기준 정본화 계획 직전의 `hybrid_candidate_map.json` |
| `2 Won-tae pushed H5` / `/asmc_hd_map/compare/won_tae_bake` | 커밋 `e67f5da`의 `link_node_map.h5`에 저장된 관측 차선 vector 66개와 정지선 7개 |
| `3 Legacy origin-aligned corridor` / `/asmc_hd_map/compare/legacy_origin_aligned` | 구 KATRI 원본 차선을 origin translation만 적용한 A/B 후보 |

기본값은 1번만 켠다. RViz Displays에서 한 번에 하나씩 켜 같은 위치를 통과하며 비교한다.
세 source를 동시에 켜도 데이터가 합쳐지지는 않지만 겹친 색 때문에 판단하기 어려우므로
구간 선택 검수에서는 권장하지 않는다. 각 source 안에서는 실제 속성을 확인할 수 있도록
white/yellow/blue를 원색으로, 정지선은 빨간색으로 표시한다. 색상 정보가 없는 선만 source별
보조색(cyan/orange/magenta)을 사용한다.

3번 구맵은 전체 구 KATRI를 표시하지 않는다. 기존 정합 audit에서 current 대회 corridor
70개 link 중 하나 이상에 대응한 Tier A/B 127개만 남긴다. 이에 따라 이전에 검수한
고주로 Tier A/B 차선 64개를 모두, 공식 전역경로 관련 경계와 current route connector에
대응된 교차로 유도선 후보를 유지하고, 대회 주행과 무관한 도로는 제외한다. geometry에는
fitted yaw/scale/ICP나
원태 관측 offset을 적용하지 않고 선언된 origin translation만 적용한다. 현재 선택 집합은
`A=111/B=16`, lane type별로 `501=43, 503=25, 505=41, 506=10, 525=3, 531=5`다.

2번 H5 화면은 H5의 최종 `lane` raster를 다시 vectorize한 결과가 아니다. 저장된
`observed_lane_points/offsets`와 `stopline_*` vector를 그대로 사용하므로 재윤곽화에 따른
계단·분기 artifact가 없다. H5에는 paint pattern 정보가 없어서 차선은 모두 연속선으로
보이지만 위치는 원태가 push한 vector 그대로다.

이 비교에서 구간별 source가 확정되기 전에는 세 자료를 자동 blend하거나 smoothing하지
않는다. 기존 operational candidate와 관련 report는 provenance 감사용으로 보존하지만 이
launch의 입력이나 표시에는 포함되지 않는다.

### 2026-09-18 route curation 1~14단계

세 source 비교 결과를 한 번에 전체 맵으로 합치지 않고 link 구간별로 확정하는 partial
candidate를 추가했다. 전체 결정·스크린샷 evidence는
[`HD맵 경로별 선택 검수 문서`](../../docs/hd-map-route-curation-review-20260917.md)에 있다.

현재 구현 범위는 출발점부터 `A2256W000751` finish까지다. 1단계는 origin-only legacy geometry를
사용하고 `A2256W000748`의 route-right 도색을 solid로 교정했다. `A2256W000329` 끝의 짧은
흰선은 횡 offset을 유지한 채 노란선과 같은 route 단면까지 `0.618m` 연장했다. `A182`의
곡률이 틀린 legacy 점선은 current link 곡률로 교체하되 legacy 시작·끝 오프셋과 종료점을
유지했다. 2단계
`219→215→213`은 원태 H5 vector 5/6/21/22/23만 사용한다. vector 6 끝을 `0.220m`
연장했고, `A2256W000213` 노란선의 `2.278m` 실제 도색 공백은 보존했다. 이 링크 끝에는
원태 H5 `A1256W000682/807` 정지선만 넣었다. 3단계 `222→220`은 교차로의 거짓 연속
실선을 제거하고 white broken guide 하나와 H5 vector 7/8만 사용한다. 4단계
`202→728→599→207`은 H5 vector 7/8과 10/17, 정지선 208을 사용한다. A202 구간의 vector 8은
`A202` 오른쪽=`A009` 왼쪽 white broken 공유선으로 교정하고, 구맵 `B219BS010398`의 A009
우측 white solid를 A728 분기 전까지만 추가했다. 정지선 뒤 교차로의 비관측 구간은 비워
두었다. A207의 짧은 route-right 흰선은 관측 station 범위를 유지하면서
최대 0.20m gate의 median route-normal offset으로만 직선화했다. 5단계 `205→315`은 H5
vector 10~15의 중앙선/U턴 조각과 vector 18의 2차로 외곽선을 사용하고, 누락된 1차로 우측
white broken divider만 legacy `B219BS010382`에서 보완했다. source yellow였던 U턴 vector
11~14는 실제 white broken으로 교정하고 vector 10과 평행하게 정렬했다. divider는
`C1256W000123` 아래 원태 H5 정지선 두 선분까지 연장하되 중앙선 관측 공백은 연결하지
않았다. A207/A205 경계의 같은 solid 사이 `0.351m/0.152m` 수집 단절만 연결하고 이어지는
broken paint에는 bridge하지 않았다. 6단계 `A2256W000318`은 route를 가로지르는 legacy curve를 기각하고 current A318
중심선과 일정 오프셋을 유지하는 좌·우 white broken guide로 교체했다. 오른쪽 guide는
`A318` 오른쪽=`A319` 왼쪽 공유 경계로 한 번만 저장한다. `A319` 바깥 white solid는 명목
1.75m 대신 앞선 A310 H5 vector 18 종점에서 계산한 1.465m 오프셋으로 연결했다. 7단계
`A2256W000308`은
원태 H5 vector 19/20의 yellow left·white outer 경계와, 이 link에서만 위치가 맞는 legacy
`B219BS011450` white broken 내부 divider를 사용한다. 최신 주행 확인에 따라 이전 정지선
기각을 폐기하고 H5 결합 선분 `A1256W000593+A1256W000742`를 `A308/A309` 끝의 두 차로
정지선으로 추가했다. A319 외곽 실선은 A309 current link를 따라 H5 vector 20 시작까지
연결하고, 내부 점선은 앞선 guide의 dash phase를 이어 일정한 간격을 유지한다. 8단계
`A2256W000083`은 pre-plan current-route 곡률을 사용하되
yellow solid를 실제 white broken 전역경로 유도선으로 명시적으로 교정했다. 이 선은 A083
link 대비 오프셋 변화폭이 0.041m로 current link 곡률과 일치한다. 9단계 `A2256W000304`
확장 2차로는 원태 H5 vector 25/28을 양 외곽선으로 사용하고, 같은 station의 midpoint를
white broken 공유 divider로 생성했다. 옛 legacy 단일차로 실선은 사용하지 않으며, vector
25의 지글거리는 짧은 되짚기 run은 제거하되 원태 첫 yellow anchor는 보존했다. 10단계
`A2256W000148→146→151`은 원태 H5를
0.5m station-bin robust profile로 평탄화해 A148 폭 중앙값 `3.524m`를 유지한다. legacy의
과대 우측 폭은 사용하지 않는다. 로터리 원형 yellow vector 26은 route-normal 평균 대신
원 polyline 호장 기준으로 최대 `0.137m`만 평탄화하고 endpoint를 고정했다. 정지선은 원태
H5 `A1256W000709` 하나만 추가했다. 정지선 뒤 vector 25의 되짚기 꼬리는 제거하고,
맞은편 H5 white solid와
`3.434m` 간격을 유지하되 legacy `B219BS011472` 중앙선 종점에서 끝나는
`4.250m` yellow solid로 교체했다. 11단계 `A866→A054→A846`은
origin-only legacy corridor를 사용하되 `A057`에서 진입해 route를 가로지르는
`B219BS010215` guide와 비경로 `A150` fork 두 source를 제외한다. A866 시작의 left solid
수집 공백을 연결하고, A866 yellow는 white broken 시작 단면까지 연장한다. A846의 짧은
left `B011113`은 아래 `B011490` anchor 차로폭 `3.530m`로 right와 평행하게 재생성하고
노드까지만 연장한다. right는 A411 right solid과 `0.0054m`만 연결한다. 12단계 `A411→A420→A408→A445→A153`은
pre-plan white corridor를 기반으로 usable 1/2/3차로를 남기고 far-left bus-only yellow를
제거했다. 오지정했던 A411 constant-width solid은 롤백하고 원래 `B011097` broken을 복원했으며,
A445 양쪽은 원태 H5 관측으로 보정한 뒤 A153 right와 꼭지점에서 연결했다.
A418 lane-1 left white solid을 복원했고, A153 legacy broken은 right solid과 같은
current-link 곡률의 `+1.797m` offset broken으로 교체했다. 차로폭은 `3.507m`다.
A423 left는 white solid로 복원하고 A445 right solid에 겹치던 H5 broken trace는 제거했다.
또한 A179 초과 dash를 제거하고 A423 우측 점선을 노드까지 연장했다. 두 번째 톨게이트
섬을 관통하던 전체 재구성 solid는 제거하고, A432 시작 노드부에만 pre-plan `A424+A432`를
사용해 이전 white solid와 `0.024m`를 연결한 뒤 `4.140m` 접선 연속 구간으로 구맵
`B010446`에 혼합했다. 이후 legacy white solid은 섬 분기점까지 유지했다.
이후 `B010435/436→B010455/456`의 원본 공통 endpoint에 연결했다. A411 left는
white broken을 유지하며 A846 left와 `0.162m` 단차를 둔다. 출력 29개는 모두
white이며 `broken=12`, `solid=19`이다. 13단계
`A451→A446→A448→A126→A128→A333→A154`는 pre-plan의 route/usable-lane 경계
와 원태 H5, origin 정합 구맵을 구간별로 선별했다. A451 초반은 구맵의 톨게이트 섬 두 개와
route-right를 유지하고, 섬 합류점 뒤 left broken과 H5 right solid로 전환한다. A446 초반
왼쪽선은 H5, A446 중간부터 A448까지는 pre-plan을 사용한다. A451 left broken은 H5 right와
평행하게 재생성하고 A451 right 및 A448→음영구간 handoff는 접선 연속으로 보정했다. A427 topology는 제외하고,
A448 뒤 음영구간에는 A126 양 경계와 A131 외곽선을 포함했으며 A128 후반 source 단절도
연결했다. A333/A154 왼쪽은 pre-plan의 `B219BS011535` 점선 조각 3개를 모두 사용한다.
bus-only yellow와 2025에 없는 A333 뒤 `A1256W000616` 정지선은 제외했다.
14단계 `A332→A751`은 origin-only legacy 세 경계를 사용하고 finish 단면에 종점을 맞췄다.
전체 결과는 차선 121개, 정지선 7개이며 계속 `operational=false`다.
공식 route 밖 `A2256W000076`은 대응되는 direct H5 vector가 확인될 때까지 생성하지 않는다.

```bash
python3 src/asmc_hd_map/scripts/build_route_curation.py
roslaunch asmc_hd_map route_curation_review.launch repo_root:=/root/ws
```

산출물은
[`route curation report`](reports/kcity2025_route_curation_20260917/README.md)에 있으며 계속
`operational=false`다.

### 주행 검수 후 구맵 차선 채택 후보

2026-09-16 주행 검수 결과에 따라 원태 observed geometry는 위치 검산 자료로 낮추고,
구맵과 current corridor가 일치하는 선분에서는 구맵의 실제 geometry·색상·형태를 채택한
별도 v2 후보를 생성한다.

```bash
python3 src/asmc_hd_map/scripts/build_legacy_adoption.py
python3 src/asmc_hd_map/scripts/build_candidate_review.py
```

채택 범위는 초기 안정구간, 로터리부터 고주로 진입·톨게이트까지, 음영구간과 이후다.
톨게이트 출구부터 음영구간 입구까지는 현재 맵과 다르므로 제외한다. 교차로도 current
link와 국소적으로 일치하는 선분만 남기며, 선을 명목 3.5m 폭으로 변형하지 않는다.
결과는 구맵 source 163개/segment 195개다. 그중 136개는 current link에 직접 귀속됐고,
교차로 확장·인접 차로 59개는 geometry를 버리지 않되 `route_context_only`로 보존했다.
기존 전역 gate에서 D였던 source 8개도 선분 단위 검산으로 복원됐다. 상세 산출물은
[`legacy adoption report`](reports/kcity2025_legacy_adoption_20260916/README.md)에 있다.
아직 `operational=false`이며 runtime crop이나 판제 입력으로 사용하지 않는다.

### 2025 변경구간 및 고주로 누락선 보완(v3)

구맵 v2 검수 후, 구맵과 실제 2025 도로가 다른 구간과 고주로 누락 경계를 current
node/link 중심으로 보완했다.

```bash
python3 src/asmc_hd_map/scripts/build_hybrid_completion.py
python3 src/asmc_hd_map/scripts/build_candidate_review.py
```

- 정적·동적 장애물 새 도로, 로터리 출구→고주로, 톨게이트→음영구간은 current 공식
  경로를 연속 offset해 재생성했다.
- 동적 장애물 새 도로의 종방향 도색은 `A1256W000208` 정지선인
  `route_s=436.798m`에서 끝냈다. legacy 이중 중앙선과 코너 조각이 다시 나타나지 않게
  했고, 중앙선은 yellow 1개만 남겼다.
- 로터리 전 우측 공유선은 원태 direct 관측의 2차로 양 외곽선 midpoint를 기본으로 하되,
  주행 검수에 따라 route 쪽으로 0.25m 이동하고 최소 half-width 1.40m를 보장한다.
  관측선 자체는 geometry로 복사하지 않고 station별 폭 근거로만 사용했다.
- `A1256W000757` 정지선 뒤 `550.619~600m` 우회전은 white/white `broken` 경계로
  재생성했다. 구맵 실선의 과도한 연장과 중간 공백을 제거했으며 RViz도 3m/3m metric
  dash/gap으로 표시한다.
- 로터리 출구 정지선 뒤 `930.6~974.9m`는 current route를 따라 white/white 직선으로
  재생성해 구맵 connector curve를 제거했다.
- 그 밖의 원태 관측선은 geometry로 복사하지 않고 accepted normal-offset의 robust
  median만 폭 prior로 사용했다.
- 고주로 톨게이트 전 `1593.586~1741.721m`의 안전지대측 구맵 실선은 원태 direct
  `hi_right.csv`에 맞춘 route-right half-width 중앙값 1.710m 경계로 교체했다.
- 고주로에서 누락됐던 lane 1–2 공유 경계 6개를 current topology로 복원했다.
- `on_stop_line` 13개 중 주행 검수상 도색이 없는 로터리 내부 1개와 음영구간 2개를
  제외해 review candidate에는 10개만 남긴다.

최종 출력은 stable legacy 146개, current route-gap/side 경계 12개, 고주로 공유 경계 6개다.
정적 오버레이와 판단 근거는
[`hybrid completion report`](reports/kcity2025_hybrid_completion_20260916/README.md)에 있다.
이 v3도 `operational=false`이며 라이브 주행 검수 후 별도 승격한다.

## 구맵 origin-only 주행 검수

UDP bridge가 `/Ego_topic`을 발행하는 상태에서 다음을 실행한다.
이 예전 단일-source 도구는 `/Ego_topic.position`이 유효한 26.R1 진단용이다. 대회판이나
두 버전을 공통으로 검수할 때는 위 `candidate_map_overlay.launch`를 사용한다.

```bash
roslaunch asmc_hd_map legacy_origin_overlay.launch
```

별도 `map → base_link` TF는 필요 없다. 노드는 Ego 위치·heading으로 주변 60m geometry를
직접 `base_link`로 변환한다. 구→신 맵에는 선언된 origin translation만 적용하며 fitted
yaw·scale·ICP는 적용하지 않는다.

| RViz 토픽 | 의미 |
|---|---|
| `/asmc_hd_map/debug/legacy_lanes` | 구 MGeo lane. white/yellow/blue 원색 표시 |
| `/asmc_hd_map/debug/legacy_link_centers` | 구 link 중심선. 기본 비활성 |
| `/asmc_hd_map/debug/current_link_centers` | 현재 link 중심선. 차선 경계가 아님 |
| `/asmc_hd_map/debug/competition_route` | 주최측 제공 전역경로 |
| `/asmc_hd_map/debug/ego_context` | 후륜축 기준 차량 footprint와 전방 방향 |

구 link까지 비교하려면 다음처럼 켠다.

```bash
roslaunch asmc_hd_map legacy_origin_overlay.launch publish_legacy_links:=true
```

소스 워크스페이스가 아닌 설치 공간에서 실행한다면 데이터가 있는 저장소 루트를
`repo_root:=/root/ws`처럼 명시한다.

이 화면은 위치·방향·구간 변경을 빠르게 검수하기 위한 것이며 실제 노면 RGB가 아니다.
물리 도색과의 픽셀 단위 검수는 후보 구간을 고른 뒤 RGB-BEV overlay에서 별도로 수행한다.

## 원태 관측 기준 vector/raster 정본 후보

`kcity2025_operational_map.yaml`은 이전 hybrid를 바로 운영 맵으로 승격하지 않고 다음 역할을
분리해 새 후보를 만든다.

- current Node/Link: link ID, 연결, 인접·반대 방향 topology
- 원태 `e67f5da` direct observation: 횡방향 위치와 station별 폭 anchor
- 구 KATRI: 관측에 정합되는 구간의 매끄러운 도색 형상과 색상·solid/broken 표현
- 공식 전역경로: route lane과 누적 station, 운영 corridor 선택

원태의 CSV 19개, `waize_katri.yaml`, H5와 모든 upstream candidate/profile은 SHA-256으로
고정된다. `generated/interpolated` 표본, manual bridge와 예전 raster inward offset은
geometry anchor가 아니다. 구맵 template은 관측 coverage 55%, 정합 후 횡오차 p95 0.25m,
heading p95 5도와 current topology 귀속 gate를 통과한 구간만 법선 방향으로 이동한다.
관측 폭과 current link의 명목 3.5m 차이는 lane-area audit에는 남기지만 개별 도색선의
정합 오차가 아니므로 선을 탈락시키는 조건으로 사용하지 않는다.

```bash
python3 src/asmc_hd_map/scripts/build_operational_map.py
```

출력은 `reports/kcity2025_operational_map_20260916/`에 생성된다.

- current centerline 70개와 logical boundary 85개
- physical lane marking 정본 후보 49개(현재 2025 재구성 18개 + 자동 gate legacy 28개
  + live 주행 승인 legacy 3개)
- station width profile을 포함한 lane area 70개
- 같은 방향 고주로 link 30개와 반대 방향 context 2개를 보존
- 원태 H5 allowlist 정지선 7개와 2026-09-16 live 주행으로 도색을 확인한
  `A1256W000736`을 유지하고, 로터리 내부 `A1256W000742`는 제거
- 관측 두 선으로 만든 고주로 안전지대 polygon 1개를 forbidden layer로 차감
- 관측 기반 53개 lane area, 일부 관측 2개, 명목 fallback 17개
- 자동 gate를 통과한 legacy template 28개와 live 주행 승인 3개; 나머지 115개는 최종 feature에서
  제외하고 `review_layers`와 audit CSV에만 보존

위 정지선 제거는 2026-09-16 operational 후보의 당시 판단이다. 최신 route curation에서는
후속 주행 확인을 우선해 `A308/A309` 끝의 H5 결합 정지선을 다시 허용했으며, 두 후보를
운영 입력으로 동시에 사용해서는 안 된다.

첫 구간 `A2256W000748`, `route_s=0.00~72.629m`는 2026-09-16 live 주행 검수로
확정했다. 왼쪽 `B219BS010411`은 yellow solid, 오른쪽의 서로 맞닿은
`B219BS010474+B219BS011269`는 white solid로 저장하며 양쪽 끝의 2m 미만 tail은
`A1256W000736` 정지선까지 current logical boundary로 닫는다. 반대차로 도색은 이 운영
구간에 포함하지 않고, 후속 좌회전 connector `A2256W000182`는 물리 도색선이 아닌
current nominal logical corridor로 유지한다.

최종 vector는 `lane_marking`, `lane_boundary`, `lane_area`, `stopline`,
`forbidden_surface`를 분리한다. 점선은 physical paint만 끊어 표시하고 logical boundary는
연속이다. `static_layers.npz`에는 다음 파생 layer가 들어간다.

```text
road_surface_mask       물리적으로 포장된 candidate lane area
route_corridor_mask     공식 전역경로 차로
maneuver_surface_mask   장애물·음영·고주로에서 검토 가능한 확장 노면
forbidden_surface_mask  안전지대 등 정적 금지 영역
unknown_mask            명목 폭 fallback으로만 만든 저품질 영역
lane_id_raster          current link ID lookup용 정수 raster
boundary_rule_raster    permitted/restricted/prohibited/unknown/stopline
source_quality_raster   nominal/partial/direct 관측 품질
```

`maneuver_surface`는 물리 후보이지 곧바로 주행 허가를 뜻하지 않는다. 판제는
`boundary_rule`과 mission/법규 policy를 별도로 적용해야 한다. 이 출력은 계속
`operational=false`이고 기존 `src/asmc_hd_map/development/won_tae_link_node` H5나 runtime을 변경하지 않는다.

주행하며 후보를 확인하려면 perception ROS 컨테이너에서 빌드·source 후 실행한다.

```bash
roslaunch asmc_hd_map operational_candidate_overlay.launch repo_root:=/root/ws
```

RViz의 `Reviewed hybrid boundaries (candidate)`가 최종 physical marking, `Observed
calibration`이 관측 anchor를 적용한 logical boundary, 빨간 선이 H5 또는 명시적 live
주행 검수로 채택한 정지선이다.
동시에 logger가 1Hz로 `HDMAP_REVIEW x=... lane=... lane_s=... width=... left=... right=...`
형식을 출력하고 JSON을 `/asmc_hd_map/review/context`에 발행한다. 수정 지점을 고정하려면
다음을 한 번 발행한다.

```bash
rostopic pub -1 /asmc_hd_map/review/capture_anchor std_msgs/Empty '{}'
```

기본 저장 위치는 `/tmp/asmc_hd_map_review_anchors.csv`다. logger와 overlay는 review-only이며
판제 runtime에 연결되지 않는다. 구간별 주행 검수에서 direct 관측 대비 0.20m, 폭 0.25m,
도색 시작·끝/형태 전환 1m 기준을 넘는 항목과 17개 nominal fallback lane을 우선 확정한다.

## Competition metadata P1-C

차선 geometry와 분리된 공식 checkpoint·mission metadata는 다음 deterministic bundle로
관리한다.

```bash
python3 src/asmc_hd_map/scripts/build_competition_metadata.py --check
PYTHONPATH=src/asmc_hd_map/src \
  python3 -m unittest src/asmc_hd_map/test/test_competition_metadata.py
```

입력 설정은 `config/kcity2025_competition_metadata_source.yaml`, 생성 결과는
`config/kcity2025_competition_metadata.yaml`이다. 공식 checkpoint 16개, 3 m 반경, 원본 yaw와
route/link projection을 보존하며 mission 15개를 `official`, `derived_official`, `provisional`로
구분한다. provisional은 `planning_enforcement_allowed=false`가 아니면 검증에 실패한다.
유효 속도 profile은 A846 종료까지 60 km/h, A411 시작부터 A153 종료까지 대회 속도상한 없음,
A451 시작부터 START/END까지 다시 60 km/h인 세 비중첩 구간으로 전달한다. A866·A054·A846은
별도 미션이 아닌 일반 전역경로 추종 구간이다.
신호등 인지 활성화 policy는 A236 시작부터 A308 종료까지이며, 해당 구간 밖에서 인지 노드를
중지할 수 있도록 `enable_within_zone_only` hint를 함께 전달한다.

상세 규약은
[`docs/contracts/kcity2025-competition-metadata-v1.md`](../../docs/contracts/kcity2025-competition-metadata-v1.md)를
참조한다.

## 2단계 통합 runtime과 RViz

publisher는 stage14 geometry를 수정하지 않고 map·metadata·공식 route와 P2-B compact product가
일치할 때만 정적 토픽을 발행한다.

```bash
roslaunch asmc_hd_map competition_map_debug.launch

# RViz 없이 실제 latched topic과 MarkerArray 검증
roslaunch asmc_hd_map competition_map_debug.launch rviz:=false
rosrun asmc_hd_map verify_competition_map_topics.py

# 결정적 local parity smoke는 별도 실행에서 Ego map XY를 명시적으로 주입
roslaunch asmc_hd_map competition_map_debug.launch rviz:=false local_pose_source:=ego_status
rosrun asmc_hd_map verify_local_map_topics.py
```

기존 토픽에 `/hd_map/global/lanes`, `/hd_map/global/drivable_areas`,
`/hd_map/global/drivable_grid`가 추가된다. P2-B build는 `operational=true`, `complete=true`이며
69개 Lane과 97개 vector area를 발행한다. RViz의 Map display는 0.2 m derived grid를,
cyan outline/centerline은 Lane geometry를 표시한다. 정적 장애물 allowlist는 주황색, 동적은
자홍색, 일반 우회전은 연두색 outline으로 구분한다. 동적 장애물 교차로는 검토 anchor 4개의
directed start/end node-pair closure로 내부 link 4개를 더해 총 8개를 사용하고, 빠져나가는
차로인 A179는 제외한다. 모든 link polygon은 노드 법선에서 flat cap으로 끝난다. 음영구간은
A126 옆 반대방향 A131만 명시적으로 포함한다. reviewed 경계 보정 41개 Lane, 인접 차로 fill
18개와 H5-검증 교차로 gap patch 10개가 Lane 사이 및 교차로 내부의 빈틈을 채운다.

운영 launch는 ego pose 기반 `/hd_map/local/lanes`와 `/hd_map/local/drivable_grid`도 기본으로
발행한다. 기본 crop은 map frame 120 m 정사각형이며 grid는 global cell에 정렬된
`600×600@0.2 m`다. local Lane은 global ID와 원 geometry를 유지하는 subset이라
`complete=false`이고, 판제는 여기에 `base_link` 변환을 다시 적용하면 안 된다.

노드 부근의 짧은 paint gap도 publisher가 자동으로 붙이지 않는다. 판제가 geometry gap을 어떻게
해석해야 하는지와 P2 topology 계약은
[`HD Map runtime·visualization v1`](../../docs/contracts/hd-map-runtime-visualization-v1.md)에 정리했다.

## P2-A lane topology·drivable-area 입력 감사

P2-A는 기존 geometry를 변경하기 전에 Node/Link road prior, reviewed marking과 topology를
동일 좌표계에서 검증하고 clip 가능 구간과 보호구간을 고정한다.

```bash
PYTHONPATH=src/asmc_hd_map/src \
  python3 src/asmc_hd_map/scripts/build_drivable_area_input_audit.py --check
```

정본 manifest는 `config/kcity2025_drivable_area_input_audit.yaml`, 사람이 읽는 결과는
[`P2-A input audit`](../../docs/baselines/kcity2025-drivable-area-input-audit-v1.md)이다. 현재 39개
경로 링크 중 18개는 기존 road 전부 보존, 15개는 끝단 보호와 중간 clip 후보의 혼합, 6개는
전 구간 lane-clip 후보로 분류됐다. 이 수치는 clip을 이미 적용했다는 뜻이 아니다.

기존 `link_node_map.h5`의 `road`는 vector road mesh가 아니라 Node/Link 중심선 buffer와 실측
보정으로 만든 5 px/m raster다. 이 감사의 guard/candidate 분류는 P2-B v1의 H5 hybrid 후보를
검토하기 위한 기록으로 보존한다. P2-B v4는 반경 확장의 경계가 불명확하다는 주행 검수를
반영해 reviewed route와 명시 link를 우선하고, H5는 선택된 장애물 교차로 envelope 안의
gap patch 경계 검증에만 사용한다. P2-A 동안
stage14 geometry hash와 `/hd_map/*` publisher 동작은 바뀌지 않았다.

## P2-B route-compact 주행가능영역

전체 원태 H5 road나 교차로 반경 mask를 판제에 주지 않는다. 공식 경로 39개, 같은 방향 usable
차로, explicit 장애물 회피 link와 음영구간 반대차로 A131만 선택한다. 일반도로는 reviewed
차선의 국소 횡거리로 경계를 보정하고, 선택된 인접 차로 사이를 vector corridor로 채운다.
정적·동적 교차로만 선택 link envelope 내부에서 H5 road와 결합한다. 일반 교차로는 전역경로
link 외의 노면을 추가하지 않는다.

```bash
PYTHONPATH=src/asmc_hd_map/src \
  python3 src/asmc_hd_map/scripts/build_compact_drivable_map.py \
  --repo-root "$PWD"

roslaunch asmc_hd_map competition_map_debug.launch
rosrun asmc_hd_map verify_competition_map_topics.py
```

입력 정책은 `config/kcity2025_compact_drivable_source.yaml`, vector/topology 산출물은
`config/kcity2025_compact_drivable_map.json`, raster 정본은
`config/kcity2025_compact_drivable_grid.npz`다. 결과와 소비 규칙은
[`P2-B compact map`](../../docs/baselines/kcity2025-compact-drivable-map-v1.md)에 기록한다.

판제 geometry 정본은 차로 번호·인접 관계·경로 station과 polygon을 담은 `LaneArray`와
`DrivableAreaArray`다. `OccupancyGrid`는 이를 0.2 m/px로 변환한 빠른 lookup용 파생물이다.
교차로 support link는 명시된 미션 공간이며 policy 밖 전역경로 이탈 허가를 뜻하지 않는다.
동적 장애물 교차로의 node-pair closure는 지정 anchor들의 시작 노드 집합에서 끝 노드 집합으로
향하는 교차로 link만 채택하므로 접근로·반대 방향·인접 도로를 추가하지 않는다.
`DrivableAreaArray`에는 69개 Lane 면 외에 18개 인접차로 fill과 10개 교차로 gap patch가
별도 provenance로 포함된다. 따라서 판제는 개별 Lane뿐 아니라 이 전체 vector union을
주행가능영역으로 사용해야 한다.
물리 paint gap은 계속 `LaneMarking`에 남지만 연속 주행영역은 Lane
polygon/grid로 전달되므로 노드 주변의 짧은 도색 공백 때문에 주행가능영역이 끊기지 않는다.
