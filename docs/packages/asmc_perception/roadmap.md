# 카메라·MGeo·LiDAR 인지 융합 개발 로드맵

> **문서 역할:** Working Proposal + Chronological Record — 인지 융합 방향과 단계별 결정 gate
> **문서 허브:** [README.md](../../README.md)
> **현재 코드 진입점:** [Perception 개발·통합 가이드](./README.md)
> **상태:** Working Proposal v0.19 — 음영 camera v3 인수 완료, current Node/Link HD맵 관측 보정 후보 생성<br>
> **기준일:** 2026-09-16<br>
> **주요 참여:** 안승현(카메라 인지), 장원태(MGeo map prior 제안), 손재호(LiDAR 인지), planning/control 팀<br>
> **상세 문서:** [인지 GT·BEV·차로 구조화 계획](./bev-lane-interface-plan.md) · [운영 가이드](../../operations/README.md) · [v4 학습 평가](../../records/perception/twinlitenet-v4-evaluation.md)

이 문서는 팀 전체의 개발 방향을 맞추기 위한 제안서다. 차량·센서 기준 좌표는 규정과
센서셋에 맞춰 정본화하되, 토픽 이름, BEV raster 범위·해상도, 메시지 필드와 담당 경계는
아직 최종 계약이 아니다. 카메라 BEV 투영과 map bake pilot 결과를 확인한 뒤
`src/interfaces/`에서 별도 합의하고 변경 이력을 남긴다.

> **시점 주의:** 이 문서에는 2026-09-16 전후의 구현 과정과 당시 후보 상태가 시간순으로
> 보존돼 있다. 현재 package, 실행법과 완성된 HD Map 상태는 위 package-group guide와 각
> package README·동결 contract가 우선한다. 과거 분석은 삭제하거나 최신 상태로 재작성하지
> 않았다.

### 제안·구현 출처와 현재 통합 기준

이 문서의 전체 우선순위와 파트 경계는 **안승현의 현행 통합안**을 기준으로 정리했다.
팀 확정 계약은 아니며, 담당자 검토에서 다른 결론이 나오면 근거와 함께 갱신한다. 다른
팀원의 의견이나 구현을 통합한 부분은 다음처럼 출처를 남긴다.

| 범위 | 제안·요구 출처 | 구현·검증 출처 | 이 문서에서의 취급 |
|---|---|---|---|
| 3카메라 drivable/lane, 음영 shared-family | 안승현 | 안승현의 bag·live 평가 | 현재 camera handoff 기준선 |
| MGeo/node-link map prior와 offline bake | 장원태 | 장원태 `src/asmc_hd_map/development/won_tae_link_node/`, PR #37 | local crop·대회 맵 검증 전 후보 |
| LiDAR 장애물·OBB·속도/추적 운용 요구 | 손재호 | 장원태가 요구를 통합 구현한 PR #37 | `/last_lidar`·camera fusion 후보, 담당자 재검증 필요 |
| safe corridor, 회피·복귀와 최종 제어 권한 | planning/control 협의 대상 | 미확정 | 인지 결과를 소비한 뒤 판제가 결정 |

따라서 PR #37의 코드 작성자는 장원태로 기록하되, LiDAR 요구 자체를 장원태의 단독
의견으로 표기하지 않는다. 반대로 이 문서의 통합 우선순위를 팀 전체의 확정 결정으로
표현하지도 않는다.

## 1. 목표와 우선순위

대회 우선 목표는 허용 센서만 사용해 K-City 경로를 안정적으로 완주하는 것이다. 하나의
인지 결과를 절대 기준으로 삼기보다 다음 세 계층을 분리하고, 신뢰도와 시간 정보를 가진
공통 ego-BEV에서 결합하는 방향을 우선 검토한다.

| 계층 | 주 역할 | 현재 방향 |
|---|---|---|
| MGeo map prior | 정적 도로 구조, route corridor, 차로 위상, 정지선 | 제공 데이터를 offline bake한 뒤 허용 측위로 local crop |
| Camera perception | 현재 보이는 drivable·white/yellow/stopline 관측 | TwinLiteNet+ 4-class road-marking 모델을 live 검증, 모델 교체 가능한 adapter 사용 |
| LiDAR perception | 동·정적 장애물의 위치·점유·추적 | 장애물 검출을 도로/차로 문맥과 연결하고 최종 safe area에서 우선 반영 |

현재 작업 순서는 **재현 가능한 Real Time UDP raw bag 기준 → 음영 camera v3 인수 → MGeo
bake/crop 결합 → LiDAR/판제 연계 → 대회판 이식**이다. Streaming/ROS GT transport 비교는
종료했으며 관련 미병합 구현은 `origin/backup/ros-gt-experimental-20260910`에만 보존한다.
정지선은 MGeo/전역경로 정지점을 1차 후보로 두고 현재 영상 stopline class를 보조 검증과
GPS Blackout 대응 후보로 평가한다.

2026-09-11부터 전체 맵의 차선 topology를 한 번에 해결하지 않고 GPS 음영 왕복 2차로를
첫 camera-lane ODD로 둔다. 개발 launch는 원거리 열화를 관찰하는
`x[-5,30]×y[-12,12] m` exploration grid를 사용하지만, 이 범위는 판제 유효범위가 아니다.
정식 pilot의 `[-5,20]×[-8,8] m` 설정은 유지한다. 인지는 yellow/white 경계, 그 사이 local
lane strip과 기하학적 current occupancy를 제공하고, home/opposing 의미와 회피·복귀 판단은
route seed 및 LiDAR occupancy를 가진 판제·퓨전이 소유하는 방향으로 검증한다.

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

> **상태 주의:** 이 절은 2026-09-16~17 `asmc_hd_map` 후보를 발전시킨 시간순 기록이다.
> 아래의 “미구현”, `candidate`, `operational=false`는 당시 snapshot을 뜻한다. 현재 Phase 2
> 제품과 실행·소비법은 [HD Map package guide](../asmc_hd_map/README.md)와
> [runtime contract](../../contracts/hd-map/runtime-v1.md)를 우선한다.

- drivable surface와 route corridor
- lane boundary/centerline, lane width와 연결 관계
- 경로 선택지가 없는 구간의 nominal ego lane
- 좌·우 인접 차로와 차로 변경 가능성
- stop-line geometry와 전역경로상의 정지 후보점
- 속도 제한/예외 link 같은 정적 route attribute

정적 지도에는 NPC와 스폰 장애물을 굽지 않는다. 지도 산출물에는 map 버전, bake 설정,
전역경로 hash와 좌표계가 포함되어야 한다.

**장원태 제안·구현(PR #37): [`src/asmc_hd_map/development/won_tae_link_node/`](../../../src/asmc_hd_map/development/won_tae_link_node/README.md)** (offline bake 부분).
`link_set.json` + `node_set.json`에서 drivable / lane / stop-line을 이진 래스터로 굽고
`map_id`·bake 설정·좌표계 원점을 H5 attrs에 담는다. catkin 패키지가 아닌 오프라인
CLI라서 `src/perception/` 아래가 아니라 `src/asmc_hd_map/development/won_tae_link_node/`에 둔다. **runtime local crop은
아직 미구현**이며 담당 파트 미정이다 (기능적 프로토타입은 `live_check.py`의
`ego_view()`, 0.28 ms/프레임). 위 목록 중 lane 연결관계·인접 차로 변경 가능성·속도
제한 attribute는 아직 굽지 않는다.

현재 산출물은 `behavior_planner`의 `waize_katri_2025-10-23` 판본에서 생성한
근사 map prior다. H5의 `map_id`와 원점 검사를 통과하더라도 공식 대회 K-City 판본 및
실제 도색과의 검산 전에는 deployment 정본으로 승격하지 않는다.

**안승현 current-only 재구축 후보: [`src/asmc_hd_map/`](../../../src/asmc_hd_map/README.md).**
기존 `src/asmc_hd_map/development/won_tae_link_node`를 수정하지 않고, 현재 K-City 2025 link/node를 topology 정본으로 삼는
별도 provenance-aware vector 후보를 만들었다. 주최측 전역경로와 current route link 39개는
폐루프 2,184.612m 전 구간에서 일치했다. route, 고주로 인접 차로, 음영과 정적 장애물
양방향 context를 합친 current link는 70개다. 고주로 15m corridor의 평행 link 32개는
진행 방향이 같은 30개와 반대 방향 2개로 명시적으로 분류한다. 경로에 짧게 닿은 link가
다른 도로까지 이어지는 경우 topology ID는 유지하되 export geometry만 corridor로 자른다.

첫 단계에서는 current link 폭을 법선 offset해 명목 lane boundary 85개를 만들고, 인접
link의 공통 경계 21개를 중복 제거했다. 현재 맵의 모든 `width_start/end`가 3.5m이므로 이는
실제 도색 GT가 아니다. 합류·분기·비일반 link 등 42개 경계는 `review_required`다. 공식
route에 연결된 `on_stop_line` node 13개는 모두 명목 stopline 후보를 가지지만 실제 도색
선분과의 검산 전이다. 모든 feature는 `candidate`, map은 `operational=false`이며 관측선과
구 KATRI 자료는 이 기준선 생성에 사용하지 않았다. 정량 결과와 source hash는
[`current corridor report`](../../../src/asmc_hd_map/reports/kcity2025_current_corridor_20260916/README.md)에
있다.

2단계에서는 장원태가 수집한 19개 관측 CSV 26,511행을 별도 계약으로 감사하고, 직접
관측 25,414행만 current 명목 경계에 정합했다. 고정 폭 생성 958행, 시간 보간 139행,
수동 bridge와 raster용 0.03m offset은 canonical fit에서 제외했다. 자동 gate를 통과한
76개 match run으로 85개 parent 경계 중 50개에 60개 보정 interval을 만들었으며, 나머지
구간은 명목 후보로 유지했다. 양쪽 관측이 함께 통과한 폭 표본은 3,143개이고 중앙값은
3.468m지만, 합류 taper를 포함하므로 current link의 일률적 3.5m 속성을 덮어쓰지 않는다.
link centerline 70개와 stopline 13개는 1단계 그대로다. 상세 결과는
[`observed calibration report`](../../../src/asmc_hd_map/reports/kcity2025_observed_calibration_20260916/README.md)에
있다. 이 결과도 `candidate`, `operational=false`다. 다음 gate는 자동 적용과
`review_required` overlay 검수 후, 변경되지 않은 구간에만 구맵 속성·누락 geometry를
보조 적용하는 것이다.

고주로 검수에서는 공식 route 차로만 남기지 않고 current 평행 link 32개를 보존했다.
route에서 좌우 인접 topology로 도달 가능한 25개 차선변경 link는 전부 포함되어 있으며,
21개는 원태 관측 보정 interval을 갖고 4개는 명목 폭 후보로 남아 있다. 원태 보정선과
동일 current link에 대응된 고주로 구맵 A/B 경계의 거리 p50/p95는 0.105/0.515m였다.
따라서 변경되지 않은 고주로의 Tier A 구맵 geometry·색상·선 형태는 우선 이식 후보로
분류한다. 단 current node/link가 topology를 계속 소유하며, Tier B·분기·변경구간은
원태 current-map 관측 또는 주행 검수 없이 자동 승격하지 않는다. 구간별 비교와 주행용
RViz layer는 [`candidate review report`](../../../src/asmc_hd_map/reports/kcity2025_candidate_review_20260916/README.md)에
정리했다. 이 검수 bundle 자체는 map을 변경하지 않고 `operational=false`를 유지한다.
주행 검수 pose는 대회판에서 `/Ego_topic.position=(0,0)`으로 파싱된 사례를 확인했으므로
이를 사용하지 않고 `/gps` UTM52N 위치와 `/imu` yaw를 조합한다. candidate map 원점
`(302595, 4124145)m`과 GPS 전방 lever arm 0.35m를 명시적으로 적용하며, 이 변환은
review node 내부에서만 수행해 공용 `map → base_link` TF를 발행하지 않는다.
동일 원칙을 2단계 planner-facing local crop에도 적용한다. 현재 센서의
`eastOffset/northOffset=(302595, 4124145)m`은 맵 원점과 일치하며, 실측 GPS를 변환한
`(-131.3, -427.7)m`은 START 부근이다. 따라서 `(0,0)`에서 약 60 m 떨어져 보인 현상은
origin 오류가 아니라 map XY가 없는 `EgoVehicleStatus`를 강제로 선택한 입력 오류다.

2026-09-16 초기 주행 검수에서는 정지선 13개를 current 후보로 유지하고, 차선은 구맵과
current corridor가 일치하는 안정 구간의 원본 geometry·색상·형태를 우선 채택하기로 했다.
원태 관측선은 깔끔하지 않은 구간이 있어 canonical geometry가 아니라 validation reference로
사용한다. 로터리~고주로~톨게이트와 음영구간 이후를 포함해 구맵 source lane 163개,
segment 195개를 별도 v2 후보에 보존했다. 이 중 인접 link가 현재 70-link 골격에 없는
59개는 잘못된 소유권을 붙이지 않고 `route_context_only`로 남겼다. 톨게이트 출구~음영 입구의 변경구간은 제외했고
국소적으로 일치하는 19개 조각도 review-only로 격리했다. 차로폭은 일률 3.5m가 아니라
채택한 실제 구맵 geometry를 따르며 current node/link가 topology 정본인 원칙은 유지한다.
상세 기준과 오버레이는
[`legacy adoption report`](../../../src/asmc_hd_map/reports/kcity2025_legacy_adoption_20260916/README.md)에
있다. 이 후보도 전체 gap/transition 주행 검수 전까지 `operational=false`다.

후속 주행 검수에서는 2025 변경구간에 남은 legacy curve fragment, 고주로 lane 1–2 공유
경계 누락, 실제 도색이 없는 정지선을 확인했다. v3 hybrid candidate는 안정 구간 legacy
geometry를 유지하면서 정적·동적 장애물 새 도로, 로터리 출구→고주로, 톨게이트→음영
전환을 current 공식 경로로 재생성한다. 동적 장애물 구간 종방향 도색은 실제
`A1256W000208` 정지선 `route_s=436.798m`에서 끝내고 단일
yellow로 확장했고, `930.6~974.9m` 고주로 진입의 구맵 connector curve는 current 직선
경계로 교체했다. 로터리 전 우측 공유선은 원태 direct 2차로 외곽 관측의 station별
midpoint를 route 쪽으로 0.25m 보정하되 이중 yellow는 유지한다. 다른 원태 관측은 직접 geometry가 아니라
link별 폭 prior로만 사용한다. `A1256W000757` 정지선 뒤 `550.619~600m`는 실제 도색처럼
white/white broken 경계로 바꾸고 RViz에서도 metric dash/gap으로 표시한다. 고주로 공유
경계 6개를 current topology에서 복원했다. 톨게이트 전 `1593.586~1741.721m`
안전지대측 경계는 구맵 약 2.2m offset 대신 원태 `hi_right.csv` 직접 관측이 지지하는
route-right half-width 중앙값 1.710m로 교체했다. 로터리 내부
`A1256W000742` 및 음영구간 `A1256W000831`, `A1256W000842`를 non-painted로 기각해
stopline candidate를 10개로 줄였다. 상세 산출물은
[`hybrid completion report`](../../../src/asmc_hd_map/reports/kcity2025_hybrid_completion_20260916/README.md)에
있으며, 여전히 `operational=false`다.

후속 원태 관측 기준 operational candidate 주행 검수에서는 여러 source를 한 화면에 섞은
상태로는 구간별 geometry 선택 근거를 분리하기 어렵다는 문제가 확인됐다. 따라서 해당
후보의 추가 승격은 일시 중지하고, 다음 세 입력만 독립 토픽으로 보여 주는
`three_source_map_compare.launch`를 추가했다.

- 원태 관측 기준 정본화 계획 직전의 hybrid candidate
- 원태 커밋 `e67f5da` H5에 저장된 관측 차선/정지선 vector
- origin translation만 적용한 구 KATRI Tier A/B 차선

구맵 layer는 current 대회 corridor 70개 link와 대응된 127개 source로 제한한다. 이 집합은
기존 고주로 전체와 공식 route/connector 관련 경계를 보존하되 무관한 도로는 제거한다.
세 source는 display 단계에서도 merge하지 않으며, 구간별 주행 검수로 선택이 끝날 때까지
자동 offset·blend·smoothing을 다시 적용하지 않는다. 기존 operational candidate와
surface bake는 provenance와 실패 분석용 artifact로만 보존한다.

2026-09-17 route curation은 이 원칙에 따라 14개 작은 검수 단계로 분리했다. 첫 구현은
`route_s=[0,186.286m)`, `A2256W000748→182→329→236`뿐이며 origin-only legacy를 사용한다.
`A2256W000748` route-right의 legacy source `B219BS010474`만 주행 화면 근거로 broken에서
solid로 교정했다. 정지선과 surface는 독립 allowlist가 확정되기 전까지 출력하지 않는다.
`A2256W000219` 이후 H5/pre-plan/legacy 조합 요청은 문서와 config에는 보존했지만 아직
geometry를 만들지 않는다. 상세 내용은
[`경로별 HD맵 선택 검수`](../../records/hd-map/kcity2025-route-curation-20260917.md)를 따른다.

2026-09-16 원태 관측 기준 operational candidate는 current topology 70개, logical
boundary 85개, lane area 70개를 유지하면서 physical marking 49개를 선택했다. 이 중
legacy는 자동 gate 28개와 live 주행 승인 3개다. 구맵 paint gate는 관측 coverage 55%, 횡오차 p95 0.25m,
heading p95 5도와 current topology 귀속을 요구한다. 실제 관측 폭과 current link의
fallback 3.5m 차이는 개별 선의 오차가 아니므로 lane-area audit에만 사용한다. 시작 구간
`A2256W000748`, `route_s=0.00~72.629m` 첫 확정 구간은 yellow solid/white solid로
고정했다. 오른쪽은 서로 맞닿은 구맵 두 조각을 연결하고 양쪽의 짧은 tail은 첫 정지선까지
logical boundary로 닫았다. 후속 `A2256W000182` 사거리 connector는 물리선이 아닌 current
nominal logical corridor로 유지한다. 또한 live 주행에서
도색을 확인한 `A1256W000736` 정지선은 원태 H5 7개와 구분된 driving-review provenance로
추가했다. 총 정지선은 8개이며 후보는 계속 `operational=false`다. 정량 audit과 파생
raster는 [`operational candidate report`](../../../src/asmc_hd_map/reports/kcity2025_operational_map_20260916/README.md)에
기록한다.

### 3.2 Camera perception 후보 출력

- view별 `drivable_probability`, `road_marking_probability`
- road marking class `background=0`, `white_lane=1`, `yellow_lane=2`, `stopline=3`
- 원본 영상에서의 coverage와 유효 영역
- 모델/checkpoint/전처리 provenance
- timestamp, frame age와 추론 지연
- ego-BEV로 투영한 확률, class ID 및 관측 confidence

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

**손재호의 LiDAR 운용 요구를 반영해 장원태가 구현한 PR #37 후보**는 tracked OBB를 map
ENU로 변환하고, `src/asmc_hd_map/development/won_tae_link_node` road mask에 대한 5×5 footprint sampling 비율로
주행가능영역 후보만 `/last_lidar`에 발행한다. source-stamped YOLO 결과와 LiDAR frame을
시간 매칭해 `person/car/unknown` 및 원본 `yolo_class` 근거도 함께 보존하며, 동적 판정은
별도 `MotionClassifier`로 분리했다. 이는 LiDAR 객체의 도로 문맥 필터와 분류 후보이지,
LiDAR occupancy를 차감해 최종 주행 corridor를 확정하는 구현은 아니다. 손재호의 최신
tracker 기준선, 대회 맵 판본, full-stack 지연과 판제 fallback을 재검증한 뒤 소비 권한을
올린다.

이 후보가 사용하는 raw GPS+latest IMU 기반 `map -> base_link` TF는 정식 측위
출력이 아니다. 기본 UDP 브리지에서는 `publish_map_tf:=false`이며, 장원태의
PR #37 기반 LiDAR 융합을 단독 검증할 때만 명시적으로 켠다. 향후 측위 노드가
동일 TF를 발행하면 브리지 쪽 TF를 동시에 켜지 않는다.

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
├── drivable_bev/                   # 현재: camera-only fused BEV pilot, 추후 map layer/fusion 후보
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
/perception/camera/{view}/road_marking/class_id
/perception/camera/{view}/road_marking/confidence
/perception/camera/{view}/debug/panel/compressed
/perception/camera/debug/mosaic/compressed
/perception/camera/diagnostics
```

- drivable probability와 road-marking confidence는 `sensor_msgs/Image` `mono8`의 0~255다.
- road marking class ID는 `0=background, 1=white, 2=yellow, 3=stopline`이다.
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
/perception/bev/camera/road_marking/class_id
/perception/bev/camera/road_marking/confidence
/perception/bev/camera/coverage
/perception/bev/camera/source_count
/perception/bev/lidar/occupancy
/perception/bev/fused/safe_drivable
/perception/bev/fused/lanes
```

초기 raster는 `nav_msgs/OccupancyGrid` 또는 `sensor_msgs/Image`, vector lane은
`nav_msgs/Path`/marker로 pilot할 수 있다. 최종 planning 소비 필드가 확인되면 versioned
custom message를 검토한다.

### 6.4 아직 확정하지 않을 항목

- map/LiDAR까지 포함한 최종 BEV 전·후·좌·우 범위와 meter/pixel
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

**2026-09-10 camera-only BEV pilot 구현:** `src/perception/drivable_bev/`에서 MGeo·LiDAR와
독립된 front/left/right 투영을 구현했다. 초기 grid는 후륜축 `base_link`, x-forward/y-left,
`x=[-20,60]m`, `y=[-20,20]m`, `0.10m/cell`이었다. v2 정지 교차로
live에서 세 view가 약 10 Hz로 출력됐고 projection+publish p95는 `4.74~5.23 ms`, 오류는
0건이었다. RGB ground-grid와 coverage로 pitch/yaw 및 좌우 축을 육안 확인했다. 짧은 8구간
비교에서 추론-only와 BEV projector+구독자 없는 lazy visualizer의 처리량 중앙값은 각각
약 `49.5`, `47.9 view-fps`였다. debug JPEG 구독 부하는 운용 계측과 분리한다. 설정의
60 m 상한을 유효 인지거리로 해석하지 않는다.

**2026-09-10 fused BEV 범위 audit:** v6 EMA와 validation 1,404 sample을 `run/frame`으로
묶은 결과 예상 156개가 아니라 468개의 complete 3-view group이 확인됐다. 계획한
`x_max={30,25,20}`, `|y|max={12,10,8}`의 sunny/foggy·5 m 종방향·4 m 횡방향 평가에서 모든 기준을
통과한 후보는 없었다. 명시된 fallback `x=[-5,20]m`, `y=[-8,8]m`, `0.10m/cell`을 적용했다.
이 범위의 전체 drivable IoU는 sunny/foggy `0.9827/0.9738`이었지만 white/yellow IoU는
`0.5191/0.4833`, recall은 `0.5439/0.5069`였다. 얇은 차선은 작은 image-space 오차가
metric raster에서 크게 반영되므로 현재 범위를 차선 강건성 보증으로 해석하지 않는다.
카메라별 geometry/edge/GSD/range 품질맵과 class confidence로 세 view를 융합하고 35 ms
timestamp gate를 적용했다. 정식 fused 출력의 live rate·sync 거부율·지연은 아래 동적
주행 gate에서 확인했으며 seam은 lane fitting 단계에서 다시 평가한다.

**2026-09-10 fused live/replay gate:** BEV 통합 launch만 20 ms view coalescing을 적용하고
35 ms source-stamp gate와 10 Hz fused throttle을 유지했다. live 누적 표본은 fused
`9.66 Hz`, sync 폐기율 `3.33%`, fusion p95 `3.90 ms`, camera-to-fused p95 `95.35 ms`였다.
242.437초 `camera_raw`를 1.0× replay한 별도 result bag에서는 fused `9.84 Hz`, 폐기율
`3.66%`, fusion p95 `5.42 ms`, camera-to-fused p95 `111.54 ms`, 추론 약 `50.1 view-fps`,
decode/inference error 0건이었다. raw RGB 세 view는 각각 약 20 Hz이고 header regression은
없었다. 다만 validation overlap marking Chamfer p95 목표는 통과하지 못했으므로 현재
fused raster를 직접 steering 기준선으로 승격하지 않는다. 다음 lane fitting에서 같은
class·방향의 물리 선을 매칭한 metric seam audit을 다시 수행한다.

위 2026-09-10 범위·loss 수치는 노면을 `base_link z=0`으로 가정했던 역사적 기준이다.
MORAI의 후륜축 중심 원점과 실제 노면 사이 높이를 빠뜨린 사실을 2026-09-11 실측으로
확인했으므로, 기존 artifact는 삭제하지 않되
`superseded_due_to_ground_plane_contract`로만 보존한다. 현재 정본 수치는 아래 보정 절을
사용한다.

**2026-09-10 native GT loss-budget gate:** 기존 범위 audit의 GT가 `384×640` cache에서
복원된 값임을 확인하고, native GT를 기준으로 468개 3-view frame의 여섯 경로를 다시
비교했다. cache 왕복은 exact white/yellow IoU가 `0.4573/0.6337`이지만 F1@0.2m은
`0.9504/0.9820`으로, 주 손실은 얇은 선의 두께·미세 위치 변화였다. 실제 prediction은
native 복원을 생략한 model-grid hard direct 투영에서 F1@0.2m이 `0.7962/0.8920`에서
`0.8400/0.9130`으로 개선됐다. 반면 단순 4-class soft 평균은 Sunny/Foggy 비회귀 및
Chamfer·연결성 gate를 통과하지 못해 runtime 승격을 보류한다. 세부 수치와 overlay는
당시 세부 수치는 기존 artifact에 보존하며 현재 정본으로 사용하지 않는다.

**2026-09-11 ground-plane 계약 보정:** `base_link`는 노면이 아니라 후륜축 중앙이므로
`cameras_v2.yaml`에 필수 평면 `z=-0.35 m`를 추가했다. 좁은 구간/음영구간 LiDAR 실측
원점 높이는 `-0.354 m`, 20-frame 중앙값 `-0.331 m`였고, 보정된 카메라 역투영 차로폭은
각각 약 `3.00 m`, `3.61 m`였다. LiDAR 도색점→RGB p95는 최대 `0.43 px`, 공통 도색선의
3-view 횡잔차 p95는 `0.03 m` 이하로 intrinsic/extrinsic은 그대로 유지했다. 카메라 mount
높이를 바꾸지 않고 homography, coverage/GSD, model-grid, visualizer와 offline evaluator가
하나의 불변 평면을 사용하며 설정이 없으면 시작을 실패시킨다.

동일 468개 Sunny/Foggy frame 재평가에서 prediction native-hard의 drivable/white/yellow/
stopline IoU는 `0.9827/0.5306/0.6589/0.7873`, white/yellow F1@0.2m은
`0.8924/0.9530`으로 정정됐다. z=0 결과보다 white/yellow IoU는 `+0.1164/+0.0982`지만,
이는 모델 향상이 아니라 물리 지면 기준 오류 제거 효과다. 단순 soft 융합은 새 기준에서도
F1/Chamfer 채택 gate를 통과하지 못했다. 범위 audit도 모든 후보가 실패해
`x[-5,20]×y[-8,8] m`는 provisional로 유지한다. BEV-only live는 fused `9.13 Hz`,
fusion p95 `4.36 ms`, camera-to-fused p95 `98.23 ms`, per-view projection p95
`2.78~3.72 ms`, 오류 0이었다. lane-vector는 30그룹 smoke만 통과시켰으며 재튜닝 수치로
채택하지 않는다. 세부 정본과 artifact 링크는
[`drivable_bev` README](../../../src/perception/drivable_bev/README.md#2026-09-11-ground-plane-보정-및-재평가)를
따른다.

**2026-09-10 metric lane-vector pilot:** 모델 격자 class/max-confidence를 최대 10 Hz의
lazy internal topic으로 추가하고, raster 확대 없이 각 pixel ray를 연속 `base_link (x,y)`
evidence로 변환했다. white/yellow evidence를 종·횡 metric cluster, source association,
robust cubic B-spline 순서로 처리하고 PointCloud2·MarkerArray와 lazy rqt debug 화면을
구현했다. 그러나 468개 Sunny/Foggy native-GT frame에서 fitted white F1@0.2는
sunny/foggy `0.733/0.733`으로 direct evidence `0.809/0.785`보다 낮았고 Chamfer p95도
악화됐다. 중복 spline은 77 frame, offline vectorization p95는 `83.0 ms`여서 정확도·지연
gate를 통과하지 못했다. confidence 0.60, 0.40 m bin, 4 m 최소 span과 shape-preserving
curve fallback도 정확도 비회귀를 만족하지 못해 채택하지 않았다.

따라서 이번 pilot의 채택 결과는 **보정된 model-grid direct metric evidence와 디버그 배관까지**다.
spline은 원인 분석용 출력으로만 유지하며 planning 인터페이스나 validated BEV 범위로
승격하지 않는다. `x[-5,20]×[-8,8] m`는 계속 provisional debug ROI다. ground-plane과
letterbox 검산 뒤 음영구간의 rejection 원인과 표시 계층을 분리했고, MGeo 링크에서 만든
pseudo-GT를 카메라·LiDAR로 검산한 뒤 shared lane-family 대표 offline A/B까지 완료했다.
일반 구간의 비차로 표식 제거와 MGeo candidate matching은 이 음영구간
기준선이 안정된 다음 단계로 둔다. 이후 visible-only GT의 관측구간 정량 평가와 60-frame
false-bridge 검수를 통과한 뒤 범위 audit을 다시 실행한다. 상세
실행·artifact 계약은
[`drivable_bev` README](../../../src/perception/drivable_bev/README.md#2026-09-10-metric-lane-vector-pilot)를
따른다.

또한 Semantic의 white/yellow는 색상 표식이므로 같은 색의 주차칸·안전지대 표식을
lane boundary와 자동으로 구별하지 않는다. 현재 spline은 모두 `lane candidate`이고,
종방향 연속성만으로 진행 방향과 평행한 주차선을 완전히 제거할 수 없다. 60-frame 검수에
비차로 표식 오검출을 별도 항목으로 두고 2% 이하를 range 승격 gate에 포함한다. 방향·길이,
평행 경계와 drivable 문맥으로도 분리가 부족하면 MGeo 또는 lane-boundary 전용 GT를 붙인
별도 classifier/모델 변경 실험으로 분리한다.

### Stage C3.1 — 음영구간 physical-prior lane family

대회 우선범위는 전 구간 lane topology가 아니라 GPS 음영 왕복 2차로의 장애물 회피·반대
차로 진입·복귀다. 현재 TwinLiteNet+ class mask는 대체로 연속인데 lane-vector와 planning
strip이 끊기는 현상이 있으므로 새 인지 모델을 추가하기 전에 후처리 손실을 분리한다.
기존 replay에서도 clear/static yellow 관측률은 `98.03/96.25%`였지만 lateral sweep에서는
`77.6%`, 최장 no-yellow streak는 13 frame이었다. 이는 전역적인 segmentation 실패보다
차량 횡이동 때의 confidence 절단·후보 분절·독립 spline 선택·시간축 형상 미보존을 먼저
고쳐야 한다는 근거다.

제공 sample scenario의 `shadedAreaList`는 중심 `(-23.668,-519.240)`, 약
`117.959×7.419 m` 영역을 담는다. 이 구간의 양방향 link `A2256W000126`과
`A2256W000131`은 각각 약 119 m이며 width는 3.5 m다. 반대 방향을 같은 arc-length 방향으로
정렬한 두 link centerline 간격은 중앙값 약 3.510 m다. 배포 데이터에는 이 link가 참조하는
lane-boundary geometry가 없으므로 첫 GT는 link centerline·width로 만든 **map-derived
pseudo-GT**로 표기하고, 카메라/LiDAR 도색점 또는 장원태 map bake로 검산하기 전에는
정식 boundary GT라고 부르지 않는다. 개발 평가에 사용하는 전역 pose/oracle 정보는 본선
subscriber에 연결하지 않는다.

음영구간 세 경계는 서로 독립된 곡선보다 공유 기준 곡선과 법선 offset으로 표현한다.

```text
T(s) = C'(s) / |C'(s)|
N(s) = (-T_y, T_x)
yellow(s)      = C(s)
left_white(s)  = C(s) + w(s) N(s)
right_white(s) = C(s) - w(s) N(s)
```

`w(s)`는 3.5 m를 soft prior로 쓰되 관측에 따라 제한된 범위에서 천천히 변하게 한다. 최적화는
model-grid confidence를 hard threshold로만 버리지 않는 robust weighted data term, 접선
평행성, 폭·폭 변화, 곡률 및 이전 frame state 항을 결합한다. 이전 상태는 learned optical
flow 모델을 추가하지 않고 Ego/IMU로 현재 `base_link`에 보상한 뒤 사용한다. 경계 상태는
`observed`, `structure_inferred`, `temporal_predicted`, `invalid`로 구분하고 예측 형상은
약 0.2~0.3초 후 폐기한다. home/opposing 의미는 계속 판제가 정한다.

연구 근거는 다음처럼 제한적으로 차용한다.

- LaneCPP: continuous spline의 평행성·노면·곡률 physical prior
- Differentiable Least-Squares Fitting: confidence-weighted fitting과 regularization
- RVLD: 이전 frame 정보를 현재 frame에 정렬해 재사용하는 원칙; learned PLD는 도입하지 않음
- BézierLaneNet/BSNet: 곡선 sampling loss와 spline 표현 비교; runtime은 국소 수정이 쉬운
  B-spline을 유지
- LaneAF/StructLane: instance 연결·차선 간 구조 관계의 설계 참고; head/attention 모델은
  이번 대회 경로에 추가하지 않음

`external/baselines`의 연구 저장소는 참고 snapshot이며 팀 runtime dependency나 복사
소스로 취급하지 않는다. 특히 LaneAF와 StructLane에는 재사용 가능한 명시 라이선스가 없고,
Differentiable LSQ 구현은 CC BY-NC이므로 팀 코드는 수식과 논문을 근거로 독립 구현한다.

작업은 다음 작은 gate로 나눈다.

1. 런타임 동작을 바꾸지 않고 evidence→track→spline→boundary→adapter 탈락 사유와
   canonical message/Marker flicker를 분리 계측한다.
2. 음영 polygon과 두 link를 version/hash와 함께 기록하고 3경계 map pseudo-GT·overlay를
   만든 뒤 30~60 frame에서 카메라/LiDAR와 검산한다.
3. 기존 independent spline과 weighted shared lane-family를 대표 4개 bag으로 offline
   A/B한다.
4. 대표 정량 gate 뒤 outlier support·false bridge를 검수하고 `observed`,
   `structure_inferred`, `invalid`를 sample 구간별로 보수화한다. **대표 검수 완료.** 통과한
   후보만 12-run+4 maneuver 전체 matrix로 확장한다.
5. 전체 A/B가 통과한 뒤에만 ego-motion temporal state와 짧은 구조 보간을 runtime에 이식한다.
6. 거리별 metric error와 false continuation을 재평가해 관측/추정 상태별 유효 x 범위를
   확정한다.
7. 이 후처리로 해결되지 않는 원거리 yellow evidence 부족이 확인될 때만 음영 데이터
   fine-tuning이나 lane 전용 모델 비교를 연다.

첫 gate의 산출물은 rejection-reason counter, stable marker 규칙, `shadow_zone` 설정,
map pseudo-GT 생성기 설계와 baseline audit이다. Marker를 보기 좋게 유지하는 것과 실제
canonical 경계를 보간하는 일은 분리하며, 무관측을 정상 관측처럼 숨기지 않는다.

**2026-09-13 첫 gate 구현 결과:** `/perception/lane/pipeline_audit`가 model pixel부터
temporal boundary까지 frame별 outcome/rejection을 발행하고 result profile이 이를 선택적으로
기록한다. producer와 C++ adapter의 debug Marker는 `DELETEALL` 대신 사라진 ID만 명시적으로
삭제한다. 이 변경은 canonical boundary를 보간하지 않는다. 약 50초 live smoke에서 typed
boundary는 약 8.4Hz로 유지됐고, smoothing spline이 evidence 끝점을 정확히 보간하지 않아
valid-x와 공개 polyline이 드물게 불일치하던 오류도 양 범위의 교집합을 공개하도록 수정했다.
종료 시 inference worker의 closed-topic publish 경고도 제거했다.

`shadow_zone_kcity2025.yaml`과 생성기로 만든 최초 map pseudo-GT는 yellow 115.71m,
0.5m 간격 231 point이며 두 link 중심선 간격 p05/median/p95는
`3.490/3.511/3.544m`다. 최초 lateral-only 보정은 road center `-0.18m`, 양쪽 폭
`3.47m`를 제안했으나, 가까운 곳은 맞고 거리에 비례해 세 경계가 같은 방향으로 밀리는
잔차가 남았다. 2026-09-12 Sunny/Foggy 12-run의 Intensity LiDAR 도색점 8,857개로 heading
후보마다 boundary offset을 재추정한 joint search 결과 `ego_heading_offset=-1.25°`, road
center 약 `0.00m`, positive/negative 폭 `3.48/3.52m`가 적합했다. 각도 보정은 음영구간
map pseudo-GT의 map→base 변환에만 적용하며 차량 heading이나 camera extrinsic을 전역
변경하지 않는다.

동일 Sunny 13시 clear bag의 camera/LiDAR 각 48 frame을 최종 재검산한 결과 세 curve의
front 투영 가용성은 48/48이고, LiDAR intensity≥40 비율은 경계 segment 0.2m 이내
`18.749%→37.854%`, 0.6~1.2m 대조구간은 `0.000%`였다. LiDAR 3D→RGB 직접 재투영 p95가
최대 `0.43px`인 점도 원거리 잔차의 주원인이 카메라 투영이 아니라 map/Ego heading
정렬임을 지지한다. calibration set과 별개인
2026-09-13 maneuver RGB 및 live 3-camera raw-BEV가 최종 human holdout이다.
자동 정합 gate는 통과했지만 주행 검수가 남았으므로 상태는 계속
`map_pseudo_gt`, `deployment_allowed=false`다. 기존 12-run+4-maneuver 결과는 새 schema로
재집계해 16/16 기술 gate 통과를 확인했으며, 과거 result에는 신규 frame audit 토픽이 없어
탈락 사유 baseline은 다음 instrumented replay에서 확정한다. 실행법과 artifact는
[`drivable_bev` README](../../../src/perception/drivable_bev/README.md#2026-09-13-음영구간-진단map-pseudo-gt-gate)를
따른다.

**2026-09-14 representative 진단 replay:** clear Sunny/Foggy, lateral sweep Sunny,
avoid-return Foggy의 대표 4개 raw를 새 pipeline-audit result로 재생했다. 총 1,247 frame에서
기술/기록 gate는 모두 통과했고 boundary와 frame audit은 약 `9.57~10.25Hz`였다. clear
3-boundary complete는 `82.57~88.47%`, lateral sweep `74.05%`, avoid-return `83.50%`였다.
불완전 또는 yellow 누락 frame은 모두 개별 spline의 residual gate와 동시에 발생했다.
confidence threshold 미달은 소수이고 association 후보 rejection은 정상 frame에도 많아,
threshold 완화 대신 shared center spline+normal offset의 offline A/B 대상으로 선정했다.

**2026-09-14 shared-family offline A/B:** 같은 대표 조건을 raw부터 다시 재생한 유효
1,255 frame과 공지 음영 polygon 내부 508 frame에서, 공통 cubic B-spline+normal offset은
ODD 내 3경계 complete를 `78.74→100.00%`로 높였다. map p95는
`0.3761→0.3775m`로 유지됐고 0~10/10~15/15~20m의 p95는 모두 개선됐다. 좌측 경계의
map-compensated reference-y jitter p95도 `0.0303→0.0213m`로 감소했다. `3.48/3.52m`는 hard
폭이 아니라 관측으로 조정되는 soft prior다.

대표 정량 gate 뒤 metric residual 기반 support 보수화도 완료했다. fitted role에서 0.35m
이내이고 0.5m 이상 이어진 evidence만 `observed`로 인정하고, 두 역할 이상의 local support가
없는 단일 역할 꼬리와 0.75m보다 긴 family support 공백은 `invalid`로 잘랐다. 대표 1,200
frame에서 residual 0.50m 이상인 75 frame 모두 outlier를 포함했고, 고잔차 overlay 54장의
대표 검수에서는 명백한 횡단 false bridge가 보이지 않았다. 구조 추정 point의 map 불일치도
0.50m 이내였지만 map은 pseudo-GT이므로 review signal로만 사용한다. 12+4 전체 matrix
4,075 frame 확대 결과 ODD complete는
`84.38→100.00%`였다. 서로 다른 출력 표본의 pooled map p95는 `0.4013→0.4077m`지만,
같은 run·frame·role을 run별 층화한 p95 변화는 16개 run 모두 개선됐고 중앙값 `-0.0582m`,
전체/거리 bin 최악 악화 `+0.0193m`로 확대 gate를 통과했다. maneuver 집합에서는 map p95가
약 10cm 개선됐다. shared state는 기존 public boundary와 분리한 debug runtime sidecar로
이식했고 lateral-sweep bag smoke에서 marker/diagnostics 약 8.4Hz, rqt 영상 5Hz를 확인했다.
현재 runtime에는 `observed/structure_inferred/invalid`만 적용했고 temporal state는 구현하지
않았다. 정본 수치와 재현 경로는 `drivable_bev/README.md`의 shared lane-family 절을 따른다.

직접 RViz 검수에서 shared family가 안정적으로 유지된다는 판단에 따라 temporal 보상과 IMU
자세 보정은 조건부 후속으로 내렸다. 현재 runtime 기준선은 `observed`를 정상 후보,
`structure_inferred`를 confidence가 강등된 제한 후보로 사용하고 `invalid`는 시각적으로 맞아
보여도 소비하지 않는다.

2026-09-14에는 straddle·lateral-sweep·Sunny/Foggy 회피/복귀 4개 raw bag을 격리 재생해
`1,126` shared frame을 result bag으로 고정했다. family accepted rate는 `100%`, 최장 reject
연속은 0 frame이며 좌·yellow·우 역할은 모든 frame에서 observed 역할을 유지했다. 곡선 sample은
`observed/structure_inferred/invalid=66.98/10.20/22.82%`였고 invalid는 usable에서 제외된다.
shared fit p50/p95는 `39.68/51.00ms`, run별 출력률은 `8.857~9.277Hz`, runtime error는
0회였다. geometry/state 기준선은 통과했으나 두 run이 9Hz 목표를 근소하게 놓쳐 production
승격은 보류한다. 이때 ROS graph는 11312로 분리했지만 기존 11311 live perception을 유지해
연산 자원은 공유했으므로 지연·출력률은 보수적인 동시부하 기준이다. production latency는
standalone 조건에서 재측정한다.

다음 우선순위는 동일 geometry 비회귀 조건으로 fitting 연산을 최적화한 뒤, temporal 보상 없이
판제 closed-loop를 먼저 검증하는 것이다. 판제가 전체-family dropout을 실제 문제로 확인할
때만 0.2~0.3초의 `temporal_predicted`를 다시 연다. 정본 수치와 lineage는
`artifacts/perception_eval/analysis/twinlite_v6_shared_runtime_baseline_20260914/`를 따른다.

같은 4개 raw bag을 기존 perception/GUI 공존부하 없이 별도 11312 pipeline에서 다시 측정했다.
영상·Marker subscriber를 제거한 `shadow_runtime_profile_result`로 총 1,264 frame을 기록했고,
shared diagnostics 출력률은 `9.762~10.298Hz`로 4개 run 모두 9Hz gate를 통과했다. shared
fit p50/p95는 `28.56/35.37ms`, 전체 profiled body p50/p95는 `64.64/78.53ms`였다. 공존부하
기준보다 shared fit p95는 `30.6%`, 각 run 종료 시 누적 rolling 전체 처리 p95는
`31.7~39.2%` 낮았다. 이는 fitting 수식이나 threshold 개선이 아니라 측정 환경 분리 효과다.

family accepted `100%`, runtime error 0회, sample 상태
`observed/structure_inferred/invalid=66.95/10.24/22.80%`, 좌·우 폭 p50
`3.522/3.536m`, inlier residual p95 분포 p95 `0.293m`로 geometry/state는 비회귀했다.
따라서 standalone 개발 latency gate는 통과했지만 shared 출력은 계속
`deployment_allowed=false`인 debug sidecar다. 다음 구현은 projection audit/extract 중복
순회 제거를 우선하고, shared fit 내부를 세분 프로파일링한다. independent spline 제거는
typed shared public 계약과 판제 closed-loop gate 뒤에 판단한다. 정본은
`artifacts/perception_eval/analysis/twinlite_v6_shared_runtime_standalone_profile_20260914/`다.

### Stage C4 — 조건별 live gate

- 규정 조건 Sunny/Foggy × 11am/1pm/3pm
- 5pm은 규정 외 강건성 조건으로 별도 표기
- 직선, 곡선, 교차로, 횡단보도, 장애물과 GPS 음영구간
- front-only와 three-view, perception 단독과 LiDAR 동시 실행 비교
- 새 독립 Capture/test run으로 정량 평가

센서 native 해상도와 주기는 현재 v2 기준을 유지한다. 먼저 direct 투영, BEV 격자와
camera-to-vector 기준선을 완성하고, 그 이후 같은 정확도·지연 기준에서 부하를 줄이는
10/15/20 Hz 및 해상도 통제 실험을 한다. 모델 변경은 live 또는 독립 test에서 front lane
품질이나 full-stack latency가 부족한 경우에만 같은 조건으로 비교한다.

급출발·급제동에서 chassis dive/squat로 pitch가 변해 지면 투영이 흔들리는 현상은 현재
정적 calibration을 임의 튜닝해 숨기지 않는다. 후속 개선은 camera+IMU+Ego source-time bag,
카메라 stamp에 보간한 IMU quaternion, neutral pose 대비 빠른 pitch/roll 성분을 사용한
프레임별 extrinsic/homography 보정을 1차 후보로 둔다. 느린 도로 경사와 빠른 차체 자세
변화를 분리하고 lateral jitter, lane width, MGeo 정합 오차로 static/dynamic 방식을
비교한다. 차선·소실점 기반 online self-calibration은 IMU 보정의 보조 후보로 둔다.
관련 연구 후보는 [WS-3D-Lane](https://arxiv.org/abs/2209.11523)이다. 실측 영향이 목표
허용오차보다 작을 때만 보정을 생략하며, 이번 lane-vector PR에서는 구현하지 않는다.

## 8. 공용 rosbag의 범위

**2026-09-09 구현 완료:** 공용 recorder는 새 직렬화 포맷을 만들지 않고 ROS1의 검증된
`rosbag record`를 allowlist profile과 metadata lifecycle로 감싼다. recorder는 추론 노드와
별도 프로세스로 실행하며 기록 중 resample이나 timestamp 정렬을 하지 않는다. message
header stamp와 bag 기록 시각을 보존하고 정렬은 offline auditor/evaluator가 담당한다.
실행법과 profile 계약은
[rosbag 재현 평가 가이드](../../operations/rosbag-evaluation.md)를 정본으로 삼는다.

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
| `lidar_dev_gt_udp` | 종료된 ID 5/11 privileged transport 실험 재현용 |
| `dev_oracle_udp` | 종료된 privileged UDP oracle 실험 재현용 |
| `camera_semantic_result` | replay 추론 결과와 구조화 diagnostics |

Real Time UDP bag은 실제 비동기 지연·drop·jitter 분석 기준이다. 같은 bag을 재생하면 모델
A/B 입력은 통제할 수 있지만 MORAI→UDP bridge 이전의 packet 손실까지 재현하지는 못하므로
최종 선택 후 live Real Time 검증을 다시 수행한다.

2026-09-12에는 GPS 음영 왕복 2차로를 대상으로 `shadow_clear`와
`shadow_static_avoid_v1` 각각 Sunny/Foggy×11/13/15시의 wheel raw bag 12개를 확보했다.
Cloudy·중복·중단·position reset run은 삭제하지 않고 rejected로 분리했으며, 정본 선택은
`data/rosbags/audits/shadow_dataset_20260912/selection.yaml`에 둔다. 이 matrix에서는
색상 표식 전체를 곧바로 판제 차선으로 쓰지 않고, 종방향 track 단계에서 노란 중앙 경계와
양쪽 흰 outer boundary를 선별한 뒤 stable local ID와 왼쪽→오른쪽 순서를 제공한다.
`home/opposing` 의미 부여는 판제 책임으로 남긴다. 장애물 없는 6개로 파라미터를 정하고
정적 장애물 6개를 회피·복귀 강건성 검증에 사용한 뒤 provisional handoff를 결정한다.

12-run replay에서 source timestamp가 run 사이에 역행할 때 외부 pipeline의 throttle이
멈추는 문제를 발견해, 100 ms 이상 역행을 새 replay/simulator epoch로 처리하도록
model-grid와 BEV를 보정했다. 15 Hz model-grid cap의 최종 v3는 모든 기술 품질 gate를
통과했고 boundary actual rate `9.56~10.43 Hz`, fused BEV `9.55~10.07 Hz`, vectorization
p95 `43.26~46.21 ms`였다. 장애물 없는 1,572 frame에서 yellow/3-boundary complete는
`98.03/82.95%`, 정적 장애물 1,201 frame에서는 `96.25/71.27%`였다. resolved role 중복과
좌우 순서 오류는 0건이지만 정적 장애물에서 right outer 가용성이 `77.52%`까지 낮아졌다.
따라서 camera boundary는 음영구간 판제 interface 개발용 provisional 관측으로 먼저
제공한다. `/perception/lane/boundary_observations`
(`asmc_msgs/LaneBoundaryObservationArray`)에 N개 metric 경계, 색상, 왼쪽→오른쪽 순서,
short-term ID와 품질을 담았고 home/opposing/절대 차로 번호는 넣지 않았다. 누락과 partial을
정상 상태로 처리하고 세 경계를 항상 가정하거나 단독 steering 입력으로 쓰지 않는다.
판제 측 계약 검토를 위해 독립 C++ `lane_boundary_adapter` pilot도 추가했다. 이 pilot은
`base_link` 경계의 frame/ODD/algorithm/freshness/completeness를 검사하고 인접 경계 사이의
metric strip과 중심선을 debug Marker/diagnostics로만 만든다. y=0을 포함한 strip 표시는
기하학적 vehicle projection일 뿐 home/opposing 판정이 아니며, Behavior/Frenet/MPC와 UDP
명령에는 아직 연결하지 않는다. 실행·실패 안전 기준은
[`lane_boundary_adapter` README](../../../src/planning_control/lane_boundary_adapter/README.md)를
따른다.
단일 Foggy 11시 typed smoke bag의 1배속 consumer 검증에서는 239개 수신 frame 중 196개
(`82.0%`)가 usable strip이었고, 나머지는 `order_invalid` 5개와 보수적 pair gate를 통과하지
못한 `no_usable_strip` 38개였다. 계약 오류는 없었다. 이 결과는 연결 smoke일 뿐 12-run
정확도 기준은 아니며, 기존 v3 result bag에는 typed 토픽이 없어 최신 producer로 raw replay를
다시 수행해야 전체 consumer audit가 가능하다.
정본 수치와 lineage는
[`drivable_bev` README](../../../src/perception/drivable_bev/README.md#2026-09-12-음영구간-replay-결과)를
따른다.

2026-09-13에는 중앙선 straddle, lateral sweep, 정적 장애물 회피·복귀 Sunny/Foggy 4개
raw bag을 추가했다. 기존 출력은 모든 곡선이 고정 전방 5m 지점을 지나야만 좌우 순서를
유효하게 해 차로 이탈 중 `order_invalid`가 run당 17~212 frame 발생했다. 실제로는 네
run의 모든 frame에서 경계 곡선들이 공유하는 다른 종방향 구간이 있었다. producer를
‘공통 support 안에서 선호 5m에 가장 가까운 지점’ 기준으로 바꾼 뒤 order-invalid는 모두
0이 됐다. 차량 lateral motion을 예측하는 최대 8-frame 재획득 tracker도 적용해 straddle,
lateral sweep, Sunny 회피·복귀의 stable ID 수를 각각 `5→4`, `12→8`, `10→4`로 줄였다.

실제 white-yellow 관측이 없는 frame을 정상 인식으로 포장하지 않는다. 다만 음영 왕복
2차로에서 yellow만 잠시 누락되고 양쪽 white outer가 왕복 2차로 폭·공통 길이·폭 변화
gate를 만족할 때, read-only planning adapter가 두 lane strip의 임시 중앙 경계를
`degraded_inferred_center` WARN으로 계산한다. perception 메시지는 관측 경계만 유지한다.
최종 4-run adapter replay는 1,258 message 중 정상 1,090(86.65%), degraded 131(10.41%),
무가용 37(2.94%)로 usable 97.06%였고 contract/order 오류는 0이었다. 이는 제어 안전성이나
metric accuracy 합격률이 아니라 판제 인터페이스 연속성 기준선이다. threshold 0.45 실험은
개선이 없어 기존 white/yellow 0.50을 유지했다. 세부 결과는
[`drivable_bev` maneuver replay](../../../src/perception/drivable_bev/README.md#2026-09-13-차로-이탈회피-maneuver-replay)와
[`lane_boundary_adapter` README](../../../src/planning_control/lane_boundary_adapter/README.md#2026-09-13-회피차로-이탈-replay)를
따른다.

### 8.1 학습용 Capture와 종료된 Streaming GT 조사

카메라 단일-frame 학습은 Real Time + Capture 1 Hz를 기준으로 둔다. 2026-09-09에
2/5/10 Hz 요청을 시험했지만 실제 저장은 약 1.4~1.7 Hz에 포화됐고 요청마다 시뮬레이션이
약 0.5초 정지했다. 따라서 Capture는 학습 GT, Real Time Connect+rosbag은 temporal 평가로
분리한다. 수동 휠 주행의 지연·drop 측정에도 Capture를 함께 켜지 않는다.

데이터 목적별 정본은 세 갈래로 고정한다.

| 목적 | 경로 | 해석 |
|---|---|---|
| 현재 단일-frame segmentation 학습 | v2 + Real Time + Capture 1 Hz | RGB와 Capture oracle을 같은 trigger로 확보 |
| 고주기 RGB–Semantic sequence 학습 후보 | 최소 RGB+Semantic 센서셋 + Simulation Time Sensor Record | 10 Hz pilot과 pairing 검증 후 별도 dataset version으로 채택 |
| 실제 추론·판제 검증 | v2 + Real Time UDP + raw rosbag | 실제 sensor skew, bridge, queue와 연산 지연을 포함한 운용 정본 |

v2 Sensor Record 실측에서 갱신된 것은 `CAMERA_1~4`, `LIDAR_5`, `GPS_6`, `IMU_7`뿐이며
Semantic/Instance GT 디렉터리는 갱신되지 않았다. Sensor Record는 장착 센서만 기록하므로
v2만으로 고주기 supervised GT를 얻었다고 간주하지 않는다. Simulation Time + Sensor Sync는
필요가 생길 때 Sensor Record/offline replay pilot으로 다시 정의하며, 과거 source-time
Streaming GT bag을 현행 정확도 gate로 사용하지 않는다. Real Time UDP가 운용 기준이다.
현재 자세한 실행·저장 경로와 pilot gate는
[data_collection README](../../../src/data_collection/README.md#simulation-time의-sensor-record)를 따른다.

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

UDP GT bridge의 source timestamp와 독립 ID 11 stream은 실험 당시 구현·검증했으나 현재
개발 우선순위에서 제외했다. direct ROS/rosbridge GT와 ROS-vs-UDP 비교도 종료했으며 미병합
구현은 `origin/backup/ros-gt-experimental-20260910`에 보존한다. Capture 학습 GT와 Real Time
UDP raw/result 재현 평가만 현행 경로로 유지한다. 대회 경로는 항상 UDP다.

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

### Shared boundary-family 판제 후보 계약

16-run 확대 평가와 standalone 9Hz gate를 통과한 shared-family geometry를 기존 independent
관측과 별도인 `/perception/lane/boundary_family` typed 후보로 노출한다. 이름은 내부 기법이
아니라 결과 의미를 사용한다. sample별 `observed`와 `structure_inferred`는 각각 정상/강등
usable geometry이고 `invalid`는 소비 금지다. 따라서 다른 두 경계에서 만든 주황색 구간은
명시적으로 낮은 신뢰도로 전달해 순간 단절을 줄이되, 붉은 support 공백을 억지로 잇지 않는다.

이 후보는 `deployment_allowed=false`, `operating_domain=shadow_two_way_v1`이며 home/opposing
의미가 없다. 기존 independent 토픽과 adapter는 유지한다. 다음 gate는 read-only 판제
adapter에서 freshness·ODD·상태를 검사하고 동일 raw bag 및 live closed-loop에서 두 입력을
비교하는 것이다. full-stack 지연을 측정한 뒤 geometry/state를 바꾸지 않는 구현 최적화를
진행하며 Ego-motion/IMU 보상은 실제 dropout 요구가 생길 때까지 보류한다.

2026-09-15에는 이 read-only 판제 adapter를 별도 C++ core/node로 구현했다. producer의
공통 sample index를 그대로 써 인접 경계 두 개의 strip/centerline을 만들고, 네 endpoint가
모두 observed일 때만 정상 segment로 둔다. 하나라도 structure-inferred이면 주황 degraded
segment로 분리하고 국소 confidence를 0.60으로 강등한다. 한 역할 전체가 producer에서 이미
강등된 경우에는 중복 적용하지 않는다. 하나라도 invalid이면 해당 interval을 생략해 앞뒤
segment를 구조적으로 분리한다. producer `valid_x`와 consumer `x[0,15]m`의 교집합 밖도
사용하지 않는다.

Sunny 13시 straddle raw bag의 11312 격리 1배속 smoke에서 398개 candidate가 모두 계약을
통과했고 adapter contract/geometry error는 0이었다. 이 run은 매 frame 일부 inferred sample을
포함해 모두 `degraded_usable`이었으며 MarkerArray는 10초 구간 약 10.1~10.6Hz였다. 마지막
frame은 관측 4개와 inferred 5개의 strip segment로 나뉘고 62개 invalid pair-interval은
출력에서 제외됐다. replay 종료 뒤 receive timeout과 marker clear도 확인했다. 이는 정확도
승격 평가가 아니라 support 보존과 fail-safe 소비 검증이며, node는 계속 debug/diagnostics만
발행하고 `deployment_allowed=false`다.

같은 날 판제 소비 규칙으로 representative maneuver 4개 bag, 1,274 frame을 다시 측정했다.
기존 shared `3경계 complete=100%`와 달리 `x=0~15m` 두 strip의 완전 연속 frame은 69.54%,
coverage 80% 이상은 96.00%였다. coverage p05/p50은 93.92/100.00%이고 80% 미만 최장
연속은 약 2.16초다. 문제는 `3.845~15m`가 아니라 차량 주변 선두 support에 집중됐다.
두 전방 구간의 80% coverage 비율은 각각 97.41/96.62%였지만 차량 envelope는 74.80%였다.
internal gap p95는 전 구간 0m여서 기존 0.75m internal bridge 상향을 첫 개선안으로 쓰지
않는다. 세 strip의 가용 coverage가 같은 것은 producer가 family-wide `sample_valid`를 쓰기
때문이며, 사용자가 본 동시 red 전환과 일치한다. 다음 gate는 12+4 전체 matrix 확대 후,
원거리 extrapolation 없이 강한 전방 support로 차량 근처 leading tail만 완성하는 same-frame
정책 A/B다. 상세 수치는
[`drivable_bev` 판제 가용구간 기준선](../../../src/perception/drivable_bev/README.md#2026-09-15-판제-실제-가용구간-기준선)을
따른다.

12+4 전체 16개 bag, 4,122 frame 확대에서도 현재 adapter audit window `0~15m`는 모든 frame에서 일부 strip이
존재했고 99.27%가 50%, 93.77%가 80% 이상 coverage였다. 반면 `15~20m`는 일부라도
camera support가 있는 frame 93.84%, 50% 이상 79.65%, 80% 이상 62.08%였다. Foggy의
원거리 80% coverage는 53.7%로 가장 약했다. 판제가 lookahead를 갑자기 잃지 않게 원거리
family와 sample 상태를 원거리까지 보존할 가치는 있지만, camera support가 없는 6.16%까지
정상 인지라고 표현할 수는 없다. `0~15m`와 `15~20m`는 현재 adapter/audit 구간이지 고정
control/preview 인터페이스가 아니다. producer는 전체 configured polyline을 내보내고,
consumer는 frame별 연속 trusted interval, 내부 gap과 최대 candidate horizon을 함께 해석하는
방향을 후보로 둔다. invalid geometry는 map/수동 검수를 통과한 경우에만 별도 저권한 상태로
승격하며, 그 전까지 제어 금지이고 fallback은 이전 승인 경계 또는 MGeo가 맡는다.
정본 수치와 lineage는
`twinlite_v6_shadow_planning_usability_full16_v1_20260915/baseline.yaml`에 기록했다.

후속 원인 분류에서는 support를 변경하지 않고 16개 run 4,023 frame의 leading/trailing 위치를
명시적으로 다시 계산했다. 차량 근처 leading tail은 13.94%, `15~20m` far-audit trailing tail은
85.83%, preview 시작 미고정은 6.39%, near/mid 전체 support 축소는 7.56%, 내부 gap은
0.75%였다. 원거리 비율에는 0.26m를 넘는 작은 말단 절단도 포함된다. `0~15m`는 bounded
read-only adapter 구간이고 20m는 현 평가 상한이며 영구 계약이 아니다.
`INVALID` 좌표는 별도 저권한
preview/fallback 연구를 위해 typed family 안에 보존하지만 기본 판제 adapter에서는 계속
분할·제외한다. 판제가 opt-in하더라도 카메라 관측으로 재표기하지 않고 MGeo/이전 승인 경계,
freshness와 별도 권한으로 다뤄야 한다. 상세 기준은
`twinlite_v6_shadow_invalid_support_full16_v1_20260915/baseline.yaml`에 있다.

대표 clear/static-avoid Sunny/Foggy 4개 run에서는 분류별 sparse overlay 44장을 추가로
검수했다. 빨간 unsupported geometry의 위치가 leading/trailing/whole-shrink/internal-gap/
far-band-entry 분류와 일치해 taxonomy 사전검수는 통과했다. 이는 정확도 승격이 아니며,
44장 중 map overlay가 가능했던 14장은 nearest-Ego gate 실패가 아니라 sparse 선택 frame 중
pseudo-GT 음영 ODD 안에 있던 표본이다. 네 run 1,031 frame에는 모두 50ms 이내 nearest Ego가
있었다. 정본은
`twinlite_v6_shadow_invalid_support_overlay_prereview_v1_20260915/baseline.yaml`에 있다.

이어 같은 4개 run에서 camera source stamp에 Ego XY/heading을 보간하고, 음영 ODD 378 frame의
INVALID 좌표를 left/yellow/right 동일 역할 map pseudo-GT와 signed-normal 거리로 비교했다.
1,030/1,031 frame이 보간됐고 1 frame만 nearest fallback이었다. nearest age p95는 `11.62ms`,
보간과 nearest pose 차이는 p95 `0.121m/0.056°`였지만 category-frame map p95 변화의 절댓값
p95는 `0.0125m`에 불과했다. 따라서 source-time 보간은 평가 ambiguity를 줄이지만 원거리
오차의 주원인은 아니다.

차량 근처 leading tail, whole shrink, internal gap의 역할별 frame-p95의 p95는 각각 최대
`0.187/0.177/0.081m`, preview-entry는 `0.200m`, far trailing은 `0.431m`였다. 완화된
`≤0.35/0.35~0.60/>0.60m` review band의 multi-label category-frame 수는 `385/23/0`이며,
이는 pass/fail gate가 아니다. pseudo-GT·투영·비동기 센서 오차를 감안해 빨간 geometry의
후속 저권한 후보 설계 우선순위에만 사용하며 support 상태는 바꾸지 않았다. 정본은
`twinlite_v6_shadow_invalid_map_distance_representative_v1_20260915/baseline.yaml`이다.

이후 2026-09-15 시간 제한 결정으로, 위 측정을 정밀 승인 gate가 아닌 fail-soft 근거로
사용해 candidate v2를 적용했다. family acceptance 조건은 그대로 유지하고, accepted frame의
`x[0,15]m` 안에 남은 unsupported fitted sample만 `STRUCTURE_INFERRED`로 승격한다. 이는
`OBSERVED` 승격, temporal prediction 또는 map 보간이 아니며 15m 이후와 family reject는
계속 invalid다. 알고리즘 ID는
`twinlite_v6_shadow_boundary_family_candidate_v2`이고 판제 adapter는 inferred를 강등
후보로만 소비한다. 제어 연결은 여전히 없으며 live 시각화와 read-only 통합 확인이 다음 gate다.

같은 날 candidate v3에서는 “역할은 여러 개지만 local support 구간만 엇갈려 family가
사라지는” 경우를 별도로 다뤘다. strict-accepted 결과는 그대로 보존하고,
`insufficient_local_multi_role_support` reject만 `x[0,15]m`의 bounded distributed-support
gate로 재검사한다. 역할별 2m, union 6m, internal gap 3m, heading 15도와
residual/outlier/curvature 상한을 모두 통과해야 하며, 다른 역할로 생성된 sample은 계속
`STRUCTURE_INFERRED`다. 한 역할 복원, temporal prediction과 map 보간은 하지 않는다.
합성 staggered 사례는 복구되고 heading 불일치 negative control은 거부됐지만, 기존 12+4
감사 4,023 frame은 전부 strict accepted였으므로 실환경 복구율은 아직 측정하지 않았다.
algorithm ID는 `twinlite_v6_shadow_boundary_family_candidate_v3`이며 targeted high-heading
live/bag 검수가 다음 gate다.

2026-09-15 직접 주행 검수 뒤 candidate v3의 geometry, threshold와 sample-state 계약을
판제 인수용으로 동결했다. 대표 straddle/lateral-sweep/Sunny·Foggy static-avoid 4개 raw
bag을 GUI 없는 격리 pipeline으로 다시 처리한 1,270 frame에서 family accepted 100%,
runtime error 0, run별 출력 `9.381~10.406Hz`였다. 전체 lane body p50/p95는
`76.37/96.96ms`, shared fit은 `31.66/40.88ms`다. 네 bag은 모두 strict path여서 v3
distributed fallback의 실사용 정확도를 과장하지 않는다.

9Hz 개발 gate는 통과했지만 full-stack 여유는 크지 않다. 지금은 파라미터를 바꾸지 않고
인터페이스를 고정하며, 판제 공존 부하에서 9Hz가 깨질 때 legacy independent 경로 opt-in
비활성화, projection 중복 순회 제거, shared-fit 내부 profile 순으로 최적화한다. 인지는
`/perception/lane/boundary_family`, `/perception/bev/camera/drivable_probability`와
`/perception/bev/camera/coverage`까지만 제공한다. 음영 ODD on/off, home/opposing 의미,
LiDAR occupancy 결합, safe corridor, fallback과 closed-loop 검증은 판제·퓨전 파트가 소유한다.
세부 계약과 bag lineage는
[음영구간 카메라 인지 v3 판제 인수 가이드](../../handoffs/perception/shadow-camera.md)를 따른다.

## 10. 제안 로드맵과 결정 gate

| 순서 | 산출물 | 다음 단계 진입 조건 | 상태 |
|---:|---|---|---|
| 1 | 센서셋·규정·수집 모드·차량 좌표계 정본 | JSON/문서 검증과 변경 범위 확인 | 완료 |
| 2 | Real Time UDP raw/result bag | lifecycle, replay/audit와 v3 성능 bag | **구현·운용 중** |
| 3 | Streaming/ROS GT transport 조사 | rate/schema와 실패 원인 기록 | **종료·백업 브랜치 보존** |
| 4 | 센서 calibration과 공통 BEV 기준 | camera-only fused pilot 완료; provisional debug ROI를 vector 재평가·GPS lever arm·map overlay 후 확정 | camera 부분 진행 중 |
| 5 | 3-camera BEV와 lane 구조화 | 판제 소유 feature flag·closed-loop·full-stack profile | **v3 interface/parameter 동결 및 성능 bag 완료; 제어 미연결 handoff** |
| 6 | Current HD맵 보정과 crop/source policy | 원태 direct anchor 기반 vector/raster 후보에서 구간별 주행 검수 후 선택 승격; 이후 정상 GPS/Blackout 정합 | **정본 후보·특수구간 surface bake·1Hz review logger 완료, operational=false** |
| 7 | LiDAR association과 판제 인터페이스 | ego/adjacent/unknown, occupancy와 ownership 합의 | 공동 |
| 8 | 대회판 이식·추가 데이터·장기 재학습 | competition-only held-out 개선과 full rehearsal 통과 | 10월 후보 |

## 11. 팀 합의가 필요한 열린 질문

다음 항목은 문서만으로 확정하지 않고 pilot 결과를 보고 회의/PR에서 결정한다.

1. 실제 map-camera overlay에 따른 공통 BEV 전·후·좌·우 범위와 m/cell
2. GPS blackout에서 허용 status 필드와 localization source 우선순위
3. map-camera 정합 결과를 localization 보정에 직접 넣을지, 평가용으로 먼저 제한할지
4. LiDAR object association의 `outside/unknown` 보존 및 collision override 규칙
5. LiDAR occupancy 차감과 `safe_drivable`은 판제·퓨전에서 소유하되 두 파트 사이 세부 경계를 어디에 둘지
6. planning이 필요한 raster/vector 필드와 update rate
7. competition branch 이름, 유지 기간과 main backport 규칙
8. 대회판 GT Capture 가능 범위와 train/val/test 분할
9. 공용 Real Time raw/result bag의 담당과 저장 용량/보존 정책

변경 제안은 이 문서의 전제를 조용히 덮어쓰지 말고, 근거가 된 실증 run, metric 또는
인터페이스 소비 요구와 함께 반영한다.
