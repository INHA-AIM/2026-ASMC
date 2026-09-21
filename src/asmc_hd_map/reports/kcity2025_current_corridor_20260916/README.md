# K-City 2025 current Node/Link 명목 기준선

이 디렉터리는 `build_current_corridor.py`가 만든 **1단계 후보 산출물**이다. 현재
K-City 2025 link/node와 주최측 전역경로만 사용했다. 원태 관측선과 구 KATRI MGeo는
입력하지 않았으며, 어떤 feature도 판제 운영용으로 승격하지 않았다.

## 결과 요약

- 주최측 전역경로와 현재 route link 39개가 정확히 일치한다.
  - 폐루프 길이: `2184.612m`
  - 위치 p95/최대 오차: `0.0m`
- 대회 corridor에는 current link 70개가 포함됐다.
  - 공식 경로: 39개
  - 고주로 평행 link: 32개
  - 진행 방향과 같은 link: 30개
  - 반대 방향 link: 2개 (`A2256W000543`, `A2256W000546`)
  - 음영구간: `A2256W000126` / 반대편 `A2256W000131`
  - 정적 장애물 구간: `A2256W000222` / 반대편 `A2256W000221`
- 고주로 후보 중 공식 경로나 별도 미션 link가 아닌 25개는 link ID와 topology를
  보존하면서 실제 고주로 주변 geometry만 잘라냈다. 짧게 스친 link의 전체 도로가
  route-only map에 섞이지 않도록 하기 위한 처리다.
- 명목 차선 경계는 85개다.
  - 경계 생성 대상 link: 53개
  - 인접 link 공통 경계: 21개
  - 합류·분기·비일반 link 등 `review_required`: 42개
- 현재 전체 link의 `width_start/end`는 모두 `3.5/3.5m`다. 따라서 이 값은 실제
  도색 폭 GT가 아니라 `nominal_width`다.
- 공식 경로에 연결된 `on_stop_line` node 13개를 빠짐없이 찾았고, 방향과 인접 차로
  폭으로 명목 정지선 후보 13개를 만들었다. 실제 도색 위치 검증 전 상태다.

## 파일

| 파일 | 용도 |
|---|---|
| `summary.json` | source hash, 개수, 검증 통계 |
| `corridor_links.csv` | link별 역할·route station·topology·geometry scope |
| `nominal_lane_boundaries.csv` | 명목 경계와 공통 경계 중복 제거 결과 |
| `route_stopline_candidates.csv` | 경로상 13개 정지선 후보와 신호 연결 |
| `nominal_candidate_map.json` | provenance를 포함한 canonical vector 후보 |

## 해석 제한

- `link_centerline`만 current topology/geometry의 정본이다.
- `lane_boundary`는 중심선을 link 폭만큼 법선 offset한 후보이므로 실제 흰색·노란색
  도색선이나 구간별 폭을 보장하지 않는다.
- 짧은 link와 교차로 connector에는 자동 차선 경계를 만들지 않았다.
- `stopline`은 존재·route 연결 후보이며, 실제 도색 선분 위치는 구맵과 RGB-BEV로
  검산해야 한다.
- 모든 feature는 `candidate`, 전체 map은 `operational=false`다.

다음 단계에서는 이 기준선을 유지한 채 원태 관측선의 법선 잔차로 폭과 경계를
보정한다. 변경구간을 제외한 안정 구간에서만 origin 정합된 구맵의 색상·형태·누락
geometry를 보조 입력으로 사용한다.
