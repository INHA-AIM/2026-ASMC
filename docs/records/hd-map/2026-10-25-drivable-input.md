# K-City 2025 P2-A lane topology·drivable-area 입력 감사

> **상태:** P2-A 완료 — 입력과 보호구간만 동결, geometry/publisher 변경 없음

## 결론

- 전역경로 39개 링크는 위상적으로 닫혀 있고 총 길이는 `2184.612 m`다.
- 기존 주행가능영역은 진짜 vector road mesh가 아니라 Node/Link 중심선 buffer에 검수 보정을 더한 H5 raster다.
- 기존 road 면적은 `89834.44 m²`, 공식 checkpoint 중심은 `15/15` 모두 road 안에 있다.
- 보호구간은 `984.488 m`, 차선 기반 clip 후보는 `1200.123 m`이며 합계가 전역경로를 정확히 한 번 덮는다.
- stage14 geometry hash는 `521b1fde181a0c10ce71696f7f05010138c041dd067ad531dce23ff5da6ad24f`로 유지됐다.

## 분류 규칙

- MGeo `link_type=1`은 시작부 최대 30 m를 교차로 보호구간으로 둔다.
- MGeo `link_type=3`인 A153/A451은 합류·분기 및 톨게이트 섬 때문에 링크 전체를 보호한다.
- 노드의 incoming/outgoing 수가 1:1이 아니면 해당 끝점 5 m를 보호한다.
- 좌우 reviewed marking 동시 coverage가 80% 미만이면 링크 전체를 보호한다.
- clip 후보가 5 m보다 짧으면 불안정한 조각을 만들지 않고 보호구간에 흡수한다.

이 분류는 P2-B에서 적용할 수 있는 후보를 정한 것이지, 아직 road를 잘라낸 결과가 아니다.
물리 도색의 노드 gap도 연결하지 않았으며 `/hd_map/global/drivable_*` 토픽은 계속 비활성 상태다.

## 링크별 판정

| 링크 | route s (m) | 양측 coverage | 판정 | 보호 사유 |
|---|---:|---:|---|---|
| `A2256W000748` | 0.0–72.6 | 100% | `hybrid_guard_and_clip_candidate` | topology_transition_at_end, topology_transition_at_start |
| `A2256W000182` | 72.6–95.0 | 50% | `preserve_prior` | insufficient_bilateral_marking_coverage, intersection_link, topology_transition_at_end, topology_transition_at_start |
| `A2256W000329` | 95.0–160.0 | 100% | `hybrid_guard_and_clip_candidate` | topology_transition_at_end, topology_transition_at_start |
| `A2256W000236` | 160.0–186.3 | 36% | `preserve_prior` | insufficient_bilateral_marking_coverage, intersection_link, topology_transition_at_end, topology_transition_at_start |
| `A2256W000219` | 186.3–208.5 | 100% | `hybrid_guard_and_clip_candidate` | topology_transition_at_end, topology_transition_at_start |
| `A2256W000215` | 208.5–237.4 | 37% | `preserve_prior` | insufficient_bilateral_marking_coverage, intersection_link, topology_transition_at_end, topology_transition_at_start |
| `A2256W000213` | 237.4–315.9 | 100% | `hybrid_guard_and_clip_candidate` | topology_transition_at_end, topology_transition_at_start |
| `A2256W000222` | 315.9–346.7 | 0% | `preserve_prior` | insufficient_bilateral_marking_coverage, intersection_link, topology_transition_at_end, topology_transition_at_start |
| `A2256W000220` | 346.7–365.8 | 100% | `hybrid_guard_and_clip_candidate` | topology_transition_at_end, topology_transition_at_start |
| `A2256W000202` | 365.8–427.9 | 100% | `hybrid_guard_and_clip_candidate` | topology_transition_at_start |
| `A2256W000728` | 427.9–436.8 | 100% | `preserve_prior` | clip_interval_below_minimum, topology_transition_at_end |
| `A2256W000599` | 436.8–462.9 | 43% | `preserve_prior` | insufficient_bilateral_marking_coverage, intersection_link, topology_transition_at_end, topology_transition_at_start |
| `A2256W000207` | 462.9–471.1 | 100% | `preserve_prior` | clip_interval_below_minimum, topology_transition_at_start |
| `A2256W000205` | 471.1–482.8 | 100% | `lane_clip_candidate` | — |
| `A2256W000315` | 482.8–550.6 | 100% | `hybrid_guard_and_clip_candidate` | topology_transition_at_end |
| `A2256W000318` | 550.6–580.9 | 100% | `preserve_prior` | intersection_link, topology_transition_at_end, topology_transition_at_start |
| `A2256W000308` | 580.9–609.9 | 100% | `hybrid_guard_and_clip_candidate` | topology_transition_at_end, topology_transition_at_start |
| `A2256W000083` | 609.9–635.1 | 0% | `preserve_prior` | insufficient_bilateral_marking_coverage, intersection_link, topology_transition_at_end, topology_transition_at_start |
| `A2256W000304` | 635.1–858.5 | 100% | `hybrid_guard_and_clip_candidate` | topology_transition_at_start |
| `A2256W000148` | 858.5–885.1 | 100% | `lane_clip_candidate` | — |
| `A2256W000146` | 885.1–904.0 | 100% | `preserve_prior` | intersection_link, topology_transition_at_end |
| `A2256W000151` | 904.0–914.8 | 100% | `preserve_prior` | intersection_link, topology_transition_at_start |
| `A2256W000866` | 914.8–930.6 | 100% | `hybrid_guard_and_clip_candidate` | topology_transition_at_end |
| `A2256W000054` | 930.6–984.7 | 100% | `hybrid_guard_and_clip_candidate` | intersection_link, topology_transition_at_end, topology_transition_at_start |
| `A2256W000846` | 984.7–1118.7 | 100% | `hybrid_guard_and_clip_candidate` | topology_transition_at_start |
| `A2256W000411` | 1118.7–1286.2 | 76% | `preserve_prior` | insufficient_bilateral_marking_coverage, topology_transition_at_end |
| `A2256W000420` | 1286.2–1368.7 | 100% | `hybrid_guard_and_clip_candidate` | topology_transition_at_end, topology_transition_at_start |
| `A2256W000408` | 1368.7–1518.2 | 0% | `preserve_prior` | insufficient_bilateral_marking_coverage, topology_transition_at_start |
| `A2256W000445` | 1518.2–1593.6 | 36% | `preserve_prior` | insufficient_bilateral_marking_coverage, topology_transition_at_end |
| `A2256W000153` | 1593.6–1741.7 | 81% | `preserve_prior` | merge_diverge_or_toll_transition, topology_transition_at_start |
| `A2256W000451` | 1741.7–1781.6 | 100% | `preserve_prior` | merge_diverge_or_toll_transition |
| `A2256W000446` | 1781.6–1859.6 | 100% | `lane_clip_candidate` | — |
| `A2256W000448` | 1859.6–1877.7 | 100% | `lane_clip_candidate` | — |
| `A2256W000126` | 1877.7–1996.7 | 100% | `lane_clip_candidate` | — |
| `A2256W000128` | 1996.7–2109.8 | 100% | `lane_clip_candidate` | — |
| `A2256W000333` | 2109.8–2121.0 | 100% | `preserve_prior` | intersection_link, topology_transition_at_end |
| `A2256W000154` | 2121.0–2132.2 | 100% | `preserve_prior` | intersection_link, topology_transition_at_start |
| `A2256W000332` | 2132.2–2165.8 | 100% | `hybrid_guard_and_clip_candidate` | topology_transition_at_end |
| `A2256W000751` | 2165.8–2184.6 | 100% | `hybrid_guard_and_clip_candidate` | topology_transition_at_end, topology_transition_at_start |

## 다음 gate

P2-B에서 Lane centerline/polygon과 vector drivable polygon을 생성한다. clip은 이 manifest의
`lane_clip_candidate_intervals`에만 적용하고 `guard_intervals`에서는 기존 H5 road를 보존한다.
checkpoint 포함, 연결성, polygon self-intersection, lane adjacency와 stage14 geometry hash parity를
모두 통과하기 전에는 `link_node`를 삭제하지 않는다.
