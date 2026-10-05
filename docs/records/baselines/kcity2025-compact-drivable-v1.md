# K-City 2025 P2-B v5 A134 로터리·A151 접속부 주행가능영역

> **상태:** 생성·검증 가능한 P2-B product. reviewed 차선 geometry는 변경하지 않음.

## 선택 정책

- 전역경로 39개 링크와 같은 방향의 사용 가능 동일-road 차로를 기본 Lane으로 채택했다.
- Lane과 추가 surface patch를 합친 vector polygon 집합이 geometry 정본이다.
- polygon 끝은 round buffer가 아니라 endpoint node의 법선에서 flat cap으로 절단한다.
- 일반도로는 신뢰 가능한 reviewed paint의 국소 횡거리로 41개 Lane의 경계를 보정했다.
- 선택된 인접 차로 17쌍과 음영구간 A126/A131 사이를 vector corridor로 채웠다.
- 교차로의 거리/반경 확장을 폐기하고 reviewed link set만 추가했다.
- 동적 장애물 교차로는 검토된 4개 anchor의 directed start/end node-pair closure 8개 link를 사용한다.
- 정적·동적 교차로는 선택 link envelope 안의 legacy H5 road만 gap patch로 사용한다.
- A134 로터리는 2.0 m 확장 link envelope와 A151 접속부 안의 legacy H5 포장면을 사용하고 중앙 섬은 비주행으로 보존한다.
- 일반 교차로는 전역경로 link만, 정적·동적 장애물 교차로는 검토된 회피 link만 포함한다.
- 반대차로는 음영구간 A131만 명시적으로 포함하고, 그 밖의 대회 외 도로는 제거했다.
- `OccupancyGrid`는 vector product를 0.20 m/px로 rasterize한 파생 표현이다.

## 결과

- route lanes: `39`, selected lanes: `71` (extra same-direction `16`)
- intersection support links: `23`, vector areas: `103`
- base lane areas: `71`, adjacent corridor fills: `19`, intersection gap patches: `13`
- reviewed-boundary refinement: `41` lanes / `74` sides
- explicit shadow opposite lane: `A2256W000131` beside route `A2256W000126`
- reviewed non-route intersection lanes: `16`
- static allowlist: `A2256W000100, A2256W000111, A2256W000221, A2256W000222, A2256W000226, A2256W000227, A2256W000746, A2256W001212`
- dynamic allowlist: `A2256W000066, A2256W000087, A2256W000206, A2256W000211, A2256W000599, A2256W000636, A2256W001124, A2256W001125`
- roundabout extension: `A2256W000134`
- roundabout surface stitch: `A2256W000151` (envelope reference only; not a new lane)
- legacy road: `89834.44 m²`, compact road: `15005.00 m²` (`16.70%`)
- reviewed geometry SHA: `01ec671fe402baf339f8f6bcfb83a97213d560f7311961c5f072a47d654bcfbf`
- product SHA: `8b07b419f472df055ba2ef13753bb18d5c07b3fc8eca07f9de2229ba53c7c049`
- grid SHA: `e4cfe317152a9dc5f7355a7483cacf74c712b49c1ae41d833a7cdcc0171e6e45`

## 소비 규칙

- 판제의 geometry 정본은 `/hd_map/global/lanes`와 `/hd_map/global/drivable_areas`다.
- `/hd_map/global/drivable_grid`는 빠른 lookup용 0.20 m 파생 raster다.
- `intersection_guarded=true` 영역은 allowlist 기반 공간이며 미션 policy 밖 이탈 허가가 아니다.
