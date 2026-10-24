# K-City 2025 주행가능영역 — kcity2025_reviewed_high_speed_adjacent_drivable_v7

> **상태:** 현재 reviewed 차선 SHA와 묶어 생성·검증하는 P2-B product.

## 선택 정책

- 전역경로 39개 링크와 같은 방향의 사용 가능 동일-road 차로를 기본 Lane으로 채택했다.
- Lane과 추가 surface patch를 합친 vector polygon 집합이 geometry 정본이다.
- polygon 끝은 round buffer가 아니라 endpoint node의 법선에서 flat cap으로 절단한다.
- 일반도로는 reviewed paint의 국소 횡거리로 44개 Lane의 경계를 보정했다.
- 인접 차로와 음영구간 A126/A131의 20쌍을 corridor 대상으로 선택했다.
- 교차로의 거리/반경 확장을 폐기하고 reviewed link set만 추가했다.
- 동적 장애물 교차로는 검토된 4개 anchor의 directed start/end node-pair closure 8개 link를 사용한다.
- 정적·동적 교차로는 선택 link envelope 안의 legacy H5 road만 gap patch로 사용한다.
- A134 로터리는 2.0 m 확장 link envelope와 A151 접속부 안의 legacy H5 포장면을 사용하고 중앙 섬은 비주행으로 보존한다.
- 일반 교차로는 전역경로 link만, 정적·동적 장애물 교차로는 검토된 회피 link만 포함한다.
- 음영 A131 및 명시적으로 검토한 인접 lane group만 추가하며 다른 도로로 자동 확장하지 않는다.
- 인접 검토 그룹: `gps_shadow_opposite_lane, high_speed_preentry_029_175_406`. H5 사용은 해당 lane envelope 안으로 제한한다.
- `OccupancyGrid`는 vector product를 0.20 m/px로 rasterize한 파생 표현이다.

## 결과

- route lanes: `39`, selected lanes: `73` (extra same-direction `16`)
- intersection support links: `22`, vector areas: `115`
- base lane areas: `73`, adjacent corridor fills: `20`, intersection gap patches: `14`
- reviewed-boundary refinement: `44` lanes / `80` sides
- explicit shadow opposite lane: `A2256W000131` beside route `A2256W000126`
- reviewed non-route intersection lanes: `15`
- static allowlist: `A2256W000100, A2256W000111, A2256W000221, A2256W000222, A2256W000226, A2256W000227, A2256W000746`
- dynamic allowlist: `A2256W000066, A2256W000087, A2256W000206, A2256W000211, A2256W000599, A2256W000636, A2256W001124, A2256W001125`
- roundabout extension: `A2256W000134`
- roundabout surface stitch: `A2256W000151` (envelope reference only; not a new lane)
- legacy road: `89834.44 m²`, compact road: `15730.84 m²` (`17.51%`)
- reviewed geometry SHA: `521b1fde181a0c10ce71696f7f05010138c041dd067ad531dce23ff5da6ad24f`
- product SHA: `a04e0fd1de642eb7cf088c1d1d98b422ad6a225564490b10f680b3a23359349b`
- grid SHA: `83d5ba63c5bab2c0d802c7c018d4ef3787be79a5e89060d78e6b2111199afda7`

## 소비 규칙

- 판제의 geometry 정본은 `/hd_map/global/lanes`와 `/hd_map/global/drivable_areas`다.
- `/hd_map/global/drivable_grid`는 빠른 lookup용 0.20 m 파생 raster다.
- `intersection_guarded=true` 영역은 allowlist 기반 공간이며 미션 policy 밖 이탈 허가가 아니다.
