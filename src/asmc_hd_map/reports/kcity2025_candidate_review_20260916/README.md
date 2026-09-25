# K-City 2025 HD맵 주행 검수 묶음

> **경로 주의:** 이 보고서의 build 명령은 2026-09-16 검수 환경을 재현하는
> 역사 기록이다. P4-C6에서 camera/BEV/traffic은 `asmc_perception`으로 통합됐으므로
> 새 workspace에서 아래 whitelist를 복사하지 말고 `./scripts/build_ws.sh`와
> [`asmc_perception` README](../../../asmc_perception/README.md)를 사용한다.

현재 K-City 2025 node/link, 원태 직접 관측으로 보정한 후보, origin만 맞춘 구 KATRI
MGeo와 주행 검수 후 만든 v3 hybrid 차선 후보를 한 화면에서 비교한다. 이 묶음은
**review-only** 산출물이며 `operational=false`다.

## 고주로 보존 여부

- 고주로 주변 current link centerline: 32개
  - 공식 경로 방향과 같은 방향: 30개
  - 반대 방향 context: 2개
- 공식 경로에서 좌우 인접 관계로 도달 가능한 차선변경 link: 25개
- 누락된 차선변경 가능 centerline: 0개
- 위 25개 중 원태 관측 보정 interval이 있는 link: 21개
- 보정 interval이 없는 link: `A2256W000179`, `A2256W000424`,
  `A2256W000426`, `A2256W000427`

따라서 공식 전역경로 차로만 남기고 다른 고주로 차로를 제거한 상태가 아니다. 네 link의
경계는 아직 current link의 명목 3.5m 폭 후보이고, 원태 관측으로 보정됐다고 주장하지
않는다. `A2256W000010`은 고주로 crop 안에 3.28m만 남아 1단계 10m 생성 gate를 통과하지
못했으므로 centerline만 있고 명목 경계가 없다. 이 link는 25개 차선변경 closure 밖이지만
legacy 이식 단계에서 별도로 검수한다.

## 구맵과 원태 관측의 일치도

고주로의 `observed_calibrated` interval을 동일 current link에 대응된 구맵 A/B 경계와
0.25m 간격으로 비교했다.

| 항목 | 값 |
|---|---:|
| 비교 가능한 observed interval | 25 |
| observed sample | 14,063 |
| 3m 안에서 대응된 sample | 13,744 (97.73%) |
| 대응 sample 거리 p50 / p95 | 0.105 / 0.515m |
| 대응 sample 중 0.30m 이내 | 87.80% |
| 대응 sample 중 0.75m 이내 | 98.17% |
| legacy 고주로 후보 | 64개 (Tier A 56, Tier B 8) |

이는 고주로가 구맵과 대체로 동일하다는 강한 근거다. 이후 실제 주행 검수에서 이 판단을
확인하여, current node/link가 topology를 계속 소유한 상태로 안정구간 구맵 geometry를
선분 단위로 채택했다. 전역 A/B feature만 복사하지 않고 current corridor와 다시 비교해
Tier D source 8개도 복원했다. 상세 결과는
[`legacy adoption report`](../kcity2025_legacy_adoption_20260916/README.md)에 있다.

기존 `observed_candidate_map.json`은 그대로 보존한다. 새 v3 후보의 흰색/노란색은 안정
구간 구맵과 current 재구성선을 합친 결과다. 원태 관측선은 위치·폭 검산 reference,
magenta 점선은 미채택 비교 layer다. 변경구간과 정지선 기각 내역은
[`hybrid completion report`](../kcity2025_hybrid_completion_20260916/README.md)에 있다.

## 구간별 오버레이

| 구간 | 이미지 |
|---|---|
| 전체 대회 corridor | [competition_corridor.png](overlays/competition_corridor.png) |
| 고주로 진입 | [high_speed_01_entry.png](overlays/high_speed_01_entry.png) |
| 고주로 북측 | [high_speed_02_north.png](overlays/high_speed_02_north.png) |
| 고주로 중북측 | [high_speed_03_middle_north.png](overlays/high_speed_03_middle_north.png) |
| 고주로 중남측 | [high_speed_04_middle_south.png](overlays/high_speed_04_middle_south.png) |
| 고주로 남측 | [high_speed_05_south.png](overlays/high_speed_05_south.png) |
| 고주로 이탈 | [high_speed_06_exit.png](overlays/high_speed_06_exit.png) |
| 음영구간 | [shadow.png](overlays/shadow.png) |
| 정적 장애물 구간 | [static_obstacle.png](overlays/static_obstacle.png) |

색상은 current center cyan, 명목 경계 gray, hybrid 후보 white/yellow, 정지선 red,
주최측 경로 green, 미채택 구맵 reference magenta 점선이다. 원태 관측은 검산 reference다.

정량 inventory는 [high_speed_inventory.csv](high_speed_inventory.csv), 전체 source hash와
구간별 layer 수는 [summary.json](summary.json)에 있다.

## MORAI 주행 검수

UDP bridge가 `/gps`와 `/imu`를 발행하는 Noetic 컨테이너에서 패키지를 한 번 빌드한 뒤
실행한다. 대회판에서 `/Ego_topic.position`이 `(0,0)`으로 파싱된 실측 사례가 있어 이를
map pose로 사용하지 않는다. GPS를 candidate map의 UTM52N 원점으로 변환하고 IMU yaw와
GPS 장착점 `(+0.35,0)m` lever-arm 보정을 적용한다.

```bash
cd /root/ws
source /opt/ros/noetic/setup.bash
catkin_make -DCATKIN_WHITELIST_PACKAGES="asmc_msgs;camera_semantic_perception;data_collection;drivable_bev;lane_boundary_adapter;morai_msgs;asmc_hd_map"
source devel/setup.bash
roslaunch asmc_hd_map candidate_map_overlay.launch repo_root:=/root/ws
```

RViz 기본 화면은 current center, reviewed hybrid boundary, route와 검수 후 남은 stopline
10개를 표시한다. 하위 호환 때문에 Marker topic 이름에는 `adopted_legacy`가 남아 있지만
내용은 stable legacy와 current reconstruction을 모두 포함한다.
왼쪽 Displays에서 다음 비교 layer를 필요할 때 켠다.

- `Current nominal boundaries`: 일률 폭 기반 fallback reference
- `Observed calibration`: 원태 위치 검산 reference
- `Changed-region legacy`: 미채택 전환구간 후보
- `Legacy A/B reference`: origin-only 구맵. 운영 geometry가 아님
- `Raw source observations`: 원태 원관측과 gate 상태
- `High-speed link labels`: current link ID와 lane 번호

별도 `map → base_link` TF는 필요 없다. 노드가 GPS/IMU pose로 주변 80m를 직접
`base_link`로 바꾼다. 진단용으로 예전 Ego 경로가 꼭 필요할 때만
`pose_source:=ego_status`를 명시한다. 모든 토픽은 검수용이며 판제가 소비해서는 안 된다.
`Ego footprint` layer의 흰 점이 후륜축 중앙 원점이고, 빨간/초록 화살표가 각각
`base_link`의 전방 `+x`/좌측 `+y`다. 차체 중심은 원점에서 전방 1.5275m에 표시된다.
