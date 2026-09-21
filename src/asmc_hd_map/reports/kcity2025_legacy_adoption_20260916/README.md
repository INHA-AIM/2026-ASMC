# K-City 2025 구맵 차선 채택 후보

2026-09-16 MORAI 주행 검수 결정을 반영한 비운영 후보 산출물이다. current K-City 2025
node/link가 연결·분기·인접 관계를 계속 소유하고, 실제 도색 geometry·색상·형태만 안정
구간의 구 KATRI MGeo에서 가져왔다. 원태 관측선은 위치 판단과 비교에만 사용하며 최종
차선 geometry로 합치지 않았다.

## 적용 정책

- 구맵 geometry에는 origin translation만 적용하고 회전·scale·snap·일률 폭 보정을 하지 않음
- current corridor link 중심선에서 2.8m 이내이고 방향 차이가 30도 이하인 원본 선분만 유지
- 1m 이하의 내부 검출 공백만 원본 polyline 위에서 연결하고 2m 미만 조각은 제외
- 후보 JSON은 원본 piecewise geometry 위의 0.5m 표본으로 저장하며 임의 smoothing은 하지 않음
- `blue`와 구맵 stopline type 530 제외; 정지선 13개는 current 후보를 그대로 유지
- 톨게이트 출구 `s=1783.057m`부터 음영구간 입구 `s=1851.057m`까지는 자동 채택 제외
- 제외 구간에서 current link와 국소적으로 맞는 19개 조각은 `review_required` layer로만 보존
- 모든 결과는 여전히 `operational=false`

전역 A/B gate만 쓰던 비교 단계와 달리 선분 단위로 재검사하여, 종전 `D`였지만 안정
구간에서 실제 current corridor와 맞는 구맵 source 8개도 복원했다. 현재 70-link 골격에
인접 link가 포함되지 않은 교차로 확장 차선은 버리지 않고 `route_context_only`로 보존했다.
이 선들은 geometry 검수는 통과했지만 정식 current-link 소유권을 임의로 만들지 않는다.

## 결과

| 항목 | 수량 |
|---|---:|
| current link center | 70 |
| current stopline | 13 |
| 채택 구맵 source lane | 163 |
| 채택 구맵 segment | 195 |
| current-link 귀속 segment | 136 |
| 인접 차로 route-context segment | 59 |
| A/B/D segment | 149 / 36 / 10 |
| white/yellow segment | 138 / 57 |
| 변경구간 review-only segment | 19 |
| nominal reference interval | 134 |
| 원태 observed validation interval | 60 |

고주로 같은 방향 current link 30개는 모두 가장 가까운 채택 물리 경계를 p95 2.8m 안에
갖는다. 이는 차로폭을 3.5m로 고정했다는 뜻이 아니라 구간별 구맵 도색 geometry가 모든
차로를 둘러싸는지 확인하는 coverage gate다.

## 산출물

- [후보 vector map](legacy_adopted_candidate_map.json)
- [요약](summary.json)
- [source inventory](adoption_inventory.csv)
- [전체 corridor](overlays/full_corridor.png)
- [초기 교차로](overlays/early_intersections.png)
- [로터리~고주로 진입](overlays/roundabout_to_high_speed.png)
- [고주로~톨게이트](overlays/high_speed_to_tollgate.png)
- [톨게이트~음영 입구 제외구간](overlays/tollgate_to_shadow_excluded.png)
- [음영구간과 이후](overlays/shadow_and_tail.png)

오버레이 색상은 white/yellow가 채택 구맵, cyan이 current link 중심, gray가 명목 경계,
orange가 원태 검산선, magenta 점선이 변경구간 review-only 구맵, red가 current 정지선이다.

재생성:

```bash
source devel/setup.bash
python3 src/asmc_hd_map/scripts/build_legacy_adoption.py
python3 src/asmc_hd_map/scripts/build_candidate_review.py
```

주행 검수:

```bash
roslaunch asmc_hd_map candidate_map_overlay.launch repo_root:=/root/ws
```

RViz 기본 layer는 `Adopted legacy boundaries (candidate)`다. nominal, 원태 관측,
전체 A/B reference와 변경구간 후보는 필요할 때만 켜서 비교한다.
