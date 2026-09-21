# K-City 2025 hybrid completion candidate

이 디렉터리는 안정 구간의 구 KATRI 도색선과 2025 current node/link 기반 재구성선을
합친 검수용 v3 산출물이다. 아직 `operational=false`이며 runtime crop이나 판제 입력으로
승격하지 않았다.

## 적용한 수정

- 정적·동적 장애물 새 도로 `route_s=200~436.798m`: 공식 경로 중심에서 좌측 yellow 1개와
  우측 white 1개를 연속 재생성했다. 원태 관측선은 직접 복사하지 않고, 허용된 관측의
  법선 offset 중앙값으로 폭만 보정했다. 동적 장애물 위치 뒤에서 재등장하던 구맵 이중
  중앙선과 12m 이내 코너 조각도 제거했다. 종방향 도색은 실제 `A1256W000208` 정지선에서
  끝내 이전 candidate가 교차로 안으로 약 8.2m 길게 이어지던 문제를 수정했다.
- 로터리 전 `route_s=478~550.5m`: 원태의 direct `6_left.csv` yellow 외곽선과
  `6_right2.csv` white 외곽선을 station별로 정합했다. 2개 차로 외곽 간 중앙을 route
  차로의 우측 공유선으로 사용해 우측 white만 재생성하고, 실제 이중 yellow는 유지했다.
  공통 관측 62개 1m bin에서 전체 2차로 폭 중앙값은 7.411m, 원시 우측 offset 중앙값은
  1.781m였다. 주행 검수를 반영해 profile을 route 쪽으로 0.25m 이동했고, 최소 half-width
  1.40m 계약을 적용한 최종 중앙값은 약 1.555m다. 관측 polyline 자체는 복사하지 않았다.
- 동적 장애물 이후 우회전 `route_s=550.619~600m`: `A1256W000757` 정지선에서 기존
  실선을 정확히 끝내고, current route를 따라 white/white 점선을 재생성했다. 실제와
  다르게 `556m`까지 남던 구맵 실선과 `576~600m`에서 다시 나타나던 yellow/white 실선을
  제거했으며, 기존 공백도 `3m dash / 3m gap` 점선으로 연결했다.
- 로터리 출구→고주로 `600~647m`: current route와 link 폭으로 양 경계를 재생성했다.
- 로터리 출구 정지선 뒤 고주로 진입 `930.6~974.9m`: 2025 전역경로처럼 직진하는
  white/white 경계를 재생성하고, 구맵에서 갈라지던 최대 12m 이내 connector curve를
  제거했다.
- 고주로 톨게이트 전 안전지대측 `route_s=1593.586~1741.721m`: 구맵의 약 2.2m 바깥쪽
  white 실선을 제거하고, current link `A2256W000153`과 원태 direct `hi_right.csv`가
  지지하는 route-right half-width 중앙값 1.710m로 재생성했다.
- 톨게이트→음영구간 `1776~1885m`: current route와 원태 폭 근거로 양 경계를 재생성했다.
- 고주로: old map에서 누락된 lane 1–2 공유 경계 6개 link-pair를 current topology로
  복원했다. 기존 구맵의 가까운 조각은 1m 이내에서 잘라 중복선을 막았다.
- 정지선: route node 후보 13개 중 실제 도색이 없는 로터리 내부 `A1256W000742`,
  음영 입구 virtual marker `A1256W000831`, 음영 출구 `A1256W000842`를 제외해 10개를
  남겼다. 로터리 진입 정지선은 유지했다.

구맵 lane feature 195개에서 재구성 주변 sample을 제거했고, clip/split 뒤 146개를
유지했다. current 재구성 경계는 route gap/side 12개와 고주로 공유 경계 6개다.

## 주요 산출물

- `hybrid_candidate_map.json`: canonical vector candidate
- `reconstruction_audit.csv`: 재구성 구간·폭·귀속 link
- `link_width_prior_audit.csv`: current 명목 폭과 accepted observation 사용 내역
- `stopline_review.csv`: 13개 후보의 유지/기각 판단
- `overlays/`: 구간별 정적 검수 이미지

오버레이에서 흰색/노란색은 유지한 구맵, 자홍색 외곽이 있는 흰색/노란색은 current
재구성선, 초록색은 주최측 전역경로, 청록색은 current link center, 빨간색은 검수 후 남은
정지선이다.

## 재생성 및 주행 검수

```bash
source devel/setup.bash
python3 src/asmc_hd_map/scripts/build_hybrid_completion.py
python3 src/asmc_hd_map/scripts/build_candidate_review.py
roslaunch asmc_hd_map candidate_map_overlay.launch repo_root:=/root/ws
```

RViz의 `Reviewed hybrid boundaries (candidate)`가 v3 전체 경계다. 기존 토픽 이름
`/asmc_hd_map/review/adopted_legacy_boundaries`는 하위 호환을 위해 유지했지만, 이제
stable legacy와 current reconstruction을 모두 포함한다.
`lane_shape=broken`은 RViz에서도 실선으로 이어 그리지 않고 metric dash/gap 길이로
분할해 표시한다.
