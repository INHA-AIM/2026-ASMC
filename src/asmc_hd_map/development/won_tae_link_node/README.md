> Ownership cleanup: use `config/waize_katri_runtime.yaml` for live builds.
> `config/waize_katri.yaml` is retained byte-for-byte as the pinned historical
> bake snapshot referenced by the operational-map source manifest.

# link_node — 링크·노드 기반 BEV 정적 레이어 베이커

MGeo `link_set.json` + `node_set.json`과 직접 수집한 GPS/LiDAR 차선 CSV로
주행가능영역 / 차선 / 정지선을 이진(0/255) 래스터로 굽고 HDF5 로 저장하는
오프라인 도구다. 베이크 자체는 ROS·UDP·시뮬레이터 없이 실행된다.

팀 로드맵의 **MGeo map prior** 계층([인지 융합 로드맵](../../../../docs/packages/asmc_perception/roadmap.md)
3.1)이 여기서 구현된다. catkin 패키지가 아니라 오프라인 CLI 라서 `src/perception/`
아래가 아닌 `src/asmc_hd_map/development/won_tae_link_node/` 에 둔다. 로드맵이 함께 잡아둔 **런타임 local crop 은
아직 이 저장소에 없다** — `live_check.py` 의 `ego_view()` 가 기능적으로 같은 일을
0.28 ms/프레임에 하지만 ROS 노드로 감싸지 않았고, 어느 파트가 맡을지 미정이다.

## 최신 운영 요약

- 기준 설정: `src/asmc_hd_map/development/won_tae_link_node/config/waize_katri_runtime.yaml`
- 실측 CSV: `src/asmc_hd_map/development/won_tae_link_node/data/observed_lanes/`의 19개 파일
- 최신·유일한 운영 H5: `src/asmc_hd_map/development/won_tae_link_node/out/link_node_map.h5`
- 라이다용 주행가능영역: `src/asmc_hd_map/development/won_tae_link_node/out/road_mask.png` + `road_mask.yaml`
- 좌표계: UTM52N 원점 `(302595.0, 4124145.0)`을 뺀 map-local ENU
- 해상도: 5 px/m, `3085 × 7232`

`link_node_map.before_*.h5`는 과거 작업 전 백업이었으며 현재는 삭제했다. 소비 코드는
파일명이 정확히 `link_node_map.h5`인 최신 파일만 사용한다.

## 베이크 실행

```bash
cd /home/jang/2026-ASMC

# 현재 운영 설정으로 road/lane/stopline과 라이다용 PNG를 모두 다시 생성
python3 src/asmc_hd_map/development/won_tae_link_node/bake_map.py \
  --config src/asmc_hd_map/development/won_tae_link_node/config/waize_katri_runtime.yaml

# 검증 PNG가 필요 없을 때
python3 src/asmc_hd_map/development/won_tae_link_node/bake_map.py \
  --config src/asmc_hd_map/development/won_tae_link_node/config/waize_katri_runtime.yaml --no-overlay

# 변경 후 회귀검증
python3 -m pytest -q src/asmc_hd_map/development/won_tae_link_node/tests
```

명령은 저장소 루트에서 실행한다. CSV 경로가 저장소 상대경로이므로 `/home/jang/lane_data`
원본이 없어도 베이크할 수 있다. 산출물은 `src/asmc_hd_map/development/won_tae_link_node/out/`에 생성된다.

| 파일 | 내용 |
|---|---|
| `link_node_map.h5` | 최종 `road` / `lane` / `stopline`, 진단 레이어와 벡터 |
| `road_mask.png` | H5의 최종 `road`와 픽셀 단위로 동일한 C++ 라이다 입력 |
| `road_mask.yaml` | PNG의 좌표 원점, 해상도, map id |
| `overlay.png` | 전체 맵 확인용 (road 회색 / 흰·황색 lane / stopline 빨강) |
| `overlay_zoom.png` | 정지선이 가장 밀집한 교차로를 원해상도로 ±60 m |

## 맵

`src/asmc_planner/config/behavior/` 의 판본을 쓴다 (`data_source` 웨이즈원, 2025-10-23).

> ⚠️ 저장소에는 좌표계가 다른 MGeo 맵이 하나 더 있다. `R_KR_PG_KATRI/` 는
> `local_origin_in_global` 이 약 **(-135, -1509) m** 어긋난 별개 측량본이고,
> 링크 `idx` 집합의 교집합이 0 이다. **절대 섞어 쓰지 말 것.**
> H5 의 `map_id` / `local_origin_in_global` attrs 가 이걸 구분하기 위해 있다.

베이크 대상 규모: 링크 634 / 노드 480 / 정지선 노드 150,
bounds `x[-404.5, 112.4] y[-775.0, 571.2]`, ppm 5 에서 **3085 × 7232 px**.

## 레이어별 방식

### 주행가능영역 (`road`)
링크 중심선을 `width_start`(이 맵은 634개 전부 3.5 m) 두께의 폴리라인으로 그린다.
이어서 두 단계 후처리를 한다:

0. **합류부 테이퍼**(`merge_taper_m`, 기본 **0 = 끔**) — 주행가능영역은 차로별 통로의
   합집합이라, 차로가 끝나면 그 통로가 그 자리에서 직각으로 잘려 도로가 뚝 좁아진다
   (차로 수가 줄어드는 노드가 116곳: 3→1 이 19곳, 2→1 이 97곳). 끝나는 차로를 조금 더
   이어 그려 계단을 완만하게 만드는 옵션이 있지만 **기본은 꺼 둔다.**

   > ⚠️ 이건 **링크에 없는 도로를 만들어낸다.** 10 m 로 시험해 봤더니 합류부에서
   > 도로 폭이 `3.5 m → 7.0 m` 로 부풀어, 시뮬에 없는 차로가 통째로 생겼다
   > (`(-6.54, 342.23)` 지점 실측). 켜려면 실제 노면과 대조해가며 아주 작은 값만
   > 쓸 것. 이어지는 링크가 없는 진짜 막다른 끝(6곳)은 어차피 늘리지 않는다.

1. 모폴로지 닫힘(`close_kernel_px`) — 인접 차로 폴리라인 사이의 틈과 코너 계단 제거
2. **래스터화 틈 메우기** — 두 조건 중 하나라도 맞으면 메운다. 907 m² 가 채워진다.

   - **가늘다** (`fill_gaps_max_width_m`, 기본 2 m) — 최대 내접원 반경이 1 m 미만.
     인접 차로 통로 사이의 길고 좁은 틈이 여기 걸린다.
   - **아주 작다** (`fill_gaps_max_area_m2`, 기본 20 m²) — 링크가 갈라지는 지점의
     작은 삼각형 틈은 반경이 1 m 를 살짝 넘어(실측 **1.16 m**) 폭 조건만으로는
     안 걸린다.

   > ⚠️ **면적 하나로만 거르면 안 된다.** 처음에 50 m² 임계를 썼더니 교차로 코너의
   > 인도 섬이 하필 **48 / 50.4 / 56.8 / 57.6 m²** 로 임계값 양옆에 몰려,
   > **같은 교차로의 네 모서리 중 하나만 메워졌다** — 인도가 주행가능영역이 된다.
   >
   > 그렇다고 폭 하나로만 걸러도 안 된다. 위의 1.16 m 짜리 분기 틈이 남아
   > **도로 한복판에 구멍**이 생긴다.
   >
   > 두 조건을 같이 써야 갈린다. 실측상 20 m² 미만 구멍 38개는 전부 내접반경
   > 1.88 m 이하로 인도 섬이 아니고, 남는 것 중 가장 작은 게 22.9 m² 다.

**링크 기반 prior의 최외곽 깎기**(`outer_trim_m`, 기본 **0.3 m**) — 링크의
`width_start`(3.5 m)로 그린
도로는 **실제 노면보다 좌우 0.3 m 씩 넓다**(시뮬과 대조해 확인). 그래서 도로 최외곽을
침식으로 그만큼 깎는다. 차로와 차로 사이는 도로가 이어져 있어 영향을 받지 않는다.

아래 표는 실측 CSV 후처리를 도입하기 전에 `outer_trim_m`만 비교했던 기준 실험이다.
현재 최종 운영 면적은 이 표가 아니라 검증 절의 **89,834.4 m²**다.

| `outer_trim_m` | 당시 prior 도로 면적 | |
|---|---|---|
| 0.0 m | 102,541 m² | 링크 폭 그대로 (실제보다 넓음) |
| 0.2 m | 97,355 m² | |
| **0.3 m** | **93,624 m²** | **실측과 거의 일치** |
| 0.4 m | 91,244 m² | |

도로를 깎으면 그 경계에 있던 차선이 잘려 두께가 반쪽이 된다. 그래서 **최외곽 차선도
같이 안쪽으로 옮긴다**(`lane.follow_outer_trim`, 이동량 = `outer_trim_m` + 선두께/2
= 0.5 m). 링크별로 한 차로 폭 옆을 짚어 이웃 차로가 있는지 보고, **없는 쪽만** 옮긴다
(331개 경계). 차로 사이 선은 그대로 둔다.

```
                0.8 0.9 1.0 1.1 1.2 1.3 1.4 1.5 1.6 …   (중심선에서 m)
[깎기 전]  도로   #   #   #   #   #   #   #   #   #
           차선   .   .   .   .   #   #   #   #   #
[적용 후]  도로   #   #   #   #   #   #   #   #   .
           차선   .   .   #   #   #   #   #   #   .   ← 두께 유지
```

당시 prior-only 결과는 도로 93,624 m², 차선 도로 밖 0 px, 도로 연결 덩어리 4개였다.
현재는 직접 수집 차선의 외곽 도색까지 road에서 제외하므로 최종 차선 일부가 의도적으로
`road=0` 위에 놓이며, 최종 수치는 아래 검증 표를 기준으로 한다.

> ⚠️ **침식 커널은 짝수 크기를 허용해야 한다.** 홀수만 쓰면 반지름이 정수 픽셀로
> 갇혀 ppm=5 에서 0.2 m 단위밖에 못 깎는다 — 0.3 m 요청이 0.4 m 로 반올림된다.
> 커널 크기를 `round(2 × trim × ppm) + 1` 로 잡으면 반 픽셀(0.1 m) 단위가 된다.

> 대안으로 '도로 경계를 따라 띠를 뽑아 차선으로 쓰는' 방법도 시험했는데, 도로 경계는
> 닫힌 윤곽이라 **교차로 개구부와 코너 곡선까지 선이 생긴다.** 링크 기반 오프셋을
> 옮기는 쪽이 기존 규칙(교차로 30 m 제외 등)을 그대로 유지한다.

### 차선 (`lane`)
**이 맵에는 `lane_boundary_set.json` / `lane_marking_set.json` 이 없다.** 링크의
`lane_mark_left` / `lane_mark_right` 가 1095개 ID 를 참조하지만 대상 파일이
저장소에 존재하지 않는다. 그래서 링크 위상에서 합성한다.

근거: **MGeo 링크는 도로가 아니라 차로 단위 중심선이다.** 같은 `road_id` 안에서
`ego_lane` 1 ↔ 2 링크의 횡방향 간격을 재면 중앙값 **3.50 m** 로 `width_start` 와
정확히 일치한다(n=80). 따라서 중심선을 법선 방향 ±`width/2` 오프셋하면 그게 곧
차선 경계이고, 한 차로의 좌경계와 왼쪽 차로의 우경계는 같은 위치에 떨어진다.
이진 래스터라 겹쳐 그려도 결과가 같아 중복 제거가 필요 없다.

#### 직접 수집 차선 override

`observed_lane` 설정이 켜져 있으면 CSV의 `lane_map_x/y`를 같은 격자에
래스터화한다. 현재 `keep_prior: false`이므로 최종 `lane`에는 직접 수집 차선만
들어가고 링크 기반 예측 차선은 `lane_prior`에만 보존된다. 좌·우 중심점에는 수집
시각화와 같은 `inward_offset_m: 0.03` 보정을 먼저 적용한다.

현재 사용하는 CSV 19개는 모두 `src/asmc_hd_map/development/won_tae_link_node/data/observed_lanes/`에 함께 보관한다.
새 CSV를 적용할 때도 이 폴더에 복사한 뒤 `waize_katri_runtime.yaml`의 `observed_lane.sources`
에 **저장소 상대경로**로 추가한다. `/home/jang/lane_data/...` 같은 개인 절대경로를
설정에 다시 넣지 않는다.

`fill_road_between_sides: true`이면 각 CSV의 보정된 LEFT/RIGHT 경계 사이를 채워
기존 링크 기반 `road`와 합친다. 링크 폭이 실제보다 좁은 구간에서도 실측 차선과 그
사이 노면이 주행가능영역에 포함된다. 여러 CSV의 영역은 합집합이므로 인접 차로를
각각 수집한 경우 다차선 영역으로 합쳐진다.

`side_colors` 설정은 CSV의 소수 색상 오검출보다 우선한다. 중앙선처럼 색이 명확한
파일은 `LEFT: YELLOW`, 외곽 흰선은 `RIGHT: WHITE`처럼 파일별로 지정한다. 합쳐진
`ro_merge3.csv`처럼 한 side에 여러 실제 색이 있는 파일은 CSV의 `lane_color`를
그대로 사용한다.

최종 `road`에는 실측 차선을 이용한 후처리도 적용된다.

1. 좌·우가 함께 수집된 파일은 두 경계 사이를 road로 보강한다.
2. 실측선 양쪽의 인접 링크를 조사해 최외곽선 바깥 road와 외곽 도색 폭을 제거한다.
3. 합류부 A411에서 외곽선으로 오판되어 잘린 원래 링크 road 17.84 m²는 지정 복구한다.
4. `A2256W000432→427`과 `A2256W000153→451` 사이 구조물은 두 실측 흰선 경계까지
   포함해 74.88 m²를 주행불가로 제거한다.
5. `turnel_left`와 `end_left2` 사이의 6.194 m 미수집 구간은 접선 방향이 일치하는
   수동 bridge로 연결한다.

내부 차선은 도로 안의 도색이므로 `lane=255`와 `road=255`가 동시에 될 수 있다.
외곽선으로 판정된 도색만 `road=0`이다. 따라서 소비 코드에서 `lane`을 무조건
주행불가로 취급하면 다차선의 내부 차선까지 장애물처럼 처리하므로 주의한다.

H5에는 비교·복구를 위해 다음 레이어를 같이 남긴다.

| 데이터셋 | 내용 |
|---|---|
| `lane` | 실제 소비 코드가 사용하는 교체 완료 차선 |
| `lane_prior` | 수정 전 링크 기반 예측 차선 |
| `lane_observed` | 직접 수집 차선 전체 |
| `lane_observed_white` / `lane_observed_yellow` | 색상별 직접 수집 차선 |
| `lane_erase_mask` | 실제로 삭제된 prior 차선 픽셀 |
| `road_prior` | 실측 경계 보강 전 링크 기반 주행가능영역 |
| `road_observed` | 각 CSV의 좌·우 실측 경계 사이 영역 |
| `road_outer_trim_mask` | 실측 최외곽선 바깥에서 제거된 road |
| `road_outer_lane_excluded` | road에서 제외된 외곽 도색 픽셀 |
| `road_manual_restore` | A411에서 되살린 원래 유효 road |
| `road_boundary_exclusion` | 두 도로 사이 구조물 제거 polygon |

`observed_lane_points` / `observed_lane_offsets`에는 0.03 m 보정된 실측 중심선
벡터를 함께 저장한다. `live_check.py --show-object-lane-distance`는 이 벡터와 MORAI
객체의 실제 회전 박스를 사용해 객체 외곽에서 차선 도색 외곽까지의 최단거리를 그린다.

차선을 긋지 않는 링크가 두 종류 있다:

**교차로 구간** — `link_type == '1'` 인 256개는 **'교차로 내부'가 아니라 '정지선에서
출발하는 링크'** 다 (256개 전부 정지선 노드에서 시작하고, `related_signal` 이
straight/left/right_unprotected 로 채워져 있다). 길이가 45~54 m 인 것도 있어서
**통째로 버리면 교차로를 한참 지난 구간의 차선까지 사라진다.**

그래서 정지선 쪽 끝에서 `intersection_clear_m`(기본 30 m)만 잘라내고 나머지에 차선을
긋는다. 30 m 는 실측한 교차로 횡단 거리에서 나왔다 — 정지선→정지선 링크 32개의 길이가
중앙값 15.8 m, p90 28.3 m, **최대 32.3 m** 다. 양끝이 모두 정지선인 링크(교차로를
가로지르기만 하는 것)와, 잘라내고 나면 남는 게 없는 링크는 통째로 제외한다.
결과적으로 209개는 제외되고 **47개는 교차로 너머 구간에 차선이 그려진다.**

`intersection_clear_m: 0` 으로 두면 예전처럼 256개 전부를 버린다.

**U턴·급커넥터 링크** (평균 곡률반경 < `min_avg_radius_m`, 기본 10 m → 20개) — 이건
주행 궤적이지 도색 차선이 아니다. 곡률은 **교차로 구간을 잘라낸 뒤의 중심선**으로 잰다.
좌회전 링크는 교차로 안에서만 급하게 휘고 그 너머는 곧게 이어지므로, 링크 전체로 재면
멀쩡한 뒷구간까지 버리게 된다. 그냥 오프셋하면 **차 앞에 부채꼴 모양 가짜 차선**이
생긴다 (예: `A2256W001098`, 43 m 에 281도 회전, 평균반경 8.8 m).

> ⚠️ **총 회전각으로 거르면 안 된다.** 주행시험장 오벌 링크(`A2256W001069`)는
> 1186 m 에 걸쳐 334도를 돌지만 평균반경이 203 m 인 정상 도로다. 회전각 기준으로
> 자르면 1.2 km 짜리 차선이 통째로 사라진다. 3점 외접원 최소반경도 쓰면 안 된다 —
> 점 간격 노이즈에 흔들려 총회전 22.9도인 링크에서 반경 2.55 m 가 나온다.
> **길이 ÷ 누적 방향변화**만이 안정적이다.

**노드 핀치 되돌리기** (`geometry.fix_node_pinch`, 링크 2개) — 이 맵에서는 같은
도로의 여러 차로가 **노드 좌표 한 점에서 시작해 부챗살처럼 벌어진다**. 예를 들어
`A2256W000172` 와 `A2256W000176` 은 시작점과 끝점이 **둘 다 동일**하고(거리 0.00 m)
중간에서만 3.4 m 벌어진다. 그 구간의 중심선은 실제 차로 위치가 아니라서, 그대로
오프셋하면 **차선이 도로를 가로지르는 대각선**으로 그려진다.

그 구간을 **지우면 차선이 뚝 끊긴다.** 그래서 지우지 않고 제자리로 되돌린다 —
핀치 구간에서 방향이 가장 덜 휘는 링크를 기준선으로 잡고, 나머지를 '기준선 + 정착
간격' 위치로 옮긴다. `A2256W000172` 기준 횡편차가 3.41 m → 0.17 m 로 떨어진다
.

주의할 점이 셋 있다:

**1. 양끝 노드를 모두 공유하는 쌍만 고친다.** 핀치 인공물은 두 끝이 같은 점에 묶여
중간에서만 벌어지는(0 → 3.4 → 0) 모양이다. 한쪽 끝만 공유하고 단조롭게 모이는 건
**실제 합류**이므로 건드리면 안 된다. 이걸 '고쳤더니' `A2256W000422` 가
`x 74.4 → 77.6` 로 3.2 m 밀려나, **사용자가 제외한 차로(`A2256W000425`, x=78.9)
자리에 도로가 되살아났다.** 이 조건을 넣으면 보정 대상이 23개 → 2개로 줄어든다.

분기와도 구분해야 한다. 교차로에서 직진/좌회전이 갈라지는 것(334쌍)은 정상이다.
기준은 '가장 많이 벌어진 지점에서 서로 평행한가'(<15°)다. **끝점에서 재면 안 된다** —
양끝을 모두 공유하는 쌍은 끝 간격이 0 이라 분기로 오판된다.

**2. 보정량에 상한을 둔다** (`max_shift_m`, 기본 4 m). 핀치는 최대 한 차로 폭만큼
어긋나는 현상이므로 그보다 크게 옮겨야 한다면 짝을 잘못 잡은 것이다. 상한이 없을 때
`A2256W000423` 이 **7.03 m(두 차로 폭)** 밀려나, ego 가 제 차로 한가운데 있는데도
차선에 걸친 것처럼 보였다 (최대 13.97 m 밀린 링크도 있었다). 비교 대상은 '지금까지
보정된 상태'가 아니라 **원본**이어야 한다 — 양끝을 각각 보정하면 조금씩 밀린 게 쌓여
상한을 통과해 버린다.

**3. 노드마다 기준선을 하나만 쓴다.** 쌍(pair)마다 따로 고치면 서로 덮어써서, 3차로가
모이는 노드에서 2.1~2.3 m 잔차가 남는다.

**4. 도로와 차선이 같은 중심선을 써야 한다.** 차선만 보정하면 도로는 노드 근처에서
좁은 채로 남아 보정된 차선이 도로 밖으로 삐져나간다 (`lane_near_road_ratio` 0.996).
그래서 보정은 CLI 에서 한 번 계산해 두 레이어에 함께 넘긴다.

**한계 (알고 쓸 것):** 이건 실측 도색 차선이 아니라 링크 위상에서 유도한 근사치다.
곡률이 큰 구간과 차로 수가 바뀌는 구간에서 실제 노면과 어긋난다. 또 중앙선(황색)
판정은 불가능하다 — `opp_traffic` 이 634개 전부 `False`, `lane_change_dir` 도 전부
`None` 이라 근거가 없다. 그래서 **이진 단일 클래스**로만 굽는다.

### 정지선 (`stopline`)
`node_set.json` 의 `on_stop_line == true` 노드 150개를 쓴다. 이 맵은 정지선이
노드의 1급 속성이라 별도 파일이 필요 없다(73개는 `traffic_light_id` 까지 물려 있어
신호 연동에 바로 쓸 수 있다).

노드는 점 정보뿐이라 기하가 없다. 길이는 그 노드로 **진입하는** 링크의 차로 폭을
쓴다(없으면 진출 링크). 차로마다 노드가 따로 있으므로 여러 차로가 자연스럽게 하나의
정지선으로 이어진다. 150개 전부 해석된다(`unresolved_heading: []`).

**방향은 링크 접선의 법선이 아니라, 같은 진입부 노드들이 이루는 직선을 쓴다**
(`align_across_lanes`, 150개 중 48개가 이 방식으로 정렬된다). 우회전 차로처럼 정지선
직전에 휘는 차로가 있으면 접선 법선은 정지선을 기울인다 — `A2256W000309` 는 정지선
직전 1.5 m 에서 꺾여 접선이 **-19.3°** 인데 바로 옆 차로(`A2256W000308`)는 **0.3°** 다.
한 줄이어야 할 두 정지선이 20° 어긋나 보인다. 창을 넓혀 재도 나아지지 않는다 —
차로가 실제로 갈라지기 때문이다 (창 2 m 에서 p90 11.8° → 창 30 m 에서 22.9°).

실측하면 같은 진입부의 정지선 노드들은 **정확히 한 직선 위에 있다**(최대 잔차
0.000 m). MGeo 가 실제 도색된 바 위에 노드를 찍었기 때문이다. 그래서 그 직선을
PCA 로 구해 정지선 방향으로 쓴다. 묶는 조건은 세 가지다:

| 조건 | 기본값 | 뜻 |
|---|---|---|
| 거리 | < 12 m | 같은 진입부 |
| 진행방향 차이 | < 30° | 같은 방향 주행 |
| **진행방향 성분** | < 1.5 m | **앞뒤가 아니라 옆에 나란히** |

세 번째가 핵심이다. 거리만으로 묶으면 서로 다른 진입부가 섞여 바가 도로를 따라
누워버린다(수직편차 최악 **89°**). 나란함 조건을 넣으면 최악 **9.4°** 로 떨어진다.
그래도 20° 를 넘는 그룹은 버리고 접선 법선으로 폴백한다. 혼자 있는 노드(단일 차로
진입부)도 접선 법선을 쓴다.

**도로 밖으로 나간 끝은 잘라낸다** (`geometry.clip_stopline_to_road`, 기본 켬).
정지선 길이는 링크의 차로 폭(3.5 m)인데 도로는 `drivable.outer_trim_m` 만큼 좌우로
깎이므로, 그대로 두면 깎은 쪽 끝이 삐져나온다. 차선의 `clip_lane_to_road` 와 같은
방식이다. 길이를 일률적으로 줄이는 것(`length_scale`)보다 남는 픽셀이 많다:

| 방안 (trim 0.3) | 픽셀 | 도로 밖 | `stopline_on_road` |
|---|---|---|---|
| 손대지 않음 | 459 | 20 | 0.9564 |
| `length_scale 0.90` | 398 | 6 | 0.9849 |
| `length_scale 0.829` (= (3.5−0.6)/3.5) | 381 | 2 | 0.9948 |
| `length_scale 0.80` | 383 | 0 | 1.0000 |
| **도로로 클립** | **439** | **0** | **1.0000** |

래스터만 자르고 **벡터(`stopline_points` / `stopline_lengths`)는 자르지 않는다** —
그쪽은 실제 기하를 그대로 담아야 '가장 가까운 정지선' 조회가 맞는다. 도로 면적에는
영향이 없다 (정지선은 별도 레이어).

#### 일부 정지선만 굽기

기본은 150개 전부다. 특정 정지선만 필요하면 네 가지 방법이 있다.

먼저 노드 id 를 본다:

```bash
python3 src/asmc_hd_map/development/won_tae_link_node/bake_map.py --list-stoplines
# node idx                 x         y  신호등 id          진입 링크
# A1256W000593        -12.70     94.88  -               A2256W000309
# A1256W000742        -12.69     98.93  C1256W000003    A2256W000308
```

그 다음 `config/*.yaml` 의 `stopline:` 아래에 지정한다:

| 방법 | 예 | 결과 |
|---|---|---|
| `include_nodes` | `[A1256W000593, A1256W000742]` | 그 2개만 |
| `exclude_nodes` | `[A1256W000001]` | 1개 빼고 149개 |
| `only_with_traffic_light: true` | | 신호등이 물린 69개 |
| `route_yaml` | `.../route_links.yaml` | 그 경로 위의 7개 |

경로 기반은 CLI 로도 된다 (behavior_planner 의 `route_links*.yaml` 형식을 그대로 읽는다):

```bash
python3 src/asmc_hd_map/development/won_tae_link_node/bake_map.py \
    --stopline-route src/asmc_planner/config/behavior/route_links.yaml
# [stopline] route route_links.yaml 위의 정지선 7개만 굽습니다
```

> 방향 계산은 **선택 여부와 무관하게 전체 노드로 한다.** 선택된 것만으로 묶으면 한 줄에서
> 하나만 골랐을 때 짝이 사라져 접선 법선으로 폴백하면서 정지선이 기울어진다. 실제로
> `A1256W000593` 을 혼자 골라도 각도는 90.0° 로 이웃과 같고, 정렬을 끄면 70.0° 로 기운다.

## 좌표 규약

CARLA / carla-roach `_world_to_pixel` 과 동일하게 OpenCV `(col, row) = (x, y)`:

```
col = round(ppm * (wx - offset_x))
row = round(ppm * (wy - offset_y))
```

`world_offset` 은 링크와 정지선 노드를 모두 감싸는 AABB 의 좌하단에서 `margin_m`
만큼 물러난 지점. 이 규약은 `mapbake/raster.py` 한 곳에만 정의되어 있다 — 레이어마다
흩어지면 축이 뒤바뀌는 버그가 난다.

## H5 스키마

```
/road /lane /stopline     uint8 (H, W), 값 0/255, gzip-9

/lane_prior /lane_observed /lane_observed_white /lane_observed_yellow
/lane_erase_mask
/road_prior /road_observed /road_outer_trim_mask
/road_outer_lane_excluded /road_manual_restore /road_boundary_exclusion
  위 데이터셋은 비교·검증용이다. 실제 소비 대상은 /road /lane /stopline이다.

attrs:
  pixels_per_meter, world_offset_in_meters(float32[2])
  width_in_meters, height_in_meters, width_in_pixels, height_in_pixels
  map_id, local_origin_in_global(float64[3])     ← 맵 혼동 방지
  layers, source_files, layer_params, stats      ← JSON 문자열
  git_sha, baked_at

/stopline_points /stopline_dirs /stopline_headings /stopline_lengths
  구운 정지선의 벡터 원본 (래스터만으로는 '가장 가까운 정지선'을 못 찾는다)
  attrs: stopline_ids, stopline_traffic_lights

/observed_lane_points /observed_lane_offsets
  보정·bridge까지 반영한 실측 차선 벡터
  attrs: observed_lane_sides, observed_lane_colors, observed_lane_sources

/link_points /link_offsets                       ← 검증 전용, 규약 미확정
  링크 중심선을 이어붙인 것 + 링크별 시작 인덱스
  attrs: link_ids, link_in_road, link_in_lane
```

읽을 때는 `mapbake.writer.read_h5_meta(path)` 로 attrs 만 먼저 확인할 수 있다.

> ⚠️ **`link_*` 는 현재 `live_check.py` 만 읽는다** (`--show-links` / `--collect-links`
> 가 주변 링크와 좌우 거리를 표시하는 데 쓴다). 인계 대상인 `drivable_query.py` 는
> 읽지 않고, **외부 파트용 규약도 정해진 바 없다.** 라이다의 `lane_relation`
> (로드맵 3.3) 에 쓰기로 정해지면 `nearest_link` + 종·횡거리 API 를
> `drivable_query.py` 에 얹고 여기에 규약을 적을 것. 안 쓰기로 정해지면 H5 에서
> 빼되, 빼면 검증 뷰어가 깨지므로 플래그로 분리해야 한다.

## 검증

`bake_map.py` 가 매 실행마다 지표를 출력한다. 현재 값:

| 지표 | 값 | 의미 |
|---|---|---|
| `stopline_on_road_ratio` | 1.000 | 정지선 픽셀이 road 안에 있는 비율 (`clip_stopline_to_road` 로 잘라내므로 1.0) |
| `lane_near_road_ratio` | 0.9978 | 차선이 road 경계 ±0.6 m 안에 있는 비율 |
| `world_roundtrip_max_err_m` | 0.084 | `world→pixel→world` 오차 (허용 `1/ppm` = 0.2) |
| `road_area_m2` | 89,834.4 | 실측 외곽 절단·링크 제외·구조물 제외까지 반영된 최종 면적 |
| `lane_px` | 26,199 | 실측 CSV와 수동 bridge가 반영된 최종 차선 픽셀 |
| `stopline_px` | 418 | 선택된 정지선 최종 픽셀 |

> ⚠️ **`stopline_on_road_ratio` 가 낮다고 방향 계산을 파지 말 것.** 정지선 길이는
> 링크의 차로 폭(3.5 m)인데 도로는 `drivable.outer_trim_m` 만큼 좌우로 깎이므로,
> 클립을 끄면 **깎은 쪽 끝이 그만큼 삐져나와** 이 값이 내려간다. 방향과 무관하다.
>
> | `outer_trim_m` | 0.0 | 0.2 | 0.3 | 0.4 |
> |---|---|---|---|---|
> | 클립 끔 | 0.9956 | 0.9695 | 0.9564 | 0.9259 |
>
> 실측(trim 0.3): 8개 중 3개(`A1256W000807` / `A1256W000582` / `A1256W000682`)의
> **우측 끝만** 밖으로 나가고 459 px 중 20 px(4.4%) 이다. 중앙은 전부 도로 안이다 —
> 방향이 틀렸다면 선 전체가 어긋난다. 이 오진을 막으려고
> `test_stopline_clip_removes_ends_left_outside_by_outer_trim` 을 박아뒀다.

`overlay.png` 는 [asmc_planner 대시보드](../../../asmc_planner/web/index.html)와
같은 맵이므로 눈으로 직접 대조할 수 있다.

## 시각화

### 정적 결과 확인

베이크 후 `src/asmc_hd_map/development/won_tae_link_node/out/overlay.png`를 열면 전체 지도를 확인할 수 있다. 회색은
최종 road, 흰색/노란색은 직접 수집 차선, 빨간색은 정지선이다. 시뮬레이터 연결 없이
ego 중심 화면을 보려면 다음을 실행한다.

```bash
cd /home/jang/2026-ASMC
python3 src/asmc_hd_map/development/won_tae_link_node/live_check.py --source demo

# 더 확대: 한 화면에 보이는 실제 범위를 줄인다. 창 크기도 함께 키울 수 있다.
python3 src/asmc_hd_map/development/won_tae_link_node/live_check.py --source demo --window-m 30 --out-px 900
```

현재 뷰어에는 실행 중 마우스 휠/키보드 줌이 없다. `--window-m`이 작을수록 확대되고
클수록 축소된다. 값을 바꾸려면 뷰어를 다시 실행한다.

## MORAI 실주행 대조 검증

`live_check.py` 는 MORAI 의 Ego 상태를 받아 구운 맵과 대조한다. **ROS 워크스페이스를
빌드하지 않아도** 되도록 Ego UDP 를 직접 파싱하는 경로를 기본으로 쓴다.

```bash
# MORAI Destination PORT가 기본 909일 때 먼저 수신 확인
sudo -E python3 src/asmc_hd_map/development/won_tae_link_node/live_check.py --probe --seconds 15

# Ego UDP 직접 수신
sudo -E python3 src/asmc_hd_map/development/won_tae_link_node/live_check.py

# MORAI Destination PORT를 911로 설정했다면 반드시 같은 포트를 명시
python3 src/asmc_hd_map/development/won_tae_link_node/live_check.py --port 911
python3 src/asmc_hd_map/development/won_tae_link_node/live_check.py --probe --analyze --port 911 --seconds 15

# Ego 패킷에 위치가 없을 때: 위치는 GPS, 방향은 Ego
sudo -E python3 src/asmc_hd_map/development/won_tae_link_node/live_check.py --source gps

# /Ego_topic과 /Object_topic이 이미 뜬 ROS 환경
python3 src/asmc_hd_map/development/won_tae_link_node/live_check.py --source ros --objects \
  --show-object-lane-distance --show-outside-objects

# 창 없이 수치만 확인
python3 src/asmc_hd_map/development/won_tae_link_node/live_check.py --source ros --no-window
```

**포트는 909** — `morai_udp_bridge.launch` 의 `ego_port` 와 같은 값이고, MORAI
**Network Settings > Ego Vehicle Status** 의 Destination PORT 다. 909 는 1024 미만
특권 포트라 바인드에 root 가 필요하다 (docker 브릿지가 그냥 되는 건 컨테이너가
root 로 돌기 때문). `sudo -E` 의 `-E` 는 `DISPLAY` 를 유지해 창이 뜨게 한다.
sudo 를 피하려면 MORAI 쪽 포트를 1024 이상으로 바꾸고 `--port` 로 같은 값을 준다.

Destination IP 는 MORAI(Windows) 에서 이 호스트로 닿는 주소다. WSL 이면 `hostname -I`
의 첫 값(NAT 모드에서는 127.0.0.1 이 아니다). 실행하면 후보 IP 를 출력해 준다.

ROS 브리지가 Docker 컨테이너 안에서 이미 실행 중이면 UDP 포트를 호스트 뷰어와
경쟁시키지 말고 같은 컨테이너에서 ROS 소스를 사용한다.

```bash
# Docker 명령이 되는 터미널에서 실행. <container>는 docker ps에 나온 이름이다.
docker ps --format 'table {{.Names}}\t{{.Image}}\t{{.Status}}'
docker exec -it <container> bash

# 컨테이너 안
cd /root/ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
python3 src/asmc_hd_map/development/won_tae_link_node/live_check.py --source ros --objects \
  --show-object-lane-distance --show-outside-objects
```

WSL에서 `docker: command not found`가 나오면 Docker Desktop의 해당 배포판 WSL
integration을 켜거나 Windows PowerShell에서 `docker exec`을 실행한다. 호스트 WSL에서
`cd /root/ws`를 실행하는 것은 컨테이너 접속이 아니며 권한 오류가 나는 것이 정상이다.

> docker 브릿지를 동시에 돌리는 중이라면 MORAI 는 Ego 를 한 곳으로만 보내므로 둘이
> 포트를 다툰다. 그럴 땐 브릿지를 띄운 채 컨테이너 안에서 `--source ros` 로 붙이면
> 된다.

패킷이 안 오면 `--probe` 가 원인을 좁혀준다. "들어온 패킷이 없다"면 MORAI 목적지
설정이나 Windows 방화벽 문제이고, 패킷은 오는데 "Ego 아님/미지 판본"이면 그 포트를
다른 항목이 쓰고 있는 것이다.

ego 중심 BEV 창과 함께 두 가지 지표가 나온다:

**1. `on_road_ratio` — 맵 판본 검증.** ego 위치를 구운 `road` 마스크에서 샘플링한다.
정상 주행 중이라면 1.0 에 가깝다. 이 값이 낮거나 `outside_map` 이 쌓이면 MORAI 에
로드된 맵이 `behavior_planner` 판본과 다르다는 뜻이다 (`R_KR_PG_KATRI` 판본이면
약 (-135, -1509) m 어긋나 즉시 도로 밖으로 나간다).

**2. `lane_dist_vs_morai_*_err_m` — 합성 차선 검증.** 245 B 판본 패킷에는 MORAI 가
계산한 `distance_left/right_lane_boundary` 가 들어 있다. 이걸 구운 `lane` 래스터에서
ego 기준 횡방향으로 훑어 잰 거리와 비교한다. 합성 차선이 실제 노면과 맞는지
정량으로 확인할 수 있는 유일한 수단이다. (구버전 패킷은 이 필드가 없어 생략된다.)

### 조향만 반영되고 차가 움직여도 화면이 안 따라올 때

**HUD 의 `odom` 과 `ego (x, y)` 를 먼저 본다.**

`odom` 이 0 이고 좌표가 고정이면 위치가 파싱되지 않는 것이다. 오프셋을 추측하지 말고
실측한다 — 차를 계속 움직이면서:

```bash
sudo -E python3 src/asmc_hd_map/development/won_tae_link_node/live_check.py --probe --analyze
```

분석기는 세 가지를 한다:

1. **프레임 검증** — 차체 제원(Ioniq 5: size 4.635/1.892/2.434, wheelbase 3.0)이
   문서상 자리에 있는지 본다. 통과하면 오프셋 계산은 맞은 것이다.
2. **변하는 값 나열** — 차가 움직이는 동안 실제로 변한 float 를 전부 보여준다.
3. **속도 정합 검증** — 위치라면 연속 표본의 이동량 ÷ 시간이 패킷이 보고하는 속도와
   맞아야 한다. accel/brake(0~1)나 360도에서 감기는 각도 같은 가짜 후보가 이 단계에서
   걸러진다.

후보가 나오면 그 인덱스를 그대로 넘긴다 (코드 수정 불필요):

```bash
sudo -E python3 src/asmc_hd_map/development/won_tae_link_node/live_check.py --pos-index 10
```

#### Ego 패킷에 위치가 아예 없을 때 → GPS 로 받는다

**181B(beta) Ego 패킷은 position 이 0 으로 채워져 오는 경우가 있다.** 이때 분석기는
"프레임 검증은 통과했는데 쓸 수 있는 위치 필드가 없다"고 알려준다. 파싱 문제가 아니라
시뮬이 실제로 0 을 보내는 것이므로, 위치만 GPS 센서에서 받는다:

```bash
sudo -E python3 src/asmc_hd_map/development/won_tae_link_node/live_check.py --source gps
```

위치는 GPS(기본 udp/9301, NMEA GGA), 방향과 속도는 Ego 패킷에서 가져온다. NMEA 에는
맵 원점 오프셋이 없으므로 위경도를 UTM(기본 EPSG:32652 = zone 52N)으로 옮긴 뒤 H5 의
`local_origin_in_global` 을 빼서 맵 로컬 좌표로 만든다. 다른 맵이면 `--utm-epsg` 로
좌표계를 바꾼다.

MORAI 센서셋에 GPS 가 켜져 있어야 한다 (`ASMC_sensor_set_v1.json` 은 Dest 9301).

### 지정할 정지선 수집하기

`include_nodes` 에 넣을 노드 id 를, **MORAI 로 실제 주행하면서 모을 수 있다.**

```bash
sudo -E python3 src/asmc_hd_map/development/won_tae_link_node/live_check.py --source gps --collect \
    --collect-out src/asmc_hd_map/development/won_tae_link_node/out/collected_stoplines.yaml
```

지나치는 정지선을 자동으로 담고, 종료하면 붙여넣기 가능한 yaml 을 출력한다:

```yaml
# live_check.py 로 수집 (2026-09-05 23:01:06), 8개
# config 의 stopline: 아래에 붙여넣으세요.
stopline:
  include_nodes:
    - A1256W000672    # (5.28, -155.70)  TL=C1256W000045  통과각 5deg
    - A1256W000516    # (-1.20, -63.94)  TL=C1256W000054  통과각 0deg
```

| 옵션 | 기본 | 뜻 |
|---|---|---|
| `--collect [R]` | 15 m | 이 반경 안에 들어온 정지선을 담는다 |
| `--collect-facing-deg` | 45° | 진행방향과 이 각도 안쪽인 것만. **교차 도로의 정지선을 거른다** |
| `--collect-out` | — | yaml 저장 경로 (없으면 화면 출력만) |
| `--collect-append` | 끔 | 기존 파일에 **합친다**. 정지선마다 프로그램을 껐다 켜며 모을 때 |

정지선 하나씩 껐다 켜며 모을 거라면 `--collect-append` 를 붙인다. 없으면 실행할 때마다
덮어써서 마지막 하나만 남는다.

```bash
sudo -E python3 src/asmc_hd_map/development/won_tae_link_node/live_check.py --source gps --collect --collect-append \
    --collect-out src/asmc_hd_map/development/won_tae_link_node/out/collected_stoplines.yaml
```

#### 차로가 2개면 노드도 2개다 (정지선이 2개가 되는 게 아니다)

정지선 노드는 **차로마다 하나씩** 있다. 왕복 2차로 진입부라면 거의 같은 위치에서
노드가 2개 잡힌다 — 예를 들어 `A1256W000742` (-12.69, 98.93) 와 `A1256W000593`
(-12.70, 94.88) 은 4.05 m 떨어져 있고 신호등 id 도 `C1256W000052` 로 같다.

둘 다 넣으면 **한 줄로 이어진다** (사이 구간 100% 채워짐). 각 노드가 자기 차로 폭
3.5 m 만 그리고, 방향은 노드 정렬로 같은 각도를 쓰기 때문이다. 겹쳐 그려지거나 두
줄이 되지 않는다.

**하나만 넣으면 그 차로만 덮는다.** 도로 전폭을 가리려면 같은 줄의 노드를 모두 넣어야
한다. `--list-stoplines` 로 좌표가 3~4 m 안에 붙어 있고 신호등 id 가 같은 것들을 찾으면
된다.



창이 떠 있으면 `s` 로 지금 가장 가까운 정지선을 수동 추가, `u` 로 마지막 취소,
`q` 로 종료한다. HUD 에는 항상 가장 가까운 정지선이 뜬다:

```
stopline A1256W000593   12.3 m ahead  lat +0.4 m  face   2deg  TL C1256W000003
```

`--collect-facing-deg` 가 왜 필요한지: 방향을 무시하면(180) 교차로마다 옆길 정지선까지
담겨 한 경로에서 17개가 나온다. 45° 로 두면 8개다. 참고로 같은 경로를
`--stopline-route` 로 고르면 7개이고, **주행 수집 8개가 그 7개를 모두 포함한다**
(서로 독립적인 두 방법이 교차 검증된다).

이게 가능한 건 H5 가 래스터뿐 아니라 **정지선의 위치·방향을 벡터로도** 담기 때문이다
(`stopline_points` / `stopline_headings` / `stopline_lengths` 데이터셋과
`stopline_ids` / `stopline_traffic_lights` attrs). 래스터만으로는 '가장 가까운
정지선이 무엇인지' 답할 수 없다.

### 주행가능영역 안의 객체만 표시하기

MORAI 의 Object Info 를 받아, **주행가능영역 안에 있는 객체만** BEV 에 그린다.
시뮬에서 객체를 배치하며 "이게 도로 위인지" 확인할 때 쓴다.

```bash
sudo -E python3 src/asmc_hd_map/development/won_tae_link_node/live_check.py --source gps --objects
```

기본 포트는 **7505** (`morai_udp_bridge.launch` 의 `object_port` 와 같다).
MORAI Network Settings 의 Object Info Destination 을 이 포트로 맞춘다.

| 옵션 | 기본 | 뜻 |
|---|---|---|
| `--objects [PORT]` | 7505 | Object Info UDP 수신 |
| `--object-margin-m` | 0 | 판정에 줄 여유. 경계 객체가 들락거리면 0.5~1.0 |
| `--show-outside-objects` | 끔 | 도로 밖 객체도 회색 테두리로 (확인용) |

HUD 에 집계가 뜬다:

```
objects 6   on-road 5   off-road 1 (hidden)
```

차량은 파랑, 보행자는 빨강으로 채워 그리고 id 를 붙인다. 도로 밖 객체는 그리지
않는다(`--show-outside-objects` 를 주면 회색 테두리로만).

판정은 중심점 하나가 아니라 **차체 네 귀퉁이와 중심 5점** 중 하나라도 도로에 걸치면
안으로 친다. 한 점만 보면 경계에서 들락거린다.

이 판정은 `lane` 레이어를 별도로 보지 않고 **오직 H5의 최종 `road`**만 본다. 다만
내부 차선이나 외곽으로 분류되지 못한 차선은 같은 픽셀에서 `lane=255`, `road=255`일
수 있다. 화면은 회색 road 위에 차선을 나중에 덮어 그리므로 회색이 보이지 않아도
차선 밑 road 값은 255일 수 있다. 또한 5점 중 하나만 닿아도 객체 전체가 표시된다.
정확한 인지 필터 결과를 검증할 때는 아래 C++의 OBB 내부비율 방식과 혼동하지 않는다.

> ⚠️ 이 판정은 **측위 정확도에 종속**된다. 위치가 0.3 m 만 틀려도 도로 가장자리
> 객체가 밖으로 튀어나간다(실측). 구운 도로도 링크 중심선 × 3.5 m 근사라 갓길이
> 없다. **인지에서 객체를 버리는 용도로 쓰지 말 것** — 계산 범위를 좁히는 용도로
> 쓰고, 쓸 거면 `--object-margin-m` 로 여유를 줄 것.

### 제외할 주행가능영역·차선 수집하기

정지선과 같은 방식으로, **주행하면서 밟은 링크 id 를 모을 수 있다.** 주행가능영역과
차선은 둘 다 링크에서 나오므로 제외 단위도 링크다.

```bash
sudo -E python3 src/asmc_hd_map/development/won_tae_link_node/live_check.py --source gps --collect-links \
    --collect-out src/asmc_hd_map/development/won_tae_link_node/out/collected.yaml
```

반경 3 m 안에 들어온 링크를 담고, 종료하면 `collected_links.yaml` 을 쓴다:

```yaml
# config 의 drivable.exclude_links / lane.exclude_links 에 붙여넣으세요.
# (road=O 는 주행가능영역에, lane=O 는 차선에 쓰이고 있다는 뜻)
exclude_links:
  - A2256W000176    # (-31.61, -248.79)  road=O lane=O
  - A2256W000370    # (5.27, -154.70)  road=O lane=-
```

HUD 에 지금 밟고 있는 링크가 뜬다:

```
link A2256W000176   0.42 m  road O  lane O
```

창이 있으면 `l` 로 수동 추가, `k` 로 마지막 취소. `--collect-append` 로 여러 번에
나눠 모을 수 있다. 정지선(`--collect`)과 동시에 켜도 된다 — 파일이 따로 나온다.

붙여넣는 곳은 두 군데다. 둘 중 한쪽에만 넣으면 그 레이어에서만 빠진다:

```yaml
drivable:
  exclude_links: [A2256W000176]   # 링크(차로) 단위
  exclude_roads: ["4000112"]      # road_id 단위 — 그 도로의 모든 차로
lane:
  exclude_links: [A2256W000176]
```

여러 차로짜리 도로를 주행하며 모으면 **지나간 차로만** 빠진다. 그러면 남은 차로의
차선만 그려져 ego 가 차선에 걸친 것처럼 보인다. 베이크할 때 경고가 뜬다:

```
[warn] road: 일부 차로만 제외된 도로 7개 — 4000104(1/5), 4000107(1/5), 4000112(2/4) ...
       도로 전체를 빼려면 exclude_roads 에 road_id 를 넣으세요.
```

어느 링크를 뺄지 고를 때는 `--show-links` 로 주변 링크를 색과 id, 좌우 거리까지
같이 그려서 본다:

```bash
sudo -E python3 src/asmc_hd_map/development/won_tae_link_node/live_check.py --source gps --show-links --collect-links
```

HUD 에 왼쪽부터 차례로 뜬다. 시뮬 화면의 색만 보고 어느 링크인지 맞출 수는 없으므로,
**좌/우 몇 m 인지로 짚는다:**

```
link A2256W000423   0.35 m  road -  lane -
  A2256W000445  L 2.2 m  road O  lane O
  A2256W000423  R 0.0 m  road -  lane -
  A2256W000422  R 1.3 m  road O  lane O
  A2256W000425  R 3.6 m  road -  lane -
```

`road -` / `lane -` 는 이미 제외된 링크다. 빼고 싶은 링크 위로 차를 옮기면 `link` 줄이
그 id 가 되고, `l` 키로 담으면 된다.

> ⚠️ 제외는 **그 링크의 통로만** 지운다. 옆 차로 통로가 같은 자리를 덮고 있으면
> 그 부분은 남는다 (실측: 38개를 뺐을 때 10개 중 5개만 완전히 비었다). 도로 한 폭을
> 통째로 지우려면 그 자리를 덮는 링크를 모두 넣어야 한다 — 같은 지점을 차로마다
> 지나가며 모으면 된다.

#### `odom` 은 느는데 도로만 그대로 보일 때

정상이다. 구운 road/lane 은 진행 방향으로 균일해서, 직선 구간에서 순수 전진으로
바뀌는 픽셀이 5% 남짓(차선 가장자리 요철)뿐이다. 그래서 뷰에 월드 고정 기준을 넣어
두었다:

- `--grid-m` 간격(기본 10 m)의 월드 축 정렬 격자 — 전진하면 흘러간다
- 우상단 북(N) 화살표 — 조향하면 돈다

격자가 흐르면 위치는 정상 반영되고 있는 것이다.

오프라인 사전 검증으로, 링크 중간점 367곳에서 좌우 차선까지 거리를 재면 중앙값
1.70 / 1.80 m 로 기대값 `width/2 = 1.75 m` 와 맞고, 87~92% 가 오차 0.3 m 이내다
(`test_lane_distance_from_lane_center_is_half_width`).

합성 Ego 패킷을 링크 중심선을 따라 쏘아 전 경로를 확인한 결과
`on_road_ratio 1.0`, `lane_dist_vs_morai_mean_err_m 0.075` (p90 0.15) 가 나온다.
MORAI 를 붙였을 때 정상이라면 이와 비슷한 값이어야 한다.

## 라이다 파트 인계

### 현재 연결 상태

| 소비자 | 읽는 파일 | 현재 상태 |
|---|---|---|
| C++ LiDAR `Main` | `road_mask.png` + `road_mask.yaml` | `main.launch`에 이미 연결됨 |
| Python 라이다/판단제어 | `link_node_map.h5` + `drivable_query.py` | API 제공, 소비 노드에서 직접 로드해야 함 |
| `live_check.py` | `link_node_map.h5` | 기본 경로로 이미 연결됨 |

C++ LiDAR는 HDF5 의존성을 두지 않기 위해 H5를 직접 읽지 않는다. `bake_map.py`가
최종 H5의 `/road`와 완전히 같은 `road_mask.png`와 좌표 메타데이터 YAML을 함께 쓴다.
현재 두 road의 픽셀 차이는 0이다.

### Python 소비자가 가져갈 파일

| 파일 | 무엇 |
|---|---|---|
| `src/asmc_hd_map/development/won_tae_link_node/drivable_query.py` | 조회 모듈. 이 저장소의 다른 코드를 import 하지 않는다 |
| `src/asmc_hd_map/development/won_tae_link_node/out/link_node_map.h5` | 약 1.7 MB. 최종 road/lane/stopline과 벡터/메타 |

필요 패키지는 `numpy`, `h5py`, `opencv-python` 뿐이다.

### 좌표계 — 변환 불필요

구운 맵은 **원점이 UTM52N (302595.0, 4124145.0) 인 ENU** 다 (E = +x, N = +y, 회전 없음).

라이다 쪽 ENU 도 같은 원점이다. `Parser.cpp` 의 GPS 콜백이

```cpp
st_GPSData.f64_E = UTMUPS::Forward(lat, lon, /*zone=*/52, ...).east - msg.eastOffset;
st_GPSData.f64_N = ... .north - msg.northOffset;
```

로 계산하고, `morai_sensor_bridge.launch` 가 그 offset 을 `302595.0 / 4124145.0` 으로
넣어준다 — 구운 맵의 `local_origin_in_global` 과 같은 값이다. 따라서
`Body2ENU_Point` / `MinMax2ENU` 결과를 **그대로 넣으면 된다.**

> 실행 중에 한 번 확인해 두면 좋다. `rostopic echo /gps -n1 | grep -i offset` 이
> `302595 / 4124145` 면 그대로 쓰고, `0 / 0` 이면 ENU 가 UTM 절대좌표라는 뜻이므로
> `dm.set_utm_input()` 한 줄을 추가한다. 코드에서 확인하려면
> `dm.looks_like_same_frame(points)` 를 쓴다.

### 쓰는 법

```python
from drivable_query import DrivableMask

dm = DrivableMask.load('link_node_map.h5', margin_m=0.0)
dm.check_map('waize_katri_2025-10-23')      # 맵 판본 확인 (권장)

# 점 단위 — (N,2) ENU 미터 -> (N,) bool
keep = dm.contains(points_en)

# OBB 단위 — 중심 + 네 귀퉁이 중 하나라도 걸치면 True(느슨한 확인용)
on = dm.obb_overlaps(cx, cy, length_m, width_m, yaw_deg)

# 이진 대신 0~1 점수로 받기 (권장)
score = dm.obb_inside_ratio(cx, cy, length_m, width_m, yaw_deg)
```

`margin_m=0.0`은 베이크 경계를 그대로 쓴다. 측위 오차 여유가 필요하면 먼저
`0.2~0.3 m`로 검증한다. `1.0 m`는 좁은 구조물 제외 영역이나 차선 폭만큼 잘라낸
영역을 다시 메울 수 있으므로 “정확한 베이크 경계”가 목적이면 쓰지 않는다.

| 메서드 | 용도 |
|---|---|
| `contains(points)` | 점 벡터 조회 |
| `obb_overlaps(...)` | OBB 가 주행가능영역에 걸치는지 |
| `obb_inside_ratio(...)` | OBB 가 든 비율 (0~1) |
| `quads_inside_ratio(quads)` | **(N,4,2) 사각형들의 내부 비율 (N,). 배치·꼭짓점 입력** |
| `quad_inside_ratio(corners)` | 사각형 하나의 내부 비율 |
| `check_map(map_id)` | 맵 판본 확인, 다르면 예외 |
| `looks_like_same_frame(points)` | 좌표계 원점이 맞는지 확인 |

### 성능

| | |
|---|---|
| 20만 점 조회 | **6.3 ms** (10 Hz 스캔 기준 여유 15배) |
| 객체 200개 × 25점 배치 | **0.88 ms** (`quads_inside_ratio`) |
| 조회 1회 | 1.6 µs, 맵 크기와 무관한 O(1) |
| 로드 | 0.05 s, RAM 상주 **22 MB** (`road` 한 장만 읽는다) |

점 하나당 나눗셈 2번 + 배열 인덱싱 1번이다. 탐색이나 매칭은 없다.

### 반드시 알아야 할 것

**0. 정지선은 8개만 구워져 있다.** 이 맵의 정지선 노드는 150개인데, 현재
`config/waize_katri_runtime.yaml` 의 `stopline.include_nodes` 가 **경로 검증용으로 주행하며
모은 8개**로 한정되어 있다. 대회 경로가 확정돼서 고른 게 아니라 그 시점 주행 구간의
것이므로 **임시 상태**다. 전체가 필요하면 `include_nodes` 를 비우고 다시 구우면
150개가 다 나온다. (라이다 조회는 `road` 레이어만 쓰므로 실사용에는 영향이 없다 —
`stopline` 레이어나 정지선 벡터를 쓸 거라면 반드시 확인할 것.)

**1. `margin_m`은 목적에 맞게 정할 것.** 현재 베이크는 실측 외곽선과 구조물 경계를
road에서 의도적으로 잘라냈다. 정확한 경계 검증은 `0.0`, 일반적인 GNSS 오차 여유는
우선 `0.2~0.3 m`부터 시험한다. `1.0 m`는 놓침을 줄이는 보수적 객체 필터에는 쓸 수
있지만, 양쪽에서 각각 1 m 팽창하므로 폭 2 m 이하의 좁은 제외 영역을 사실상 다시
주행가능하게 만든다.

**2. 판정은 측위 정확도에 종속된다.** 위치는 이 모듈이 추정하지 않는다 — 주어진
좌표를 읽을 뿐이다. GPS 가 어긋나면 판정도 그만큼 어긋난다.

**3. 전체가 실측 노면인 것은 아니다.** 기본 road는 링크 중심선 × 차로폭에서
유도하고, 수집한 경계 사이 보강·외곽 절단·수동 복구·구조물 제거를 그 위에 적용한다.
CSV가 없는 구간은 여전히 링크 기반 근사치다.

**4. 제외된 구간이 있다.** 현재 링크 24개를 뺀 상태다. 거기 객체는 전부 '밖'으로
판정된다. 목록은 `config/waize_katri_runtime.yaml`의 `drivable.exclude_links`에 있다.

**5. 하드 게이트로 쓰지 말 것.** 객체를 버리는 판단에 쓰면 되돌릴 수 없다.
계산 범위를 좁히는 용도로 쓰고, `obb_inside_ratio()` 점수를 넘겨 최종 판단은
뒷단에 남기는 것을 권한다.

### 자체 확인

```bash
python3 src/asmc_hd_map/development/won_tae_link_node/drivable_query.py
# map_id=waize_katri_2025-10-23  ppm=5.0  margin=1.0 m  shape=(7232, 3085)
# 20만 점 조회 6.30 ms, 주행가능영역 안 17.6%
# OBB 예: True (내부 비율 1.00)
```

이 데모는 성능 확인을 위해 코드 내부에서 `margin_m=1.0`을 사용한다. 운영 권장값을
뜻하지 않는다. 베이크 원본을 확인하려면 호출부를 `margin_m=0.0`으로 사용한다.

## 판단·제어 파트 사용법

현재 저장소의 판단·제어 노드가 `link_node_map.h5`를 자동으로 읽도록 연결돼 있지는
않다. 아래처럼 직접 로드하거나, 필요한 local crop을 ROS 토픽으로 발행하는 adapter를
붙여야 한다. “H5에 들어 있음”과 “판단제어가 이미 사용 중”은 다르다.

세 래스터를 같은 API로 점 조회할 수 있다.

```python
from pathlib import Path
from drivable_query import DrivableMask

h5 = Path('/home/jang/2026-ASMC/src/asmc_hd_map/development/won_tae_link_node/out/link_node_map.h5')
road = DrivableMask.load(h5, layer='road', margin_m=0.0)
lane = DrivableMask.load(h5, layer='lane', margin_m=0.0)
stopline = DrivableMask.load(h5, layer='stopline', margin_m=0.0)

road.check_map('waize_katri_2025-10-23')
is_drivable = road.contains(points_map_xy)   # (N, 2) -> (N,) bool
is_lane = lane.contains(points_map_xy)
is_stopline = stopline.contains(points_map_xy)
```

위 예제는 `drivable_query.py`를 소비 패키지에 함께 둔 경우다. 원본 위치에서 직접
import하려면 실행 전에 `export PYTHONPATH=/home/jang/2026-ASMC/src/asmc_hd_map/development/won_tae_link_node:$PYTHONPATH`
를 적용한다. 입력은 반드시 map-local ENU여야 한다.
UTM 절대좌표를 넣는 경우 각 조회기에 `set_utm_input()`을 호출한다.

차선과 정지선은 폭이 얇기 때문에 차량 중심 한 점을 정확히 찍는 방식보다 벡터를 쓰는
편이 낫다.

- 차선: `observed_lane_points` + `observed_lane_offsets`, 색/side attrs
- 정지선: `stopline_points`, `stopline_dirs`, `stopline_headings`, `stopline_lengths`
- 주행가능영역: `/road` 래스터 또는 `road_mask.png`

판단제어가 nearest lane/stopline 거리, 진행방향, 신호등 ID까지 필요하면 위 벡터를
읽어 공간 인덱스를 구성한다. `stopline_ids`와 `stopline_traffic_lights` attrs가 정지선
행과 같은 순서다. 현재 정지선은 선택된 노드 8개가 7개 벡터 그룹으로 저장돼 있으므로
대회 전체 150개를 의미하지 않는다.

## 주행가능영역 최종 출력 (`/last_lidar`)

실시간 운용에서는 `Main`의 C++ 발행기가 트래킹된 객체를 ENU로 변환하고,
**주행가능영역 안에 있는 것만** 골라 최종 토픽으로 발행한다.

```
입력  /lidar/tracked_objects_map   Msg/TrackedObjectArray, frame_id = "map"
출력  /last_lidar                  같은 타입, frame_id = "map"
시각화 /last_lidar_markers         MarkerArray, frame_id = "velodyne"
```

객체 하나(`Msg/TrackedObject`)가 담는 값. 좌표/속도는 모두 `header.frame_id`
(= `map`, ENU) 기준이다.

| 필드 | 뜻 |
|---|---|
| `id` | 트래커 ID (프레임 간 유지) |
| `type` | 정수 코드. 1 = person, 2 = car/truck/bus, 3 = 그 외 |
| `classification` | 융합 정책 결과 문자열: `car` / `person` / `unknown` |
| `yolo_class` | 이 클래스를 확정시킨 YOLO 원본 라벨: `car`/`truck`/`bus`/`person`, 카메라 근거가 없으면 `unknown` |
| `is_dynamic` | 동적 객체 판정 |
| `age` | 연속 추적된 프레임 수 |
| `miss_count` | 미관측 프레임 수 (`/last_lidar` 는 항상 0) |
| `cluster_center` | 클러스터 중심점 (x, y, z) |
| `obb_center` | OBB 중심점 |
| `obb_corners[8]` | OBB 꼭짓점. 0–3 = 아랫면(zMin), 4–7 = 윗면(zMax) |
| `obb_yaw` | OBB 긴 축의 방위각 [rad], +x 기준 CCW |
| `velocity_mps` | 절대 속도 벡터 [m/s] (기준값) |
| `speed_kph` | 위 벡터의 크기 [km/h] (표시·검증용) |
| `length` / `width` / `height` | OBB 치수 [m] |

```bash
cd src/lidar+sensorfusion
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch Main main.launch
```

| 파라미터 | 기본 | 뜻 |
|---|---|---|
| `~drivable_margin_m` | **1.0** | 마스크를 이만큼 팽창시켜 여유를 준다 |
| `~drivable_ratio_threshold` | **0.3** | OBB 내부 비율이 이 값 이상이면 남긴다 |
| `~drivable_samples` | 5 | OBB를 5×5 격자로 훑는다 |
| `~drivable_map_id` | `waize_katri_2025-10-23` | 판본 확인. 다르면 `Main` 시작 실패 |
| `~dynamic_speed_kph` | **5.0** | 동적 객체 판정 속도 |
| `~enable_sensor_fusion` | **false** | 카메라 클래스 퓨전과 FusionLiDAR 실행 여부 |

> ⚠️ `main.launch`의 현재 기본 `drivable_margin_m=1.0`은 최종 road를 로드한 뒤
> 메모리에서 1 m 팽창시킨다. 그래서 베이크 파일에는 정확히 잘린 구조물도 라이다
> 런타임에서는 다시 포함될 수 있다. 정확한 경계를 그대로 적용하려면
> `main.launch` 값을 `0.0`으로, 작은 측위 여유만 필요하면 `0.2~0.3`으로 바꾼다.

`drivable_filter_node.py`는 C++ 결과와 대조하는 독립 구현으로 남겨 둔다. 실시간
운용 중 함께 실행하면 `/last_lidar` publisher가 둘이 되므로 동시에 실행하지 않는다.

### 좌표 변환이 없다

라이다의 `map` 프레임은 [Parser.cpp](../../../lidar+sensorfusion/src/Main/src/Parser/Parser.cpp)
가 UTM52N 에서 `eastOffset`/`northOffset` 을 뺀 ENU 이고, 그 offset 을
[morai_sensor_bridge.launch](../../../asmc_bridge/launch/morai_sensor_bridge.launch)
가 `302595.0` / `4124145.0` 으로 넣는다 — 구운 맵의 `local_origin_in_global` 과
**같은 값이다.** 그래서 좌표를 그대로 넣는다.

실주행 중 확인한 값이다:

```
$ rostopic echo /gps -n1 | grep -i offset
eastOffset: 302595.0
northOffset: 4124145.0
```

그때 실차 위경도 `(37.240995, 126.774368)` 를 맵 로컬로 옮기면 `(-9.63, -216.69) m`
이고 구운 주행가능영역 **안**이다. 노드는 첫 프레임에서 객체가 맵
bounding range에 드는지만 점검하고 어긋나면 로그로 알린다
(`좌표 범위 확인 OK` / `객체 좌표가 맵 범위 밖입니다`). 이 검사는 큰
원점 불일치를 찾는 후리스틱이며, 맵 범위 안의 GPS 점프나 수 m 오차를
증명하지는 않는다.

### 판정은 OBB 꼭짓점으로 한다

메시지의 `obb_corners[0..3]` (하층 4개, 상층은 4..7) 은 **이미 map ENU 로 개별
변환된 값**이다. 그래서 중심+yaw 로 박스를 다시 만들지 않는다 — yaw 규약(라디안/도,
법선 방향)이 어긋날 여지를 없애기 위해서다. `obb_yaw` 는 **라디안**이라 `yaw_deg` 를
받는 API 에 그대로 넣으면 안 된다.

꼭짓점 순서는 가정하지 않는다 (`quads_inside_ratio` 가 중심 기준 각도로 다시 정렬).
두 방식이 같은 답을 내는지는 회귀로 붙잡아 뒀다
(`test_quad_ratio_matches_center_yaw_and_ignores_corner_order`, 무작위 OBB 50개에서
차이 0.000000).

### OBB 비율 판정과 기존 1.0 m 여유의 의미

느슨한 방식(중심+네 귀퉁이 5점 중 하나라도 걸치면 살림)과 비교해 실측했다. 가장 긴
직선 링크에서 도로 경계를 찾고 소형차(4.6×1.9 m)를 옮겨가며 잰 값이다.

| 차 중심 위치 | 5점 느슨 | 25점 비율 | 임계 0.3 | 임계 0.5 | 임계 0.8 |
|---|---|---|---|---|---|
| 도로 중앙 | 살림 | 1.00 | 살림 | 살림 | 살림 |
| 경계 안쪽 0.5 m | 살림 | 0.72 | 살림 | 살림 | **버림** |
| **경계 바로 위** | 살림 | **0.40** | 살림 | **버림** | **버림** |
| 경계 밖 0.5 m | 살림 | 0.20 | **버림** | 버림 | 버림 |
| 경계 밖 1.0 m | 버림 | 0.00 | 버림 | 버림 | 버림 |

(여유 없이 `margin_m: 0` 으로 잰 값)

세 가지가 여기서 나왔다:

1. **임계 0.5 이상은 쓰면 안 된다.** 도로 최외곽에 세워진 차가 비율 **0.40** 밖에
   안 나온다 — 이 맵은 갓길이 없고 `outer_trim_m` 으로 좌우 0.3 m 를 더 깎았기
   때문이다. 0.5 로 두면 **실물 정차 차량을 버린다.**
2. **느슨 vs 비율의 차이는 0.5 m 폭 띠 하나뿐이다.** "경계 밖 0.5 m" 한 줄만 갈린다.
   방식 선택보다 `margin_m` 이 훨씬 크게 흔든다.
3. **그래도 비율을 쓰는 이유는 크기 정규화다.** 5점 방식은 **박스가 클수록 귀퉁이가
   멀리 뻗어 도로에 닿기 쉽다** — 벽이나 건물을 오검출한 거대한 OBB 가 귀퉁이 하나로
   통과한다. 비율은 박스 크기로 나누므로 이 문제가 없고, 0.2 m 격자에서 귀퉁이가
   픽셀 하나에 걸리는 운에도 흔들리지 않는다.

`margin_m: 1.0`을 함께 쓰면 경계 밖 1.0 m 객체까지 살아남는다. 이는 객체 누락을
줄이는 보수적 설정이지, 베이크 경계를 정확히 따르는 설정은 아니다. 외곽선과 구조물
제외 결과가 중요한 운용에서는 margin을 낮춘 뒤 실제 측위 오차로 재검증한다.

### 실제로 돌려서 확인한 것

격리된 마스터(실시스템과 분리)에 노드를 띄우고 합성 객체를 쏴서 확인했다:

```
[drivable_filter] map_id=waize_katri_2025-10-23 shape=(7232, 3085) ppm=5.0
                  margin=1.0 m  threshold=0.3  samples=5x5
[drivable_filter] /lidar/tracked_objects_map -> last_lidar
[drivable_filter] 좌표 범위 확인 OK — 객체 6개가 맵 범위 안입니다.
[drivable_filter] 1 프레임  객체 6 -> 4 (버림 2)
```

| 위치 | 내부 비율 | 기대 | `last_lidar` 실제 |
|---|---|---|---|
| 도로 중앙 / 경계 안쪽 0.5 m / 경계 위 / 경계 밖 1.0 m | 1.00 / 1.00 / 0.92 / 0.56 | 살림 | **살림** |
| 경계 밖 2.0 m / 5.0 m | 0.08 / 0.00 | 버림 | **버림** |

MORAI 기록 bag의 `/lidar/tracked_objects_map` 398프레임을 원 속도로
재생한 종단 검증에서는 입력 398프레임/객체 781개가 출력
398프레임/객체 540개가 되었고, 프레임 유실은 없었다.
다만 이 bag의 GPS에는 연속 샘플이 최대 46.25 m 점프한 구간이 있어,
이 결과는 ROS 종단 전송과 필터 실행을 검증한 것이지 keep/drop의 실세계
GT 정답률을 검증한 것은 아니다.

### 알고 쓸 것

- **기하를 못 믿는 객체는 버리지 않고 남긴다.** 꼭짓점이 비유한값이거나 한 점으로
  모인 축퇴 박스는 비율이 무의미하므로 통과시키고 로그에 센다. 놓치는 쪽보다 남기는
  쪽으로 기울인 것이다.
- **입력은 이미 정제되어 있다.** `MakePlannerObjects` 가 `miss_count != 0`(KF 추정만
  된 객체), 노이즈 클래스, 빈 클러스터를 발행 전에 걸러낸다. 최대 200개
  (`~map_output_max_objects`).
- **점수를 메시지에 실으려면** `TrackedObject.msg` 에 필드를 추가해야 하고 그건
  `Msg` 패키지 수정 → 중첩 워크스페이스 재빌드다. 지금은 넣지 않았다.
- `/lidar/tracking_markers_map` 은 같은 객체의 RViz 표시용이다. 필터 입력으로는
  `/lidar/tracked_objects_map` 하나만 쓴다.
- **하드 게이트로 쓰지 말 것** 은 여전히 유효하다 (위 "반드시 알아야 할 것" 5번).
  이 노드는 계산 범위를 좁히는 용도다.

## 구조

```
src/asmc_hd_map/development/won_tae_link_node/
├── bake_map.py                 CLI (로드 → 베이크 → 지표 → H5/PNG)
├── live_check.py               MORAI 실주행 대조 (Ego UDP / ROS / demo)
├── config/waize_katri_runtime.yaml     소스 경로와 레이어 파라미터
├── data/observed_lanes/        베이크에 사용하는 실측 차선 CSV 19개
├── out/
│   ├── link_node_map.h5        최신 운영 H5
│   ├── road_mask.png           C++ LiDAR가 직접 읽는 최종 road
│   └── road_mask.yaml          PNG 좌표·해상도·map-id 메타데이터
├── mapbake/
│   ├── io_mgeo.py              JSON 로더 + 스키마 정규화
│   ├── topology.py             리샘플·법선·heading, 노드→링크 역인덱스
│   ├── raster.py               좌표 규약, 그리기, 구멍 메우기
│   ├── layers.py               road / lane / stopline 베이크
│   ├── observed_lane.py        실측 CSV 보정·연결·road 후처리
│   └── writer.py               H5 입출력
├── drivable_filter_node.py     주행가능영역 밖 객체를 걸러 last_lidar 로 발행
├── drivable_query.py           H5 래스터 조회 API
└── tests/                       베이크·실측 차선 회귀테스트 101개
```

`topology.py` 가 세 레이어의 공통 기반이다 — 차선은 법선, 정지선은 노드 heading 이
필요하고 둘 다 같은 코드를 쓴다.

## 의존성

`numpy`, `opencv-python`, `h5py`, `pyyaml` (+ `--source gps` 에 `pyproj`)
