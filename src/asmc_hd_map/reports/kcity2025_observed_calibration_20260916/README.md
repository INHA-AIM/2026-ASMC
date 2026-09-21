# K-City 2025 원태 관측선 기반 경계 보정 후보

이 디렉터리는 `calibrate_observed_lanes.py`가 만든 **2단계 비운영 후보**다. 1단계
`nominal_candidate_map.json`의 SHA-256
`209a0d06fdecaa570c6364a791b7b3f2a5404dc1e8f570d9bb54841fcfe57b9a`를 입력 계약으로
고정했다. 구 KATRI MGeo, H5/raster와 수동 bridge는 사용하지 않았다.

## 입력과 적용 정책

- CSV 19개, 총 26,511행
- fit support: 직접 관측 25,414행
  - `current_lidar_semantic`: 13,679행
  - `current_gps_direct`: 11,735행
- audit-only: 고정 폭 생성 958행, 시간 보간 139행
- 기존 raster 표시용 `inward_offset_m=0.03`: 미적용
- 기존 수동 bridge: 미적용
- CSV `side`: 수집 차량 기준 audit 값으로만 보존
- 선 색상: 검수된 source별 side-color override 또는 원본 CSV
- 선 형태: 아직 `unknown`

## 정합 결과

직접 관측을 시간·공간 gap으로 60개 segment로 나누고 0.25m 간격으로 재표본화했다.
각 점은 명목 경계 선분에 정확히 투영하며, 현재 link topology로 이어진 경계 사이에서만
전환할 수 있다.

| match run 상태 | 개수 | 의미 |
|---|---:|---|
| `automatic` | 76 | 모든 자동 gate 통과 |
| `review_required` | 74 | 완화 gate만 통과하거나 일부 기준 미달 |
| `rejected` | 47 | 완화 gate도 미달 |
| `unmatched` | 15 | 호환되는 명목 경계 없음 |

자동 run은 총 5,766.5m의 관측 조각이며, 1m station bin 3,734개가 conflict 없이
적용됐다. 이는 물리 경계의 중복 관측 길이가 포함된 합계이므로 route coverage로
해석하면 안 된다. `review_required/rejected/unmatched` geometry는 candidate map에
적용하지 않고 CSV와 overlay에만 보존했다.

## 후보 map

- 기존 link centerline: 70개 그대로 유지
- 기존 stopline 후보: 13개 그대로 유지
- parent lane boundary: 85개
- 관측 보정 parent: 50개
- `observed_calibrated` interval: 60개
- `link_nominal` 잔여 interval: 134개
- 양쪽 자동 보정이 겹친 실제 폭 sample: 3,143개
  - 중앙값 3.468m
  - p05/p95 3.276/3.803m
  - 전체 범위 1.755~4.349m

폭의 낮은 꼬리는 합류·소멸부 taper 후보를 포함한다. 이 수치를 일반 차로폭으로
승격하거나 current link의 3.5m 속성을 덮어쓰지 않는다. 한쪽만 관측된 곳은 offset만
기록하며 폭을 생성하지 않는다.

## 파일

| 파일 | 용도 |
|---|---|
| `summary.json` | 입력 계약, match와 후보 개수 |
| `observed_candidate_map.json` | provenance와 parent station을 가진 vector 후보 |
| `source_audit.csv` | 19개 source의 전체 행 분류와 hash |
| `segment_matches.csv` | match run별 gate 수치와 미적용 이유 |
| `boundary_offset_profiles.csv` | 1m signed-normal offset, source와 conflict 상태 |
| `link_width_profiles.csv` | 양쪽 자동 보정이 겹친 구간만의 폭 표본 |
| `overlays/full_corridor.png` | 전체 명목·보정·원관측 비교 |
| `overlays/high_speed.png` | 고주로 확대 비교 |
| `overlays/shadow_and_static.png` | 음영·정적 장애물 context 비교 |
| `overlays/review_required.png` | 수동 검토가 필요한 관측 중심 overlay |

## 해석 제한과 다음 gate

이 산출물과 모든 feature는 `candidate`, map은 `operational=false`다. 색상은 source
규칙을 보존했지만 shape는 확정하지 않았다. 교차로·짧은 link에서 명목 경계가 없다고
관측으로 새 경계를 만들지도 않았다.

다음 단계는 overlay에서 자동 적용과 `review_required`를 먼저 수동 검수하는 것이다.
그 뒤 current map과 실제 도로가 안정적으로 같은 구간에만 구 KATRI의 선 형태·누락
geometry를 보조 이식한다. 정지선 도색 보정, runtime crop과 판제 연결은 별도 단계다.
