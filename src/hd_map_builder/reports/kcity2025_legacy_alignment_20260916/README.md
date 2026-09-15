# K-City 2025 ↔ legacy KATRI 정합 audit

> 상태: 분석 전용, 운영 맵 미변경
> 설정: `config/kcity2025_legacy_transfer.yaml`

## 결론

구 MGeo는 현재 topology를 대체할 수는 없지만, 고주로를 시작으로 검증된 구간의 road
polygon과 lane 속성을 보강하는 데 사용할 수 있다. 링크 이름은 대응 기준이 아니며 선언된
origin 변환 뒤 geometry·heading·route station으로 many-to-many 대응한다.

## 좌표와 전역경로

- 입력: 주최측 제공 `R_KR_PG_KATRI/2026_molit_comp_global_path.txt`
- 역할: 주행 corridor와 route station 기준. 차선 경계·차로폭 GT로는 사용하지 않음
- legacy→current local translation: `[-135.058, -1509.463, +28.991]m`
- 별도 fitted yaw/scale: 사용하지 않음
- 경로 길이: 2,184.61m
- 1m·15° 기준 재사용 구간: 1,959.66m, 89.70%
- 안정 구간 XY residual median: `[-0.039, -0.059]m`
- XY residual norm p95: 0.575m
- Z residual median: +0.102m

가장 큰 rebuild 구간은 `s=212.96~341.91m`, 좌표 약
`(-91.8,-251.8)→(-59.9,-129.4)`이다. 상세 구간과 link 대응은
`route_intervals.csv`에 있다.

## 고주로 pilot

현재 전역경로에서 `A2256W000411` 시작부터 `A2256W000153` 끝까지는
`s=1118.74~1741.59m`다. 주행 lane만 고르는 대신 좌우 15m 안의 방향이 비슷한 현재
링크 32개를 corridor로 잡았다.

- legacy road mesh vs current link-width road proxy IoU: 0.942
- road proxy가 legacy mesh 안에 들어가는 비율: 0.997
- lane 후보: 64개
  - tier A 56, tier B 8
  - white 49, yellow 15

현재 관측과 구 lane boundary 비교:

| 파일 | 수집 방식 | 0.5m 이내 | 1.0m 이내 | 판정 |
|---|---|---:|---:|---|
| `hi_en_left` | LiDAR Semantic | 99.2% | 100% | 강한 검증 anchor |
| `hi_en_right` | LiDAR Semantic | 100% | 100% | 강한 검증 anchor |
| `hi_middle_left` | GPS_DIRECT | 91.7% | 100% | 후보 |
| `hi_middel_right` | GPS_DIRECT | 100% | 100% | 후보 |
| `hi_right2/3` | GPS_DIRECT | 100% | 100% | 후보 |
| `hi_right` | GPS_DIRECT | 76.1% | 91.2% | 부분 구간 검토 |
| `hi_left` | GPS_DIRECT | 12.5% | 13.2% | 한 차로 폭 오대응 가능성, 제외 |

따라서 다음 단계에서 `hi_en_left/right`를 우선 anchor로 쓰고 `hi_left`는 자동 승격하지
않는다.

## 일반화하면 안 되는 구간

- `fourth_point`: legacy lane 1m 이내 0%, median 11.62m
- `turnel_left`: 1m 이내 34.5%, median 3.25m
- 음영 tail road mesh IoU: 0.744. legacy polygon이 current link proxy보다 넓다.
- 가장 큰 rebuild corridor road IoU: 0.273

음영구간은 기존 camera shared-family/current observation을 우선하고 legacy lane을 정답으로
사용하지 않는다.

## 부가 레이어

안정 link에 연결된 1차 후보 수는 surface marking 167/238, traffic light 56/81,
traffic sign 63/66이다. 경로 15m 이내 geometry는 각각 69, 15, 26개다. 횡단보도는 경로
15m 이내 17개지만 legacy `link_id_list`가 비어 있어 geometry/manual 검증이 필요하다.

이 수치는 물리 asset이 현재 버전에도 존재한다는 뜻이 아니다. 현재 link/node에 새로
연결하고 시뮬레이터에서 확인하기 전까지 모두 candidate다. 속도·신호 schedule과 current
topology는 legacy에서 이전하지 않는다.

## 산출물 역할

| 파일 | 역할 |
|---|---|
| `summary.json` | source hash, 좌표 계약과 전체 결과 |
| `link_correspondence.csv` | legacy link→current link many-to-many 후보 |
| `lane_transfer_candidates.csv` | legacy lane 전체 A/B/D 판정 |
| `route_intervals.csv` | route station별 reuse/rebuild 구간 |
| `observed_lane_alignment.csv` | 현재 수집 차선과 legacy 차선 거리 |
| `auxiliary_transfer_candidates.csv` | 노면표시·신호·표지·횡단보도 후보 |
| `high_speed_candidate_index.json` | 고주로 64개 lane 후보 index |
| `canonical_candidate_example.json` | 세 feature만 포함한 canonical schema 예제 |

다음 PR은 고주로 후보 overlay와 current observation 교차검증까지만 수행한다. 운영 H5,
`src/link_node`, planning interface는 변경하지 않는다.
