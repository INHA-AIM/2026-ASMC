# L2-A2 VLP16 지면분할: Patchwork++와 linefit 비교 (2026-10-04)

후속 Patchwork++ 높이 보호 비교는 [추가 검증](./l2a2-patchwork-vlp16-adaptation-20261004.md)을 참조한다. 아래의 원본 Patchwork++ 보류 결론과 구분한다.

상태: **이 평가 시점에는 운영 교체를 보류하고 linefit `90×60`을 다음 지면 backend 후보로 선정했다. 후속 Patchwork++ 후보는 위 추가 검증을 참조한다.**
원본 LiDAR·GT 라벨을 같은 ROI, 10cm voxel, pitch 수평화 조건에서
손재호 기준선, Patchwork++ 고정 높이, linefit으로 직접 재처리했다.
지면점 제거율만이 아니라 객체점 보존과 동일 Euclidean 군집의 완전성·순도를
함께 봤다. 이미 분석한 Autonav v3 데이터이므로 홀드아웃 성능이 아니다.

## 입력·소스·좌표 계약

- 입력은 `$ASMC_DATA/lidar_assessment/20261004_gt_dual_ground`의 raw
  4,048프레임과 동기 1,439프레임이다. `raw/snapshots.jsonl` SHA-256
  `0e4667775d9eaeacc1d11375ad7e3e34738d6f0605e12ce20ee64e1cab4f25c7`,
  `sync/snapshots.jsonl` SHA-256
  `9922abde16285180fd061f29fc56b03c5eac22ac7c88fcf8814f945f552a5787`.
  두 방식은 같은 센서·시뮬레이터 계열이며 독립적인 대회 환경 검증은 아니다.
- 비교 소스는 손재호 지면 코드 고정 기준 `6435af1`, Patchwork++ v1.4.1
  `3e6903a1d5`, linefit `e6b9418cc5756e30e4ea87b11d8792f936a2002a`.
  외부 checkout과 운영 LiDAR는 변경하지 않았다. 손재호의 실제 `Main`과
  점별 기준선은 이전 [실행 parity 검증](./l2a2-actual-main-gt-audit-20261004.md)의
  raw 216/216, 동기 191/191 프레임에서 교차 확인했다.
- `sensor_height=1.78/1.80m`는 **LiDAR 원점에서 평지 노면까지의 높이**다.
  Autonav v3 설정의 rear-axle `base_link` 기준 장착 z=1.45m와 다르다.
  공통 처리 입력은 실제 센서 원점의 점을 pitch 수평화한 중력축 정렬 좌표다.
  `map` 좌표로 지면분할하지 않는다. 후보의 공개 ROS 점군·OBB 출력은
  적용된 수평화를 되돌린 뒤 장착 변환을 한 rear-axle `base_link`다.
  원본 관측 stamp는 유지한다. 현재 운영 손재호 코드의 frame 혼용이
  자동으로 수정된 것은 아니며 후보를 판제에 연결하지 않았다.
- 기존 Patchwork++ `sensor_height=1.80m`, 1–60m, RNR OFF, 단일표본 높이
  갱신 수정·고정 높이 모드를 유지했다. 동기 1,439프레임 모두에서 이전
  Patchwork++ 결과와 ground mask가 정확히 일치했다. 1.78m 변경은 이
  데이터의 분류를 바꾸지 않았고, `th_dist=.08`과 TGR OFF도 차량 완전
  소실을 해결하지 못했다. 사전 높이를 맞추는 일은 필요하지만 충분하지 않다.
- linefit의 공식 기본 360 angular × 120 radial bin은 VLP16의 희소한
  단일 scan에서 지지점이 부족해 이 입력의 아스팔트를 1.6–4.2%밖에
  제거하지 못했다. 후보는 높이 1.78m, 90 angular × 60 radial bin,
  1–60m, `long_threshold=2m`, `max_dist_to_line=.05m`, 4 threads로
  고정했다. 경사·fit 허용치는 [평가기 코드](../../../tools/lidar/ground_method_review/vlp16/review.cpp)에
  명시돼 있다. `90×40/gap3`도 민감도 시험으로 남겼다. 장면별·GT별
  파라미터 변경은 하지 않았다.

## 같은 원점 라벨에서의 결과

수치는 **공통 ROI 안에서 GT semantic 원점이 ground로 분류된 비율**이다.
객체 하나의 여러 프레임 관측은 서로 독립 객체가 아니다. `완전 소실`은
10점 이상 보이던 GT instance가 지면 단계 뒤 0점이 된 관측 수다.

| 수집본·방법 | 아스팔트 제거 | 차량점 삭제 | 보행자점 삭제 | 일반 장애물점 삭제 | 차량 완전 소실 | 장애물 완전 소실 |
|---|---:|---:|---:|---:|---:|---:|
| raw 손재호 기준선 | 100.00% | 4.87% | 8.50% | 13.68% | 2/249 | 26/577 |
| raw Patchwork++ 1.80m | 95.51% | 26.52% | 3.17% | 11.28% | 30/249 | 65/577 |
| raw linefit 90×60 | 97.45% | **0.37%** | **1.64%** | **2.74%** | **0/249** | **1/577** |
| 동기 손재호 기준선, 마지막 239프레임 | 100.00% | 6.33% | 18.06% | 29.99% | 0/106 | 85/827 |
| 동기 Patchwork++ 1.80m, 동일 구간 | 98.50% | 8.64% | 7.61% | 17.46% | 1/106 | 81/827 |
| 동기 linefit 90×60, 동일 구간 | 99.33% | **1.23%** | **4.43%** | **6.40%** | **0/106** | **2/827** |

GT `StandingOBJ(109)`는 일반 구조물까지 포함할 수 있어 객체 성공률에
합치지 않았다. 미라벨 점을 자동으로 오검출이라고 세지도 않았다. raw와
동기의 클래스 구성은 달라 두 구간의 비율을 하나의 종합점수로 합치지 않는다.

동일한 후단 1.5m Euclidean 연결과 voxel 계보로 점→cluster를 추적한
**표본** 결과는 다음과 같다. 완전성 ≥80%와 순도 ≥90%를 동시에 만족한
관측만 `매칭`으로 셌다. 이는 운영 OBB·tracker 정확도가 아니다.

| raw 203프레임 표본 | 차량 매칭 | 보행자 매칭 | 장애물 매칭 | 전체 cluster |
|---|---:|---:|---:|---:|
| 손재호 | 9/13 | 15/22 | 12/27 | 1,051 |
| Patchwork++ | 4/13 | 15/22 | 12/27 | 2,156 |
| linefit 90×60 | **10/13** | 13/22 | **19/27** | **4,148** |
| linefit 90×40 | 12/13 | 14/22 | 16/27 | 3,432 |

동기 개발 구간 800–1199 표본의 차량 매칭은 손재호 68/92,
Patchwork++ 30/92, linefit 90×60 69/92였다. linefit의 남은 점이
`sync_001211`·`sync_001231`·`sync_001241`의 차량 cluster에 섞여
순도가 각각 약 85%, 72%, 89%로 떨어졌다. **지면 단계의 객체 보존은
개선됐지만 객체 출력 전체가 우세하다고 단정하지 않는다.** 남는 노면·보도·
구조물 조각을 지면 단계에서 무조건 없애려다가 객체를 자르는 대신,
다음 군집·객체 단계에서 완전성·순도와 함께 줄이는 것이 합리적이다.

## 처리시간과 판정

같은 C++ 벤치에서 공통 ROI·10cm voxel·pitch 준비와 지면분할만 분리해
측정했다. `p50/p95/p99` 단위 ms이다. linefit 숫자는 별도 VLP16 평가기의
0ms placeholder가 아니라 새 실제 계측값이다.

| 수집본·단계 | p50 | p95 | p99 |
|---|---:|---:|---:|
| 동기 공통 준비 | 0.796 | 1.064 | 1.240 |
| 동기 Patchwork++ 분할 | 0.429 | 0.531 | 0.637 |
| 동기 linefit 90×60 분할 | 0.519 | 0.682 | 0.822 |
| raw 공통 준비 | 0.562 | 0.874 | 1.000 |
| raw Patchwork++ 분할 | 0.429 | 0.555 | 0.691 |
| raw linefit 90×60 분할 | 0.513 | 0.706 | 0.839 |

linefit은 이 측정에서 약 0.09ms p50 느리다. 벤치는 동일 프로세스에서
Patchwork++ 다음에 linefit을 순차 실행하며, 캐시·CPU 부하 영향이 있을
수 있다. 두 숫자 모두 전체 ROS 지연이나 손재호 전체 `Main`의 처리시간과
직접 비교할 수 없다. 차량점 보존 개선에 비해 분할 비용 차이는 작지만,
cluster 수 증가에 따른 후단 비용은 아직 별도 실시간 검증이 필요하다.

**선택:** Patchwork++는 현재 입력에서 차량 완전 소실과 후단 차량 매칭
저하 때문에 보류한다. VLP16에 맞춘 linefit 90×60을 다음 비교·개발의
지면 backend 후보로 사용한다. 다만 기존 판제 출력 교체는 하지 않는다.
다음 단계는 linefit의 남은 점이 만든 차량 cluster 혼합을 독립적으로
줄이고, 새 장면·연속 주행에서 80/90 매칭과 CPU를 확인하는 것이다.
그때 실패하면 손재호 기준선을 rollback 대상으로 유지한다.

## 재실행과 산출물

평가기는 `tools/lidar/ground_method_review/`에 있다. 외부 소스의 고정
checkout과 기존 H1 생성 Patchwork++를 읽기 전용으로 빌드한다. 핵심
실행 형태는 다음과 같다. 기존 파일이 있으면 도구가 덮어쓰지 않는다.

```bash
cmake -S tools/lidar/ground_method_review/vlp16 -B /tmp/asmc-linefit-review \
  -DLINEFIT_SOURCE=/tmp/asmc_linefit_source
cmake --build /tmp/asmc-linefit-review -j4
/tmp/asmc-linefit-review/linefit_vlp16_review \
  /data/lidar_assessment/20261004_gt_dual_ground/sync/snapshots.jsonl \
  /tmp/sync_linefit_check.jsonl
```

전체 비교 산출물은 `$ASMC_DATA/lidar_assessment/20261004_gt_dual_ground/reports/`의
`sync_method_sweep_v3_summary.json` (SHA-256
`503a6ffa7aa0bcb84c3f3a5d5697b5ea6f54bb45c7eae8015075f15cc0138c00`),
`raw_linefit_vlp16_v1_summary.json` (SHA-256
`adbad9a14557d876f404d3a7ebeae7cd68dc3a9d06136e722268856bd910aa91`),
`sync_method_vehicle_cases.json`, `sync_ground_method_bench_v1.csv`,
`raw_ground_method_bench_v1.csv`에 있다. 그림·수치는 필터 라벨을 정답으로
사용하지 않고 GT point index와 instance 대응에서 계산했다.

## 참고 기법의 역할

- [linefit](https://github.com/lorenwel/linefit_ground_segmentation)은
  방위각 sector와 거리 bin의 낮은 점을 선으로 피팅하는 지면분할기다.
  센서의 실제 지상 높이와 중력축 정렬, 빔 밀도에 맞는 bin 설정이 중요하다.
- [TRAVEL](https://github.com/url-kaist/TRAVEL)은 그래프 기반 traversable
  ground와 above-ground 객체 분할을 함께 제공한다. 이번에는 지면 backend
  하나만 바꾸고자 했고 VLP16 투영 조건도 추가 검증이 필요해 비교군에
  넣지 않았다.
- [SimpleTrack](https://github.com/tusen-ai/SimpleTrack)은 3D 객체
  tracking baseline이며 지면 제거 방법이 아니다. 후속 ID·생애주기
  안정화 참고자료로 유지한다.
