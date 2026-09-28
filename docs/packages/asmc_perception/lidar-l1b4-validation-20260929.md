# L1-B4-A — 카메라 의존성을 낮춘 벽·장애물 분리 후보

**상태: 격리 구현·비교 완료. 합성 진급 기준 통과, 실제 구조물 조각 문제로 운영 미채택.**
손재호와 같은 시점에 개발하는 것을 전제로 하지 않고 독립 브랜치에서 비교한다.
이 기록은 [L1-B3-G](./lidar-l1b3g-validation-20260928.md)를 잇고,
[공통 소비 계약](../../contracts/perception/lidar-planning-consumer-v1.md),
[기준 manifest](../../../config/baselines/lidar_l1b4_20260929.yaml),
[재실행 도구](../../../tools/lidar/tunnel_geometry/README.md)를 연결한다.

## 비교 기준과 변경 경계

`c536c3c`에서 `feat/lidar-l1b4-geometry`를 만들었다. 손재호
`6435af10671235e735e4a9ed1bd747bd8d8389ac`은 immutable reference다.
공통 main `1ce434d`, 서준 `8b3f0b1`, 이전 L1-A/B1/B2/B3 결과를 덮어쓰지 않는다.
운영 LiDAR·bridge·camera·planner·message MD5·센서·HD맵 geometry는 바꾸지 않았다.
후보 source는 `tools/lidar/tunnel_geometry/`이고 운영 ROS publisher는 없다.

카메라 결과는 YOLO11m `/YOLO/BBox`이며 TwinLite/신호등 결과가 아니다.
`baseline` 즉시 매칭을 주 비교로 쓴다. 25ms 대기나 새 모델을 동시에 넣지 않았다.
roll/pitch 수평화된 기존 sensor-origin geometry를 유지해 삭제 정책만 비교한다.
이 좌표를 raw velodyne, rear-axle base_link, map으로 간주하지 않는다.
원본 stamp는 int64 ns로 보존하며 bridge stamp를 실제 광선 측정 시각으로 승격하지 않는다.

## 후보의 의미

원래 ROI/voxel, leveling, ceiling, ground, wall fit/persistence 결과를 공통 입력으로 고정한다.
build 디렉터리의 pinned header에 friend 한 줄만 추가하고 algorithm body는 그대로 컴파일한다.
벽 단계 이전 label을 복원하여 새 삭제 정책을 적용한 뒤 원래 군집화·L-shape OBB·OBB 제거를
호출한다. 따라서 현재 결과는 **같은 벽 모델에서의 삭제 정책 비교**다. 새 wall estimator나
실제 tracker의 성능을 평가한 결과는 아니다.

| 모드 | 처리 |
|---|---|
| baseline | pinned 즉시 매칭·FOV ambiguity fallback |
| policy_only | 빈 배열만 미수신처럼 취급; 넓은 FOV 보존의 대조군 |
| geometry | fit 잔차·관측 지지 구간·돌출부 연결성으로 벽/전경/미확정 분리 |
| candidate_off | 후보 OFF의 실제 guard를 거쳐 원래 label과 군집/OBB 재현 |

벽의 법선은 센서 쪽으로 통일한다. `median(abs(residual)) + 3×1.4826×MAD`를 5~15cm로
제한하고, 전경 연결은 20cm, 국소 확장은 최대 30cm의 one-hop이다. 확장은 원래 전경 seed에서만
가능하고 `signed residual >= band/2`가 필요해 벽 표면으로 전파하지 않는다. 이 가설값과 기존
wall/ground/cluster/OBB 파라미터를 실험 중 바꾸지 않았다. 짧은/희소 component를 보호 mask에서
제거하지 않지만 마지막 cluster 최소 10점과 OBB 필터는 그대로다.

카메라는 valid boxes, valid empty, unsupported-only, invalid boxes, missing, timeout,
time-window 초과, invalid stamp, protection OFF를 구분한다. 메시지 선택·지원 class·실제
보호점 수를 따로 기록한다. 빈 결과는 free-space가 아니다. camera class와 geometry mask를
합쳐 car/person class로 위장하거나 후단 OBB 보호 조건을 우회하지 않았다.

## 입력과 평가 지표

기존 36조건/180 frame을 원본 float4 내용과 대조해 보존하고 추가 80조건/400 frame을 만들었다.
총 116조건/580 frame이다. 추가 조건은 양쪽 벽, car/person/low-unknown, 벽만 있는 장면,
step/curve, 낮은 floor, roll/pitch, 부분 가림, 동일 점군의 camera 상태별 변형이다.
first-return은 16 elevation·0.25° azimuth analytic rays의 첫 교차면만 남긴 입력이며
MORAI/VLP16의 실제 발사 순서·노이즈·채널 모델을 재현한 것은 아니다.

GT는 별도 truth.jsonl에만 있고 후보에는 전달하지 않는다. wall-stage 객체 점 보존의 분모는
지면·천장 단계 이후 남은 GT 점이다. upstream 삭제를 이번 wall 손실로 합산하지 않는다.
final object의 completeness/purity와 OBB 관측점 포함률을 따로 계산하며 cluster 수를 recall로
바꾸지 않는다. voxel 이후 index에서 cluster/OBB로 이어지고 raw→voxel int32 mapping을 별도
artifact로 저장한다. ROI 제외는 -1, voxel 병합은 동일 index로 표시한다.

고정 진급 기준은 객체 점 보존 95%, final completeness 80%/purity 90%, 미수신 기준 대비
잔여 벽 50% 감소, 기존 성공 객체 추가 소실 0이다. negative camera 상태의 삭제 mask 차이,
OFF/historical parity, 미래 pose·stamp·intensity·point lineage 오류와 반복 차이는 모두 0이어야 한다.
이 기준은 격리 합성시험의 진급 기준이며 전 미션/실주행 수락을 뜻하지 않는다.

## 확정 실행 결과

평가 코어/driver revision은 `86c6891`이다. 합성 580 frame의 두 실행 모두 진급 기준을
통과했고 geometry digest가 일치한다. 기존 2,562 scan 중 1,011개를 평가하고
1,551개는 GPS/filter 비활성 조건으로 건너뛰었다. 기존 evaluator와 OFF parity,
stamp·intensity·점 수·raw lineage·미래 pose 사용 오류는 모두 0건이다.
합성 negative camera 35개 그룹의 삭제 mask 차이와 기존 성공 객체의 추가 소실도 0이다.
시간 계측값을 제외한 geometry SHA-256은 각각 다음과 같다. 실제 bag도 반복 일치했다.

- 합성: `79168f9756efd9c2b2572243f44ef8a75c458ee1df1fa6b7a94492a64349671f`
- Autonav: `834ba5937450cc4d4cb175b37a17365577818557b02aa38b1f2f34d2212ab5d6`

핵심 세 사례는 wall-stage 분모이며 각 행의 조건은 거리 15m·벽 gap 0.1m다.
누락 camera 상태에서 남은 기존 점을 고정한 비교다.

| 객체 | 분모/보존점 | 후보 final completeness/purity | 잔여 GT 벽점 대조군→후보 | 최종 출력 해석 |
|---|---:|---:|---:|---|
| car | 1,569/1,569 | 100%/100% | 2,994→0 | 기존 길이 25m·purity 51.2% 합침에서 길이 3.9m로 분리 |
| person | 248/248 | 100%/100% | 2,994→0 | 기존 wall-attached OBB 거부에서 0.5×0.45×1.4m 객체로 보존 |
| low-unknown | 64/64 | 100%/100% | 2,994→0 | 기존 벽 cluster OBB 거부에서 0.75×0.60×0.10m 관측 box로 보존 |

세 후보 OBB는 해당 관측 target 점을 100% 포함한다. 낮은 물체의 관측 box는 전체 물리
치수를 복원한 결과가 아니다. 기존 ground-residual flag는 true이나 제거 옵션은 false이며
이를 이번에 바꾸지 않았다. 전체 fixture의 accepted-target 조건 수는 21→72이지만
wall-only/무반환/카메라 변형을 포함한 수이므로 검출 재현율로 해석하지 않는다.
희소 low-unknown 16조건은 raw target 반환점 자체가 0이고 검출 성공으로 세지 않았다.

### 실제 bag에서 드러난 실패와 미채택 이유

유효 입력의 camera 상태는 valid-empty 407, 50ms 창 밖 604였다. 이 입력에는
유효 양성 box에 의한 실제 객체 보호 검증 근거가 없다. 아래 924개 정식 측정 scan의
벽 제거 수는 **필터 label 수**이고, 벽 GT/정확도는 아니다.

| 구간/scan | 기준선 최종 객체/평균 | empty→missing 대조군 | 기하 후보 | 벽 label 제거 기준선→후보 |
|---|---:|---:|---:|---:|
| A126 진입/463 | 324/0.70 | 501/1.08 | 8,976/19.39 | 2,666,882→2,653,672 |
| A126 중간/461 | 280/0.61 | 475/1.03 | 7,866/17.06 | 2,734,380→2,764,852 |

후보 최종 객체 16,842개 중 14,861개에 원래 `wall_attached` flag가 있다.
OBB width 중앙값 0.216m, height 중앙값 2.436m이며 벽 지지 구간 안의
중심 16,800개 중 16,791개가 해당 평면에서 0.3m 안에 있다.
대표 scan `bag_scan_2411`은 출력 2→21개, `bag_scan_3551`도 벽 주변의
좁은/높은 OBB 25개를 보인다. BEV·측면 그림에서 확인한 구조물 조각 위험이다.
정답 없는 count를 오탐률로 계산하거나, 모든 증가 객체가 벽이라고 확정하지 않는다.

기존 OBB 규칙은 wall-attached의 길이 8m 이상 또는 lateral wall의 길이 15m 이상과
aspect ratio 6 이상을 함께 요구한다. 분리 후 짧아진 조각은 이 조건을 피할 수 있다.
residual band·연결성만으로 구조물의 단차/국소 평면 오차와 장애물을 구분하기에는
근거가 부족하다. 4,483개 객체의 ground-residual flag도 진단이지 삭제 근거가 아니며,
이를 일괄 활성화하면 낮은 실제 물체를 다시 잃을 수 있다. 이번에 OBB/ground 규칙이나
가설 파라미터를 조정해 이 결과를 숨기지 않았다.

**판정:** 합성 조건에서는 점 보존과 벽 분리를 함께 개선했으나 실제 입력의 안정적인 객체
출력은 수락하지 못했다. 후보는 후속 평가용으로 보존하고 운영에는 채택하지 않는다.
정지 bag으로 연속 추적·속도·전 미션 안전성도 판정하지 않는다.

### 오프라인 계측

주 실행은 세 replay process를 동시 실행했고 이후 실제 bag 반복도 추가했다.
동일 GPU/ROS 전체 stack 부하나 live scheduling 계측이 아니다. 아래는
measurement-tag frame의 monotonic CPU 시간(ms)이며 실제 전달 지연으로 바꾸지 않는다.

| 입력/단계 | p50 | p95 | p99 |
|---|---:|---:|---:|
| Autonav 추가 기하 보호 | 1.580 | 2.053 | 2.436 |
| Autonav 후보 군집·OBB 재실행 | 0.986 | 1.347 | 1.660 |
| Autonav 후보 filter 추정 | 3.784 | 4.519 | 4.976 |
| Autonav 기준선 filter | 4.023 | 4.523 | 5.179 |
| Autonav frozen result 복사 | 0.004 | 0.008 | 0.014 |
| Autonav raw lineage+artifact 기록 | 4.139 | 4.586 | 5.153 |
| 합성 추가 기하 보호 | 1.651 | 3.357 | 4.236 |
| 합성 후보 filter 추정 | 4.037 | 17.815 | 18.736 |

filter 추정은 공통 수평화/지면/천장/벽 fit과 후보 정책·군집/OBB의 합이다.
ROI/voxel, camera 보호점 계산, tracker·ROS publish·동시 GPU 작업·대기 시간은 별도다.
lineage 시간은 평가용 대응표 계산/기록 비용이고 운영 필터의 추가 비용이 아니다.
작은 cluster에서 연산이 줄었다고 더 안전한 결과라고 해석하지 않는다.

## AIM 보조 기하 입력

기존 감사에서 raw/ROS XYZI가 검증된 7 campaign의 중간 표본 하나씩을 재확인했다.
원본 SHA, raw FLU, 해당 sensor profile의 `(1.00,0,1.48m)` mount를 확인하고 declared AIM base로
한 번만 rigid transform·inverse roundtrip을 계산했다. FRU를 추가 적용하지 않았다.
Autonav `(0.80,0,1.45m)`를 대신 넣거나 동일 datum으로 단정하지 않았다.

7개 모두 표본 형식·변환 점검 대상이다. 선택 cloud의 attitude가 기존 감사에서 승인되지 않았고
AIM declared base↔rear-axle 등가성도 독립 확인되지 않아 **filter replay admission은 0**이다.
원본을 복사하지 않았고 camera4·미승인 object GT·우리 route station을 사용하지 않았다.
다음 AIM filter 단일-frame 실험은 session별 attitude/기준점을 확보한 뒤 진행한다.

## 재현 산출물과 오류 기록

산출물은 `$ASMC_DATA/lidar_assessment/20260929/`에 있고 원본 bag/AIM은 그대로다.

| 경로 | 역할 |
|---|---|
| l1b4_fixtures_v1 | 580 frozen snapshots, truth, 원본·생성 cloud SHA |
| l1b4_synthetic_v4 | 확정 코드의 주 합성 비교 |
| l1b4_synthetic_repeat_v2 | 동일 합성 입력 반복 결정성 |
| l1b4_autonav_v2 | 기존 2,562 scan snapshot의 immediate 비교/historical parity |
| l1b4_autonav_repeat_v1 | 실제 bag 동일 입력 반복 |
| l1b4_aim_support_v1 | 7개 AIM 표본의 rigid transform/미승인 이유 |
| l1b4_synthetic_figures_v1 | 핵심 세 사례의 baseline/대조군/후보 BEV·측면 PNG/SVG |
| l1b4_autonav_figures_v1 | bag_scan_2411/3551의 실제 구조물 조각 그림 |
| l1b4_autonav_fragments_v1 | 최종 OBB 형태·wall-attached flag·지지 구간 근접도와 분석 SHA |

각 run manifest는 source commit, source/config/binary/**policy shared library** SHA와
입력 cloud SHA를 시작에 고정한다. 결과 JSONL·derived XYZI·raw lineage artifact SHA와
시간을 제외한 geometry digest도 저장한다. input/source가 실행 중 바뀌면 실패한다.
재실행과 figures/verification 명령은 도구 README에 있다.
원본 bag은 baseline manifest의 상대 경로·SHA로 연결하며 복사하지 않는다.

검증은 Python 38개(기존 기준 검사 포함), camera-sync 경계/도착/FIFO 18개,
C++ geometry-policy CTest 대상 1개를 통과했다. CTest 대상 안의 여러 assertion을
독립 테스트 수로 부풀리지 않는다. L1-A/B1/B2/B3의 고정 근거 검증과 문서 링크/이관
검사도 유지했다. `git diff c536c3c -- src scripts config/morai`는 비어 있다.

```bash
python3 -B tools/lidar/tunnel_geometry/verify.py --data-root ../data \
  --run ../data/lidar_assessment/20260929/l1b4_synthetic_v4 \
  --run ../data/lidar_assessment/20260929/l1b4_synthetic_repeat_v2 \
  --run ../data/lidar_assessment/20260929/l1b4_autonav_v2 \
  --run ../data/lidar_assessment/20260929/l1b4_autonav_repeat_v1 \
  --repeat ../data/lidar_assessment/20260929/l1b4_synthetic_repeat_v2

# 실제 bag 반복 hash는 위 --run 검증 후 별도로 비교
python3 -B tools/lidar/tunnel_geometry/verify.py --data-root ../data \
  --run ../data/lidar_assessment/20260929/l1b4_autonav_v2 \
  --repeat ../data/lidar_assessment/20260929/l1b4_autonav_repeat_v1

# geometry를 변경하지 않는 OBB 후처리, output은 신규 경로
python3 -B tools/lidar/analyze_l1b4_fragments.py \
  --run ../data/lidar_assessment/20260929/l1b4_autonav_v2 \
  --output /tmp/<new-l1b4-fragment-report>
```

중간 `l1b4_synthetic_v1`은 ROS setup을 source하지 않아 shared library 로딩이 실패했다.
`v2`는 필터·지표 계산 뒤 `st_size`를 함수처럼 호출하는 보고서 생성 오류로 manifest를 완성하지
못했다. `synthetic_v3/repeat_v1/autonav_v1`은 geometry 계산 뒤 컨테이너 Git의
ownership 검사 때문에 manifest 생성이 실패했다. Git 2.25는 이 환경에서 command-scope
safe.directory를 무시하므로 임시 XDG 설정에 이 repo 한 곳만 허용하는 읽기 전용 조회로
정정했다. 사용자 global Git 설정을 바꾸거나 모든 repo를 신뢰하지 않았다.
이 디렉터리는 디버깅 이력으로 보존하며 최종 회귀·성능 근거에서 제외한다.
원래 알고리즘이나 고정 가설값을 조정해 통과시킨 오류 정정은 아니다.

## 다음 검토와 rollback

이번 출력은 판제에 연결하지 않는다. 다음 단위는 **L1-B4-B: Autonav 실제 벽 근접 장애물 배치와
연속 주행 검증**이다. 먼저 정차 상태에서 양쪽 벽·거리·gap별 car/person/낮은 일반 장애물을
배치하여 source-time/좌표 근거가 확인된 객체 geometry와 비교한다. 실제 카메라 결과와 격리
empty/missing 보호 상태를 같은 scan에 적용해 비교한다. 그 뒤 ego 이동·벽 단차·가림으로 확대한다.

raw에 낮은 장애물 점이 없는 경우에는 filter를 바꿔 복구할 수 없다. 관측 부족, upstream ground,
wall 삭제, cluster 병합/최소점, OBB 거부를 원인별로 나누어 다음 수정 하나를 선택한다.
YOLO 모델 비교·ground/clustering/tracker 교체·GPS 전환 운영 연결은 이 단위와 별개다.
사용자 검토 후 같은 입력으로 비교하며 손재호 최신 후보와의 비교도 별도 revision으로 남긴다.
운영 적용 전까지 rollback은 실험 브랜치/도구만 제외하면 되고 현재 planner 동작은 변하지 않는다.
