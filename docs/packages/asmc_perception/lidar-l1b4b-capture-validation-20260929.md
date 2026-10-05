# L1-B4-B — 실제 Capture GT 비교와 다음 군집화 단위

상태: **105개 실제 Capture의 격리 비교·반복 검증 완료, 기하 보호 후보 운영 미채택 유지**.
이번 단계는 데이터 수집 확인만 한 것이 아니다. 새 importer·raw 점 계보 채점기·회귀시험을
구현하고, pinned C++ 필터와 기존 기하 후보를 컨테이너에서 실제 실행했다.
운영 LiDAR·bridge·판제·ROS topic/MD5·sensor profile·HD Map은 변경하지 않았다.

[이전 후보와 실패](./lidar-l1b4-validation-20260929.md) ·
[재실행 도구](../../../tools/lidar/capture_validation/README.md) ·
[기계 판독 기준](../../../config/baselines/lidar_l1b4b_capture_20260929.yaml) ·
[소비 계약](../../contracts/perception/lidar-planning-consumer-v1.md)

## 1. 동결한 자료와 사용 범위

`data/capture_runs/kcity_capture_20260929T052220_840`의 성공 sequence 0~104만 사용한다.
원본 LiDAR는 Windows MORAI 기본 `SaveFile/SensorData/LIDAR_5`, state·camera는 별도
`SensorData`에 있다. 각 원본의 상대 경로·크기·SHA를 admission manifest에 보존했다.
원본은 이동/정규화/덮어쓰지 않았고 container 재생용 bit-identical XYZI 파생 사본만 생성했다.

| 확인 항목 | 결과 | 수락 범위 |
|---|---:|---|
| paired Intensity/Semantic/Instance | 105/105, XYZ·순서 완전 일치 | raw 점별 GT 대응 |
| float4 slots | 19,200/장면, 총 2,016,000 | padded slot 포함; runtime ROI에서 제외 |
| instance-labelled 점 | 21,346 | Vehicle 17,162 / Obstacle 4,184 |
| box 관측 | Vehicle 175 / Obstacle 83 | 서로 다른 actor 258개라는 뜻이 아님 |
| GT box/corner 최대 불일치 | 0.052 mm | 내부 형식 일치; point truth와의 일치는 별도 |
| box+3cm 밖 instance 점 | 826점, 43 box 관측 | 해당 box의 OBB 정확도 수락 제외; 점 GT는 유지 |
| native IMU nSecs 범위 오류 | 81/105 | 측정 stamp로 사용하거나 secs에 더해 복구하지 않음 |
| front BBox2D 파일 부재 | 33/105 | 정상 빈 YOLO 결과의 근거가 아님 |

선언 센서셋은 Autonav v3이며 reference의 LiDAR `(0.80,0,1.45m)`와 identity rotation을
대조했다. Capture 자체에 센서 JSON이 내장된 것은 아니므로 **사용자 선언 profile**이라는
출처를 보존한다. paired IMU quaternion으로 자세만 얻고 장착 이동을 중복 적용하지 않는다.
front-camera box와 같은 ID의 중심을 교차 투영했을 때 선언 y축의 중앙 residual은 0.712px,
y 반전은 227.18px였다. sensor-origin FLU 사용을 뒷받침하지만 clipping·box 형상 영향이
있으므로 이를 정밀 내외부 calibration 또는 물리적 동시 촬영 수락으로 확대하지 않는다.

GPS 파일의 lat/lon은 모두 nonzero지만 status·음영 mode routing은 이번 검사 대상이 아니다.
trigger Ego 위치는 sequence 100→101에서 150.54m 점프했다. Capture pause와 이 점프 때문에
인접 Capture로 ego-motion·속도·ID tracking·GPS 복구를 평가하지 않는다.
보행자와 실제 YOLO 도착 기록도 없다. raw bag과 이 Capture는 별도 실행이므로 서로 같은
시각의 RGB/GT인 것처럼 붙이지 않았다. Sync 자료는 보존하며 이번 GT 점 채점에는 사용하지 않았다.

## 2. 비교 조건과 지표

손재호 `6435af1`, 기존 L1-B4 설정·ROI/10cm voxel·지면/천장·벽 fit·L-shape OBB·제거 규칙을
고정했다. 실행 시작점은 독립 브랜치 `8d2e201`이다. 기존 후보의 parameter를 이번 결과에 맞춰
조정하지 않았다. 필터는 격리 harness에서 강제 호출하며 운영 GPS gate를 검사한 것은 아니다.

각 장면은 camera **통제 missing/통제 empty** 두 조건으로 독립 reset한다.
각 조건에서 같은 점군·자세·trigger 정수 stamp를 5회 정차 warmup하고 마지막 호출만 채점한다.
전체 호출 1,050개, 측정 결과 210개다. 실제 5-frame 주행이나 YOLO latency 재현이 아니다.
25ms 대기는 비교에 넣지 않았다. GT box를 YOLO box로 만들어 보호 마스크에 넣지 않았다.

raw→voxel→cluster mapping을 통해 ROI 제외와 upstream 지면·천장 소실을 먼저 분리한다.
wall-stage 보존율의 분모는 그 단계까지 남아 있던 해당 Instance 점이다. 최종 cluster는
그 점의 80% 이상을 포함하고 raw-point 순도 90% 이상일 때 `accepted_target`로 진단한다.
혼합 voxel도 모든 raw 멤버를 순도 분모에 포함한다. majority label로 합침을 숨기지 않는다.
이번에는 혼합 voxel 5개에 대상 점 5개가 들어 있었다.

이는 **관측점의 군집 완전성/순도**이며 full-box IoU, 전체 차량 검출률 또는 대회 성능 보증이 아니다.
StandingOBJ(109)는 일반 구조물을 포함하므로 벽 정답으로 쓰지 않는다. Instance GT가 없는
cluster 역시 정답 목록 완전성과 주행 관련성을 승인하기 전에는 오검출로 단정하지 않는다.

## 3. 실제 객체 점 보존과 최종 출력

아래 값은 105장면 한 camera 조건의 합이다. 두 조건과 세 정책 모두 GT 대상에 대해 같은
보존·군집 수락 건수를 보였다. outside-ROI·희소 관측까지 포함한 175/83을 검출률 분모로 쓰지 않는다.

| 단계/진단 | Vehicle | Obstacle |
|---|---:|---:|
| 원본 Instance 점 | 17,162 | 4,184 |
| ROI 밖 점 | 522 | 186 |
| 지면으로 제거된 점 | 192 | 326 |
| 천장으로 제거된 점 | 0 | 0 |
| wall-stage 입력 점 | 16,448 | 3,672 |
| wall-stage 보존 점, 모든 정책 | 16,448 (100%) | 3,672 (100%) |
| wall-stage 대상 voxel ≥10 관측 | 91 | 41 |
| 최종 cluster의 80%/90% 조건 만족 | 69 | 41 |

현재 실제 배치에서 기존 벽 정책은 이미 대상 점을 보존했다. 따라서 기하 보호가 객체 점을
더 살렸다는 개선 증거는 없다. 100%는 **벽 단계만의 값**이며 ROI/지면 소실까지 없다는 뜻이 아니다.
지면 label로 제거된 접지부 점 전체를 오류로 판정하지도 않는다.

충분한 voxel이 있는 Vehicle 91관측 중 19관측은 군집 완전성이 부족하고 3관측은 최종
군집이 없다. 점이 벽 단계에서 삭제된 것이 아니라 **군집 연결/최소 점 수 단계**에서 분리된다.
Obstacle의 전면 소실 2관측은 39.58/38.16m의 희소 입력(ROI 안 3점/1점)이 지면 처리에서
없어진 사례로, 충분한 점이 있는 객체의 검출 실패와 분리한다.

대표 Vehicle sequence 28, ID2는 54 raw wall-stage 점/47 voxel이 살아 있지만 두 출력에
27점/26점씩 나뉜다. 각각 순도는 100%이고 완전성은 50%/48.1%다.
두 점집합의 최소 3D 간격은 1.0618m로 기존 1m 연결 기준 밖이며 최소 XY 간격은 0.9661m다.
이는 **군집 거리 정의를 별도 비교할 근거**이지 XY만 사용하거나 radius를 일괄 늘리자는 결론은 아니다.
벽·인접 차량 합침 위험을 동시에 검증해야 한다.

## 4. 구조물 잔여와 후보 미채택 이유

105장면 합이며 actor 수가 아니라 장면별 출력 합이다.

| 정책 / camera 조건 | 최종 객체 | Instance GT 점으로 뒷받침됨 | Instance GT 점 지원 없음 | 109 잔여 proxy |
|---|---:|---:|---:|---:|
| baseline, empty | 312 | 139 | 173 | 7,262 |
| baseline, missing (= policy_only) | 441 | 139 | 302 | 124,983 |
| geometry, empty/missing 동일 | 1,549 | 139 | 1,410 | 89,720 |

기하 후보는 missing 대비 109 잔여 proxy를 약 28.2% 줄였지만, 더 짧게 분리된 구조물
cluster가 기존 긴 벽 OBB 제거 조건을 통과하여 객체 수는 늘었다. labelled target 지원 객체는
139개로 개선되지 않았다. sequence25에서는 baseline 2 / policy_only 4 / geometry 32객체다.
이는 이전 정차 bag의 구조물 조각 문제를 실제 Instance 자료에서도 다시 확인한 근거다.
1,410개를 측정된 FP로 부르거나 109 잔여 감소를 벽 제거 정확도로 부르지는 않는다.

따라서 **기존 기하 보호 후보는 운영 미채택 유지**다. 단순히 empty를 missing처럼 취급하는
대조군도 이번 대상 분리를 개선하지 못한다. 판제 fragment 억제/ID 재연결/소실 유예를 제거하지 않는다.

## 5. 처리시간, 반복과 검증

단위 ms, 210개 측정 호출의 p50/p95/p99. 고정 filter 내부의 offline 계측이며
tracker·publish·camera GPU 부하·실제 전체 지연은 포함하지 않는다.

| 측정 | p50 | p95 | p99 |
|---|---:|---:|---:|
| baseline filter | 1.835 | 4.945 | 5.605 |
| geometry 추가 정책 | 1.574 | 2.049 | 2.412 |
| geometry 재군집/OBB | 0.899 | 1.825 | 2.012 |
| geometry 공통 전처리 이후 filter 추정 합 | 3.605 | 4.827 | 5.206 |
| frozen result copy | 0.004 | 0.009 | 0.011 |
| raw→voxel GT 계보 추적 | 4.200 | 4.822 | 5.018 |

위 filter 추정 합은 ROI/voxel과 평가 계보 추적을 포함하지 않는다. p99가 낮다는 이유로
동일 입력의 객체 품질 저하를 상쇄하거나 후보가 운영에서 더 빠르다고 판정하지 않는다.

- host LiDAR 시험 51개(새 Capture 회귀 13개 포함), camera_sync 시험 18개, C++ CTest 1개 통과.
- L1-A/B1/B2/B3와 L1-B4 baseline SHA·scope 검사가 계속 통과.
- candidate_off와 baseline의 geometry 차이 0; stamp·intensity·변환 후 count·계보 오류 0.
- controlled empty/missing 후보 마스크 차이 0; 기존 성공 대상 추가 소실 0.
- 독립 반복 1,050호출의 geometry digest 동일:
  `76c73a90a2d3628658b73c4ca62f57786b325505b7ce07593cdfd31db1eef285`.
- score의 summary/objects/frames 파일 SHA도 반복 시 완전히 일치.
- `git diff c536c3c -- src scripts config/morai` 없음. 운영/판제·wire·센서·HD Map 변화 없음.

## 6. 산출물 위치와 후속 한 단위

`$ASMC_DATA/lidar_assessment/20260929/l1b4b_capture_gt/` 아래:

- `inputs_v4`: 원본별 SHA, 선언 sensor profile, attitude/native time 제한, separate truth.
- `run_v1` / `repeat_v1`: 기존 정책·대조군·기하 후보·OFF의 마스크/군집/OBB/시간 JSONL.
- `scores_v1` / `repeat_scores_v1`: 객체별 소실 단계와 점 군집 지표, 구조물 proxy.
- `figures_v1`: sequence 2/40/85 BEV·측면; `figures_v2`: sequence25의 조각 증가와 sequence28의 차량 분할.

admission v1/v2는 보조 camera box의 미대응/부재를 필수 GT 오류로 처리해 중단된 파생물,
v3는 importer 완성 후 시험·검증기 정리 전 기록이다. 원본은 모두 유지하며 **최종 동결 입력은 v4만** 쓴다.
파일 없는 camera box를 valid-empty로 승격하지 않고 camera에만 보이는 actor도 오류로 세지 않도록
정정했다. camera와 LiDAR의 시야가 다르다는 것을 결측 검출과 혼동하지 않는다.

다음 승인 단위는 **L1-B5-A: 이미 관측된 객체의 군집 분할·누락 비교**로 제안한다.

1. 이번 22개 충분-voxel Vehicle 실패, 기존 성공 110관측, 배경 조각·혼합 voxel 사례를 고정한다.
2. 기존 wall 정책과 ROI/voxel·지면·OBB를 유지한다. GT는 평가 전용으로 둔다.
3. 실패 간격의 XYZ/range/연결성 분석 후 거리 적응형 또는 높이 제약 XY 연결 중 후보 하나만 비교한다.
   무조건 radius 증가, 최소 점 수 감소, GT class 주입, wall/ground parameter 동시 조정은 하지 않는다.
4. 관측 완전성 회복과 함께 인접 차량·벽 합침/지원 없는 조각 증가·기존 성공 회귀를 검사한다.
5. 동일 입력 반복과 비용을 보고한 뒤 사용자 검토를 받는다. 운영 연결·tracker 교체는 별도 단위다.

현재 배치로 위 군집 비교를 시작할 수 있어 **새 벽 바로 근접 bag 수집은 우선 요구하지 않는다**.
주행 차로를 막지 않는 극단적 벽 접촉 배치는 후순위 민감도 시험으로 두되 후보를 최종 운영
수락할 때는 미검증으로 명시한다. 보행자/낮은 물체·진짜 연속 주행·실제 YOLO 부하·GPS 전환은
이 Capture로 대신하지 않고 필요한 후속 수락시험으로 남긴다.
