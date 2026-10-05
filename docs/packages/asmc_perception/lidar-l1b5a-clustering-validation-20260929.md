# L1-B5-A — TRAVEL AOS 격리 비교 결과 (2026-09-29)

## 결론과 운영 경계

**현재 고정 설정의 AOS는 미채택이다.** Capture는 투영 전제 미충족으로 AOS 평가가
차단됐고, 합성 비교에서도 기존 성공의 추가 실패가 발생했다. 원본 점 Euclidean만으로는
Capture의 차량 실패 22관측을 개선하지 못했다. 성공 기준을 낮추거나 다른 단계까지 함께
수정해 통과시키지 않았다.

`feat/lidar-l1b4-geometry`에서 격리 평가기·회귀 입력·보고서만 추가했다. `src/`, 센서 JSON,
ROS 토픽/message MD5, HD맵 geometry, 판제 연결, 손재호 checkout, external checkout은
변경하지 않았다. 푸시·PR·운영 입력 교체는 하지 않는다.

[재실행 도구](../../../tools/lidar/clustering_compare/README.md) ·
[동결 설정](../../../config/baselines/lidar_l1b5a_clustering_20260929.yaml) ·
[결과 manifest](../../../config/baselines/lidar_l1b5a_result_20260929.yaml) ·
[소비 계약](../../contracts/perception/lidar-planning-consumer-v1.md).

## 1. 고정한 비교 조건

입력은 [L1-B4-B](./lidar-l1b4b-capture-validation-20260929.md)의 105장면,
[L1-B4-A](./lidar-l1b4-validation-20260929.md)의 기존 580호출 합성 입력,
기존 정차 bag의 2,562개 snapshot이다. Capture는 camera missing/empty 각각 독립 reset,
같은 장면 5회 정차 warmup 중 마지막 1회만 측정한다. 210조건을 210개의 독립 상황으로
세지 않으며 tracking 수락도 아니다.

| 비교군 | 점 표현과 군집 | 고정한 후단 |
|---|---|---|
| baseline | 기존 voxel, 3D Euclidean 1.0m, 10~30,000점 | 기존 L-shape·객체 제거 |
| raw Euclidean | 같은 voxel 제거 마스크를 원본 점에 lift, 3D Euclidean 1.0m | 동일 코드·규칙 |
| TRAVEL AOS | raw Euclidean과 같은 원본 입력, 센서 원점 range-image | 동일 코드·규칙 |

raw 두 군집군 모두 cluster의 **서로 다른 기존 voxel**을 10~30,000개 요구한다.
ROI·voxel mask·수평화·지면·천장·벽·카메라 보호 설정은 고정했고 미채택 기하 보호 및
25ms 대기는 연결하지 않았다. 1↔2는 점 표현 영향, 2↔3은 투영 손실을 포함한 AOS 경로
영향이다. 더 조밀한 raw 입력의 효과를 AOS 개선으로 보고하지 않는다.

손재호 기준 `6435af10671235e735e4a9ed1bd747bd8d8389ac`, TRAVEL checkout
`95dc2fbd66a343efd9060c45a5711b6307a950a4`를 유지한다. 이번 구현 전 팀 repo 기준은
`eedc78e`, 설정 동결은 `f39b23e`, 초기 구현은 `c6d9bb7`이다. 실행 시 revision과 파일 SHA가
각 manifest에 있다. 일부 실행 도구가 로컬 수정 상태였으므로 **revision만으로 재현됐다고
하지 않고 도구/바이너리/generated-source SHA를 함께 확인한다.**

## 2. 투영 전제: Capture AOS 전량 차단

점 수가 19,200이라는 이유로 `16×1,200` 배열 순서를 확정하지 않았다. 실제 유효 점의
수직각, interleaved channel 대응, 연속 column의 방위각 간격을 고정 gate로 검사했다.
수직각 잔차 p99 ≤0.8°, 채널 대응 차이 ≤0.5%, 방위각 step 잔차 p95 ≤0.01°는
**이 실험의 사전 동결된 근사 투영 승인 기준**이며 제조사 정확도 규격이 아니다.

- 105장면 중 97개 통과. sequence **20, 21, 22, 40, 65, 66, 67, 68**에서 기준 초과.
- 전체 2,016,000 슬롯 중 유효 return 1,676,611개; channel 대응 차이 1,799개.
- 근사 투영 셀 중복 제외 예상 770점. 이 수치는 sensor/GT 오류율이 아니다.
- 전체 투영 승인은 `false`. Capture에서 AOS 호출 **0건**이다. 통과 97개만 선택해 비교하지 않았다.

각도 차이의 원인이 데이터 축/원점, 실제 beam 모델, 장착·자세, 기타 요인 중 무엇인지는
미확인이다. GT나 최근접 IMU로 nominal ray에 맞춰 데이터를 임의 이동/회전시키지 않았다.
측정 ring과 점별 광선 시각도 만들어내지 않았다. **Capture 자체가 잘못됐다고 판정한
것이 아니라, 이 AOS 투영 계약을 전체 Capture에 대해 입증하지 못한 것이다.**

원본 감사는 `$ASMC_DATA/lidar_assessment/20260929/l1b5a_projection_v1/`에 있다.
`projection.py`는 보고서를 보존한 뒤 exit 1로 차단하며, runner는 감사 manifest와 설정·입력
SHA를 확인하고 전량 차단을 강제한다. blocked를 fresh empty나 객체 0개로 채점하지 않는다.

## 3. 실제 Capture: 점 복원만으로 해결되지 않음

아래 값은 camera empty와 missing **각각에서 같은 결과**다. 두 조건을 합쳐 관측 수를
두 배로 부풀리지 않는다.

| 지표 | voxel baseline | raw Euclidean | AOS |
|---|---:|---:|---|
| 충분한 점 차량 | 91 | 91 | 평가 차단 |
| 완전성 ≥80%·순도 ≥90% 차량 | 69 | 69 | 평가 차단 |
| 충분한 점인데 실패한 차량 | 22 | 22 | 평가 차단 |
| 충분한 점 장애물 / 성공 | 41 / 41 | 41 / 41 | 평가 차단 |
| 기존 성공 110관측의 추가 실패 | — | 0 | 미검증 |
| 기존 차량 실패의 회복 | — | 0 | 미검증 |

empty 최종 객체는 양쪽 312개(Instance 지원 139, 미지원 173), missing은 양쪽 441개
(지원 139, 미지원 302)다. Instance 지원 없는 출력을 즉시 FP로 세지 않는다.
추가 instance 합침 관측도 없지만 같은 정차 입력의 점 품질 결과일 뿐 전 미션 검출 정확도가 아니다.

실패 22개 중 19개는 분할/부분 군집, 3개는 충분한 전체 점이 있지만 허용 군집이 없는 사례다.
ROI 밖·희소 관측은 이 22개와 분리한다. sequence 28의 VehicleID 2처럼 두 표면 간
3D 최단거리가 약 1.062m인 기존 사례는 raw에서도 남는다. 1m 군집 연결 기준은 유지했다.
단순 voxel 해제나 벽 보호 강화로 해결됐다고 주장할 근거가 없다.

거리·관측량을 분리하면 다음과 같다. 충분한 관측량의 대상만 분모로 사용했으며,
아래 거리 값은 기존 GT metadata 기준이다. 원본/voxel 두 군집군의 결과가 같다.
같은 장면의 반복·class별 분포 차이가 있으므로 인과관계나 일반화 성능으로 단정하지 않는다.

| Capture 거리 | 차량 성공 / 관측 | 장애물 성공 / 관측 |
|---|---:|---:|
| <10m | 24 / 24 | 11 / 11 |
| 10~20m | 25 / 28 | 17 / 17 |
| 20~40m | 20 / 39 | 13 / 13 |

| wall-stage 대상 voxel 수 | 차량 성공 / 관측 | 장애물 성공 / 관측 |
|---|---:|---:|
| 10~29 | 16 / 33 | 15 / 15 |
| 30~99 | 25 / 30 | 21 / 21 |
| ≥100 | 28 / 28 | 5 / 5 |

차량 실패의 19/22가 20~40m, 17/22가 10~29 voxel에 있다. 범위/빔 간격과 고정 3D
연결의 상호작용을 다음 후보에서 검토할 근거이지, 단순 tolerance 확대의 채택 근거는 아니다.
40m 이상 충분한 대상은 이 입력에 없다. 부분 가림은 합성 `occluded` 조건과 점 지원량으로
분리하며 실제 Capture의 가림 수준은 정량 GT가 없어 임의 등급을 부여하지 않았다.
합성 객체의 거리 metadata가 없으면 unknown(null)으로 남기고 0m로 채우지 않는다.

## 4. 합성: 개선과 회귀를 함께 기록

기존 580호출의 116 측정 중 nominal sensor ray를 확인할 수 있는 60조건에서만 AOS를
평가했다. 임의 dense 점군 54조건과 원래 ray에 추가 회전한 tilted 2조건은 승인하지 않았다.
ray spacing 0.25°인 기존 분석용 합성과 AOS 고정 1,200 column(0.3°)의 차이도 보존했다.
이는 VLP16 실입력을 재현한 것이 아니며 이 설정의 셀 충돌 결과를 모든 AOS 구현의
본질적 결함으로 일반화하지 않는다.

| 같은 AOS 승인 입력의 대상 관측 | baseline | raw Euclidean | AOS |
|---|---:|---:|---:|
| 기존 합성 38 대상 관측 성공 | 4 | 4 | 0 |
| 추가 합성 18 대상 관측 성공 | 9 | 9 | 10 |

기존 합성에서는 좌우 벽 근처 차량의 empty/unsupported 각 2관측, 총 **4개 추가 실패**다.
각 대상 109점 중 9점은 기존 벽 단계에서 제거됐다. 남은 100점에서 셀 중복 제외 17점,
작은 component의 기존 voxel gate 탈락 6/8점으로 가장 큰 최종 객체 완전성이 약
70.6%/68.8%였다. 순도는 100%여도 완전성 80%를 못 넘는다. `labels=kept`와 실제
projection/cluster 포함은 다르므로 단계별 사유를 같이 저장했다.

추가 8종 기하×camera empty/missing =16조건은 인접 차량, 양벽/차량, column 빈 구간,
가림, 원거리 희소점, ±180° 방위각 seam, 벽만 있는 장면을 포함한다.
공통 16방향×1,200 column 광선의 첫 교차면만 사용한 합성이며 실제 센서 모델이 아니다.

- separated 인접 차량의 missing 조건에서 baseline/raw는 벽과 합쳐 실패했고 AOS는
  차량 두 관측을 분리했다. GT 기반 연결이나 카메라 양성 box는 입력하지 않았다.
- azimuth seam의 empty 조건에서 기존 성공 **1개가 새로 실패**했다. 140점 중 최종 객체
  68점(48.6%), 다른 component 72점은 기존 unique-voxel gate에서 탈락했다.
  missing에서는 벽 잔여를 포함한 연결 구조가 달라 이 대상이 유지됐으므로 camera 조건을
  섞어 평균으로 회귀를 가리지 않는다.
- close 인접 차량 합침, wall-car missing 합침, far sparse의 관측량 부족은 해결되지 않았다.
- 후단 L-shape/OBB 필터를 바꾸거나 작은 군집을 무조건 살려 회귀를 통과시키지 않았다.

현재 설정은 **추가 성공 소실 0건** 기준을 위반한다. 실제 Capture 차량 22실패 중 11개
회복 기준은 투영 전제 미충족으로 **평가 불가**다. 둘을 알고리즘 성공 또는 배포 수락으로
표현하지 않는다. 다음 독립 후보 전까지 AOS 추가 튜닝/복합 알고리즘은 진행하지 않는다.

## 5. 시간·반복·기존 회귀

Intel i5-14600K, 기존 Noetic 컨테이너의 Release C++17/PCL/Eigen에서 측정했다.
여러 격리 평가가 병행된 CPU 계측이다. 아래는 GPU/YOLO/ROS 송수신을 포함한 latency가
아니며, 실제 runtime 예산 수락도 아니다. setup/직렬화/점 계보 및 파일 복사는 별도다.

| 동일 추가 합성 16조건: 단계 [ms] | p50 | p95 | p99 |
|---|---:|---:|---:|
| AOS 투영(진단·setup·core 투영 포함) | 0.363 | 0.487 | 0.580 |
| AOS 연결·core label 변환 | 0.067 | 0.110 | 0.111 |
| AOS 기존 voxel 관측량 gate | 0.079 | 0.163 | 0.175 |
| AOS pinned OBB | 0.269 | 0.531 | 0.539 |
| AOS 위 단계+OBB filter | 0.777 | 1.264 | 1.344 |
| raw Euclidean 군집+gate+OBB/filter | 4.046 | 8.077 | 8.558 |

빠르더라도 점 손실/분할 회귀를 허용하지 않는다. 실제 Capture raw Euclidean의
군집+OBB/filter는 p50 1.800, p95 10.672, p99 17.321ms였고 성공 수는 늘지 않았다.
평가 전용 lineage 비용과 전체 운영 latency를 합성하거나 타 논문 FPS로 환산하지 않는다.

- Capture 1,050호출×2, 기존 합성 580×2, 추가 합성 80×2에서 시간 계측을 제외한
  **군집·OBB·점 계보 digest가 동일**하다. Capture의 과거 L1-B4-B baseline과도 동일하다.
- 정차 bag 2,562 snapshot을 재처리: 유효 1,011(예열 제외 측정 924), inactive/상태 부적합
  1,551은 skipped로 유지했다. AOS는 센서 ray 계약 미확인으로 차단했다.
  과거 L1-B3-G baseline의 동일 관측 geometry 비교는 별도 parity 보고서에 기록한다.
- 원본 관측 stamp, point count, intensity, raw→voxel 계보, 후보 OFF parity, 결정 이후 도착한
  pose/camera 사용 모두 오류 0건이다. 과거 live 실행의 정확한 내부 상태 복원이라고 하지 않는다.
- 호스트 Python 회귀 63개, AOS C++ indexed-clustering 시험, 기존 B4 C++ 시험,
  L1-A revision/MD5·B1·B2·B4-B의 고정 artifact 검사가 통과했다.

## 6. 산출물과 초기 실행 오류 보존

산출물은 `$ASMC_DATA/lidar_assessment/20260929/` 상대 경로로 연결한다. 원본 Capture,
bag·external 레포를 복사해 팀 repo에 넣지 않는다.

| 정본 산출물 | 상대 경로 |
|---|---|
| Capture run/repeat, 후처리 점 점수 | `l1b5a_capture_final_v1/{run,repeat,run_scores,repeat_scores}` |
| 기존 합성 run, 점 점수 | `l1b5a_synthetic_final_v1/{run,run_scores_v4}` |
| 기존 합성 repeat, 점 점수 | `l1b5a_synthetic_repeat_final_v1/{run,run_scores_v4}` |
| 추가 합성 input/run/점 점수 | `l1b5a_extra_fixtures_v1`, `l1b5a_extra_final_v1/{run,run_scores_v4}` |
| 추가 합성 repeat/점 점수 | `l1b5a_extra_repeat_final_v1/{run,run_scores_v4}` |
| 기존 bag snapshot 재처리·parity | `l1b5a_stationary_final_v1/{run,parity.json}` |
| BEV/측면 비교 | `l1b5a_figures_v1/*.png`와 대응 SHA sidecar |

대표 그림은 `capture_sequence28_missing.png`, `adjacent_separated_missing.png`,
`azimuth_seam_empty.png`, `sparse_car_empty_projection_losses.png`다. 색은 cluster이고 GT가
아니며, 주황 OBB는 pinned 후단이 살아남긴 결과다. 일부 단일 면의 OBB 두께가 0이어도
point-score 성공만으로 전체 차량 box가 정확하다고 수락하지 않는다.

초기 `l1b5a_synthetic_v1`은 진단 셀 반올림과 코어의 double/float 연산 차이를 잡아
중단했다. 이후 정확한 core getRowIdx/getColIdx로 진단을 계산했다. 일부 초기 실행은
Ubuntu Git 2.25의 command-scope safe.directory 미지원으로 metadata 마감에서 실패했고,
기존 private temporary XDG helper로 정정했다. synthetic score의 NumPy ID 직렬화 결함은
int 변환과 end-to-end 시험으로 수정했다. 이 초기 산출물은 삭제/덮어쓰기하지 않았으며
수정된 후처리는 새 v3 폴더로 저장했고, 합성의 거리 metadata 보존/unknown 정정은
v4로 별도 저장했다. 군집 성능 summary는 v3/v4가 같다. 이 정정은 mask·군집 threshold 튜닝이 아니다.

## 7. 다음 사용자 검토 지점

이번 단위에서 멈춘다. 새 bag 수집, YOLO·ground·tracker 교체, 운영 연결은 하지 않았다.
우선 **원본 Capture의 축·센서 원점과 실제 beam layout 근거를 확인**하는 작은 검증 단위를
권장한다. nominal 각도에 맞추기 위해 임의 extrinsic/가짜 ring을 생성하지 않는다.
이 전제가 해결되면 `depth_clustering`을 별도 군집 비교로 진행할 수 있다. 전제를 확인하지
못하면 투영을 요구하지 않는 거리 적응형 3D 대조군을 별도 승인 후 검토한다.
일괄 threshold 확대는 인접 차량/벽 합침 회귀 때문에 자동 채택하지 않는다.

관측·추적 안정화, 구조물 지지점/주행 관련성, linefit→Patchwork++, 차로 귀속은 이후
독립 단위다. 카메라 미검출을 LiDAR 객체 삭제 근거로 사용하지 않고, 기존 판제 보완을
제거할 근거도 아직 없다.

[TRAVEL 원 논문](https://arxiv.org/abs/2206.03190)은 구형 투영의 수평·수직 그래프를 설명한다.
현재 [공식 구현](https://github.com/url-kaist/TRAVEL)의 `TravelGroundSeg`와 `ObjectCluster`는
분리돼 있어 **AOS만 비교할 수 있다**. 본 결과는 pinned 구현·이번 입력·고정 설정의 결과이며,
지면분할까지 반드시 함께 바꿔야 한다거나 타 환경 FPS로 후보를 배제한다는 근거가 아니다.
