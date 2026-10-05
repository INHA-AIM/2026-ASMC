# L1-B3-G — 터널 보호 매칭 기하 비교 및 AIM 자료 적합성

**상태: 격리 평가 완료, 운영 미채택.** LiDAR 운영 코드, YOLO 모델, ROS 토픽/MD5, planner 연결,
센서 설정은 변경하지 않았다. 이번 검증은 25ms 대기가 실제 tunnel-filter geometry에 미치는 영향과
AIM 자료의 후속 실험 사용 조건을 확인했다.

기존 [L1-B3 스케줄러 기록](./lidar-l1b3-validation-20260928.md)은 당시 v1 산출물과 해석을
보존한다. 현재 판정은 이 corrected 후속 결과다. 계약은
[LiDAR–판제 소비 계약](../../contracts/perception/lidar-planning-consumer-v1.md),
우선순위는 [LiDAR 평가·로드맵](./lidar-assessment-and-roadmap.md)을 따른다.

## 결과 요약

정차 bag의 2,562 scan에서 25ms 대기는 즉시 처리보다 304개를 더 매칭했다. 후보가 선택한 1,453개
중 box 검출은 82개, 유효 빈 검출은 1,371개, 50ms stamp 조건 초과 timeout은 1,109개다.
대기 p50/p95/p99는 12.84/25/25ms다.

pinned filter의 same-scan 재실행에서 음영 진입 58건과 중간 41건의 추가 선택은 모두 **유효 빈 검출**이었다.
이 99건은 ambiguous-wall fallback을 꺼서 벽 점 제거를 늘리고 geometry를 바꿨다.
매칭률은 개선됐지만 **25ms 후보는 운영에 적용하지 않는다.** 정상 빈 검출과 카메라 미수신·timeout을
구분해 fail-safe 정책을 먼저 정하는 것이 다음 단계다.

## 고정 입력과 산출물

원본 bag/AIM 자료는 복사·수정하지 않았다. 결과는 저장소 바깥
$ASMC_DATA/lidar_assessment/20260928/ 아래에 있으며, 실행별 manifest가 입력·도구·바이너리·
결과 SHA와 한계를 기록한다.

| 산출물 디렉터리 | 내용 |
|---|---|
| l1b3g_fifo_v2 | 2,562 scan의 즉시/25ms 인과적 FIFO 결정 |
| l1b3g_geometry_v7 | 고정 점군·pose·YOLO 선택으로 pinned filter 재실행 |
| l1b3g_synthetic_v6 | 36개 합성 조건과 manifest |
| aim_audit_v3 | AIM session 목록, 표본 payload 검사, sensor profile |

Autonav 기준은 stationary_0.bag (SHA-256 c4ce3c2afe53167c38dba3f284fe166ef5ebc6cb755581815ea1200eb6c8b076)다.
구간은 시작, 고주로 A423, 음영 A126 진입·중간, 복귀와 텔레포트 transition이다. pinned Son source
6435af10671235e735e4a9ed1bd747bd8d8389ac, 격리 evaluator SHA
e4703597fde5ce5ad87a7f5a3610322c8476a7abf708850a2119aa74d0606ea0이다. 운영 node가 아니다.

## 즉시/25ms 비교

양쪽에 기존 millisecond 절삭 stamp 의미를 동일 적용하고 원본 nanosecond stamp도 보존했다.
scan 점군·ROI/voxel·scan 도착 시점까지의 pose·수평화 입력을 고정했다. 기다리는 동안 새 pose를
과거 scan에 소급 적용하지 않았다. FIFO, 최대 queue 2, 결정 후 도착 데이터 사용 0 조건이다.

| 구간 | scan | 즉시 | 25ms | 추가 |
|---|---:|---:|---:|---:|
| 시작 | 475 | 260 | 310 | 50 |
| 고주로 A423 | 467 | 203 | 267 | 64 |
| 음영 A126 진입 | 463 | 184 | 242 | 58 |
| 음영 A126 중간 | 461 | 188 | 229 | 41 |
| 복귀 | 471 | 208 | 268 | 60 |
| transition/unassigned | 225 | 106 | 137 | 31 |
| **전체** | **2,562** | **1,149** | **1,453** | **304** |

25ms 후보 사유는 box 82, valid-empty 1,371, timeout 1,109다. 실 bag의 invalid·중복·역행 stamp와
queue overflow는 0건이다. 합성 검증은 49/50/51ms stamp, 0/10/25/26/50/130/200ms 도착, empty·
missing·invalid·역행 입력과 FIFO를 다룬다. 대기시간은 end-to-end latency가 아니다. offline CPU
filter-stage 측정에는 ROS publish·tracker·동시 camera/GPU 부하가 없다. 그러므로 25ms가 runtime
예산에 맞는다는 결론은 내리지 않는다. 인과 replay는 verify_evaluation.py --data-root ../data로
통과했다.

## 실제 filter 재실행

손재호 pinned 6435af1의 실제 보호점 계산, tunnel filter, L-shape OBB를 썼다. baseline/candidate의
초기 상태·파라미터·scan geometry는 동일하고 선택 YOLO 결과만 달라진다. 2,562 scan 중 1,011개는
음영 filter 입력으로 유효했고, 1,551개는 해당 GPS/filter 조건이 비활성이라 건너뛰었다. 미래 pose,
point-count, intensity 보존 오류는 모두 0건이다. valid geometry에서 107건의 출력 geometry가
달라졌다. 공식 음영 구간의 99건(진입 58, 중간 41)은 timeout 대신 valid-empty를 선택한 경우이고,
나머지 8건은 transition/unassigned다. 벽 정답 GT는 없다.

아래는 음영 99 scan의 누적 filter 결과다. 최종 객체 수는 파이프라인 출력이지 정답 객체 수/재현율이 아니다.

| 구간 | scan | 벽 제거 baseline→후보 | 유지점 baseline→후보 | cluster baseline→후보 | 최종 객체 출력 baseline→후보 |
|---|---:|---:|---:|---:|---:|
| A126 진입 | 58 | 300,711→383,365 (+82,654) | 83,353→699 | 183→3 | 65→3 |
| A126 중간 | 41 | 218,800→278,331 (+59,531) | 59,782→251 | 121→0 | 39→0 |

유효 빈 검출을 선택한 candidate는 protection available로 취급됐다. pinned ProcessWalls는 이 경우
ambiguous-wall fallback으로 점을 남기지 않았다. 결과는 벽점 제거 증가와 유지점·cluster·출력 감소다.
하지만 filter의 벽 분류는 벽 GT가 아니며, 객체 출력 count 감소도 미탐지 건수가 아니다. 카메라 시야와
가림 여부가 검증되지 않은 상태에서 empty 하나로 fallback을 끄는 것은 안전 정책 검토가 필요하다.

합성 36조건에서 같은 양성 box가 허용 대기 안에 도착한 geometry regression은 통과했다. valid-empty
세 조건은 fallback을 꺼 객체 점 중 벽으로 처리된 점이 car +402, person +92, low-unknown +20
늘어나는 pinned 동작을 재현했다. 이는 analytic fixture이지 simulator 정확도나 GT 점수가 아니다.

**당시 판정(2026-09-28):** 다음 정책 후보는 YOLO 결과를 valid-boxes, valid-empty, unavailable/stale, invalid로 분리하고
empty가 보호 fallback을 끄는 게 맞는지 담당자와 결정하는 것이다. positive box, empty, timeout을
동일 입력으로 재생하여 벽 근접 객체와 wall residual을 회귀한다. Son 운영 코드와 판제 보완 로직은
이번에 수정하지 않았다.

**후속(2026-09-29):** 개발 일정이 다른 손재호와 각자 독립 브랜치에서 비교하기로 했다.
[L1-B4-A](./lidar-l1b4-validation-20260929.md)는 이 정책과 벽 기하 보호를 격리 구현했다.
합성 개선과 실제 구조물 조각의 다수 객체 출력을 함께 기록하며 운영에는 채택하지 않았다.
위 당시 논의와 L1-B3-G 원본 산출물은 보존한다.

## AIM 자료 적합성

ASMC_AIM_DATA를 읽기 전용으로 감사했다. sensor contract가 있는 174 session과 producer 보고
79,304 bundle을 조사했다. capture cadence 선언은 10Hz, 보고서상 motion-consistent는 160 session,
physical simultaneity 검증 0, accuracy/training label 승인 0이다. 이는 사용 금지가 아니라 현재
증거로 정량 GT 승인을 할 수 없다는 의미다.

7 campaign group에서 LiDAR native binary 5개씩, camera1~4 image 표본을 검사했다. LiDAR XYZI
float4 35표본은 ROS1 PointCloud2와 bit-exact이고, image 28표본은 catalog SHA·해상도와 일치했다.
선택 파일의 형식 확인이며 전체 session 인증이나 실제 촬영 동기 증명은 아니다.

| 속성 | Autonav v3 | AIM contract/sample | 주의 |
|---|---|---|---|
| LiDAR mount | rear-axle 기준 (0.80, 0, 1.45m) | base_link→Lidar3D-5 (1.00, 0, 1.48m) | 같은 좌표로 옮겨도 가림·점밀도 차이 |
| 축 | Autonav 설정 | raw FLU, encoder FRU 선언 | 확인된 raw/ROS에 FRU를 다시 적용하지 않음 |
| camera4 | 전방 경사, pitch 328° | 후방, yaw 약 180° | 이번 전방 YOLO 보호 비교 제외 |
| GPS | (0.35, 0, 1.25m) | (0, 0, 0) 설정 | 원점/lever arm 미확인 |
| cadence | Autonav 설정 | Sensor Record 10Hz 선언 | 15Hz 동등 자료 아님 |

AIM contract의 base_link가 rear-axle datum과 같은지는 독립 확인되지 않았다. 동일 map_id라도
link/node/route-definition SHA 조합은 3종이다. HD Map station은 session/frame별 route 대응 전
직접 적용하지 않는다. full loop_235314에는 sensor contract도 없다.

campaign 이름상 general/full-loop, dynamic_obstacle 100 session, static_010101 50,
static_obstacle, roundabout/highway/tollgate 및 shadow 후보가 있다. frame별 링크/미션 대응은 미검증.
기하 비교는 profile을 고정한 단일 점군부터, tracking은 time/pose/ID를 확인한 연속 세션만,
camera fusion은 exposure와 LiDAR source-time 관계가 입증된 구간만 후보로 한다. 불완전 object
geometry는 정확도/학습 GT로 사용하지 않고 누락 label을 false positive로 세지 않는다.
camera4는 제외하며 camera1~3도 intrinsics/exposure/time 확인 전에는 fusion 정답으로 쓰지 않는다.

## 검증과 재실행

camera_sync 회귀 18개 통과, isolated CMake evaluator 빌드 성공, 저장된 causal 결정 검증 통과.
운영/planner/sensor diff는 없다. filter 및 YOLO 교체, AIM 학습은 수행하지 않았다.

재실행은 [camera_sync README](../../../tools/lidar/camera_sync/README.md)를 참조한다.
대표 그림과 실행 manifest는 $ASMC_DATA/lidar_assessment/20260928/l1b3g_geometry_v7/에 있다.
