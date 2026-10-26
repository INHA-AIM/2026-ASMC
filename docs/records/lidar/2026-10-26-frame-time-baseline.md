# C0 — 좌표·시각·보정 기준 대조

2026-10-26, 기준 main `0dab4adf643470ddcff2edb26c3e8ca91928a196` (PR #112).
**후속:** 아래는 수정 전 C0 v1 결과다. 사용자 요청으로 같은 PR #115에서
[GPS 유효성·복귀 수정과 v2 재검증](../bridge/2026-10-26-gps-validity-recovery.md)을 추가했다.
기존 수치·실패·원본 SHA는 당시 근거로 보존한다.

**진단·재현 기준을 마련했다. 운영 좌표·추정기·필터·launch·공용 메시지는 변경하지 않았다.**
다음 수정 한 가지는 bridge가 GPS `(0,0)`을 유효한 위치로 받는 입력 경계다.

## 재현 자료와 실행 경계

- [입력·소스·설정 SHA manifest](../../../config/baselines/lidar_frame_time_c0_v1.json)
- [작은 결과 사본·산출물 SHA](../../../config/baselines/evidence/lidar_frame_time_c0_v1/results.json)
- [probe 빌드·검사·재실행](../../../tools/perception/frame_time/README.md)
- [설계 정본](../../architecture/perception-coordinate-refactoring.md) · [공동 작업표](../../../src/asmc_lidar/docs/migration-tasks.md)

대용량 결과 정본은 등록한 `D:\morai dataset\shahn03\artifacts\<run-id>`다.
호스트에서는 `/mnt/d/morai dataset/shahn03/artifacts/<run-id>`, 컨테이너에서는 `/artifacts/<run-id>`로 접근한다.
사용자가 C0 결과 저장에 한해 50GiB 미만 저장을 허용했다. 공유 저장 임계값을 변경하지 않았다.

| Run ID | 상태·용도 |
|---|---|
| `lidar-frame-time-c0-20261026-r1` | 실패 보존. 반복 결과의 `timings_ms`까지 동일성에 포함한 계측 오류. 객체 차이로 해석하지 않음 |
| `lidar-frame-time-c0-20261026-r2` | 객체 동일성·투영·카메라 시각·갤러리 통과. support 확장 이전 결과 |
| `lidar-frame-time-c0-20261026-r3` | 전체 support·속도 경계 결과. 지도 표시 색 오류가 있어 갤러리는 r4 사용 |
| `lidar-frame-time-c0-20261026-r4` | 최종 오프라인 결과. 소스 `3ecc167c`, dirty patch 없음. 격자 표시 수정 후 전체 재실행, r3와 수치 동일 |
| `lidar-frame-time-c0-live-20261026-r1` | 별도 실시간 메타데이터·터널 주행·종료 복원 기록. 이전 bag과 동시 수집처럼 합치지 않음 |

최종 `summary.json`에는 실행 binary SHA도 있다. `gallery.html`은 RGB의 local OBB 코너 투영,
지도 위 기존/대조 OBB, 별도 원본 시각 카메라 자료의 M0/M1 기준 격자를 표시한다.
카메라 기준 격자는 LiDAR 관측·semantic GT가 아니다. 원본 시각이 없는 bag의 RGB 겹침을 촬영 시각 정합으로 표시하지 않는다.

| 입력 | 실제 범위·제한 |
|---|---|
| `normal` (`lidar_manual_20260929_130623`) | 649 scan. B4 자료 이름을 유지했지만 터널 입구 영상도 포함한다. 일반 도로 전 구간 대표 자료가 아님 |
| `tunnel` (`20260912_003803_…sunny_11am_wheel`) | 208 scan. 두 bag 모두 receive header만 있고 SensorFrameTiming·YOLO 관측은 없음 |
| `camera-imu-motion-20261018-r1` | 226개 3뷰 묶음, 678개 영상. 원본 clock·bridge session·IMU 기록. 위 bag과 별도 수집 |
| C0 live | 약48초 메타데이터, 565 scan 상태. raw cloud/RGB 미저장. 가속·제동·작은 조향 25초 |

## 좌표와 시각 계약

1. 센서 → 실제 사용한 수평화 → 역수평화·장착 변환 → 후륜축 `base_link` → 유효 pose의 `map` 순서다.
   중심·코너·방향에 같은 변환을 적용한다. nominal 지면 거리 **0.36m**와 GPS 전방 장착 **0.35m**는 다른 값이다.
2. 카메라 투영 보정은 전방 **1280×720**, 측방 **640×480** 원본 크기다. TensorRT 입력 크기는 보정 크기가 아니다.
3. LiDAR는 scan header 이전 IMU/CAN을 선택하고 bounded GPS pose를 사용한다. 카메라는 같은 source clock·session 안에서 SLERP한다.
4. bridge GPS 안테나 위치·ENU yaw와 판제 후륜축 위치·UTM grid yaw는 같은 `map` 이름만으로 호환되지 않는다.
5. 판제 기본 `competition` 상태 경로는 GPS/IMU pose와 별도 종방향 속도 입력을 결합한다.
   이번 native probe는 GPS/IMU **pose 코어**를 직접 호출하며 그 속도 결합까지 복제하지 않는다.
6. 기록 도착 순서를 보존하고 미래 표본을 소급 사용하지 않았다. 오프라인 replay는 실시간 worker의 최대50ms 대기·최신 scan 대체까지 재현하지 않는다.

저장소 센서셋 v3와 현재 카메라/LiDAR 설정을 대조한 장착 기준은 다음과 같다. 위치는 후륜축 base 기준 m,
각도는 설정 파일의 roll/pitch/yaw 도 단위다. 이는 파일 간 일치 확인이며 실시간 시뮬 설정을 독립 실측한 값은 아니다.

| 센서 | 위치 x/y/z | 설정 회전 | 확인 |
|---|---|---|---|
| 전방 카메라 | 1.90 / 0 / 1.20 | 0 / 2 / 0 | 원본1280×720·FOV90° 일치 |
| 좌 카메라 | 1.15 / 0.65 / 1.20 | 0 / 10 / 70 | 원본640×480·FOV130° 일치 |
| 우 카메라 | 1.15 / −0.65 / 1.20 | 0 / 10 / −70 | 센서셋290°와 동치 |
| LiDAR | 0.80 / 0 / 1.45 | 0 / 0 / 0 | `autonav_v3`와 pipeline mount 일치 |
| GPS | 0.35 / 0 / 1.25 | 0 / 0 / 0 | LiDAR/판제는 x lever arm 제거, bridge pose는 안테나 기준 |
| IMU | 0 / 0 / 0 | 0 / 0 / 0 | 비영 mount를 적용한 것으로 가정하지 않음 |

지도 원점은 UTM52N `[302595, 4124145, 0]`다. YOLO 최종 픽셀 투영에 이 map 원점은 필요하지 않으며,
원본 축→optical 축 변환은 현재 보정 클래스가 담당한다. mount와 optical 변환을 각각 다시 적용하지 않는다.

## 수치·객체 회귀

| 검사 | 결과 | 해석 |
|---|---:|---|
| 3뷰·거리·mount/roll/pitch 투영 | 54 사례, 최대차이 `2.54e-13px` | 순수 기하 `1e-8` 기준 통과. 센서 정확도 아님 |
| 잘못된 역수평화/자세 삽입 | 비영 자세에서 누락 ≥11.91px, 중복 ≥11.87px, 다른 자세 ≥8.03px | 오류 검출 대조군 통과. 0 roll/pitch에서 역변환 누락은 관측 불가능 |
| 동일 replay 두 번 | 649+208 = 857 scan 일치 | 객체 ID·순서·OBB·분류·속도·필터·상태 보존. 세션 ID와 실행 시간 지표만 제외 |
| support probe 실행 | 857 scan의 local/map 전달 배열 일치 | 원본 객체·tracker·필터 출력 변경 없음 |
| map 왕복 | 최대 `1.79e-13m` | 현재 변환의 수치 일관성 |
| qualified 속도 경계 | 객체/운동 상태 대응·무효→UNKNOWN 통과 | 독립 속도 GT나 정확도 수락 아님 |

기존 scan float 변환 기준 `1e-5m/rad`는 그대로다. 별도로 frame_time_core 35,
scan_frame 11, gps_ingest 15, pose_history 32 및 geometry boundary·camera buffer 검사를 통과했다.
카메라 sensor timing 10·ground projection 4 검사도 통과했다.
추가로 IMU 선택, source/epoch 3·전방 fusion 27·typed/planning output 15 검사에서
중단·정상 빈 결과·session/generation·속도 무효·중복/역행 경계를 확인했다.
C0 새 합성 회귀 7개와 각 추정기75 scan의 GPS 정상→0→중단→복귀 fixture를 CI에 추가했다.
대용량 bag·GPU·MORAI·외부 연구 저장소는 hosted CI의 필수 입력이 아니다.

## pose 차이와 HD맵 민감도

| 항목 | normal | tunnel |
|---|---:|---:|
| LiDAR–판제 유효 비교 쌍 | 643 | 205 |
| 위치 차이 p50 / p95 / p99 (m) | 0.0516 / **0.1866** / 0.2671 | 0.0591 / **0.1217** / 0.1889 |
| 최대 위치 차이 (m) | 0.3766 | 0.3073 |
| 판제 source stamp − scan 중앙값 | −0.830ms | −1.702ms |
| 정확히 같은 source 시각의 쌍 | **0** | **0** |
| LiDAR GPS age p95 | 191.67ms | 196.70ms |
| LiDAR 선택 IMU age p95 | 24.65ms | 6.98ms |

이는 **비동기 추정 정책 차이**다. 판제 pose를 정확한 scan 시각의 GT로 취급하지 않는다.
bridge와 LiDAR의 위치 차이 p95는 0.921/1.261m이며, 중앙값 약0.35m와 yaw 약1.35°에는
장착 기준점·grid convergence·오래된 GPS source 시각의 차이가 섞인다. 전부 좌표 변환 버그로 분류하지 않는다.

같은 측정 OBB와 전체 voxel 군집 지지점을 고정하고 대조 pose의 xy/yaw만 바꿨다.
roll/pitch/z는 원래 LiDAR context를 유지했고, 새로운 pose 예측·카메라 라벨을 만들지 않았다.

| 범위 | normal | tunnel |
|---|---:|---:|
| 전달 전 observed 객체-프레임 | 8,440 | 2,490 |
| 실제 조회한 군집 지지점 | 1,446,370 | 466,825 |
| 판정 변경 | 3 | 2 |
| 도로 내부 점 비율 차이 p95 / 최대 | 0.00625 / 0.25 | 0 / 0.13333 |

변경 사례는 `summary.json → support.bags.*.decision_changes`와
`support/*-support.jsonl`의 scan·track ID로 찾는다. **pose 변경이 지도 경계 판정에 영향을 줄 수 있음**을 보인 결과이며,
어느 pose가 맞는지, 객체를 실제 제외해야 하는지 결정한 결과는 아니다. 별도의 전달된 OBB만 보는 갤러리 집계에서는
relation 변화가0이지만, 그것만으로 전체 필터가 둔감하다고 결론내리지 않는다.

## 카메라 시간·속도 계약의 확인 범위

scan 수신 순간에 이미 도착한 RGB 중 receive header ±50ms를 만족한 수는
normal 전/좌/우 **526/523/530 (649 중)**, tunnel **159/162/157 (208 중)**이다.
YOLO 관측·정상 빈 결과·클래스가 없는 bag이므로 이는 실제 fusion 성공률이 아니다.
기존 fusion의 최대200ms 대기나 새 3뷰 matching도 이 RGB 조회에 구현하지 않았다.

원본 시각 카메라 자료를 bridge 발행 시각+0/10/100ms에 조회하면 678뷰 중
**65/349/678**개가 정확 표본 또는 SLERP bracket을 얻는다. +100ms는 오프라인 민감도 후보다.
실제 family worker 시작 시각 기록이 없으므로 운영 fallback 비율로 해석하거나 대기를100ms로 늘리지 않는다.
원본 clock·session·중복·역행·결측 판단은 현재 카메라 코어와 작은 합성 검사로 확인했다.

판제용 KF 속도와 `/tracks_map` 운동 분류 추정의 벡터 차이 p95는 6.73/2.39m/s다.
서로 다른 추정값이므로 정확도 오차라고 단정하지 않는다. 동일 객체 대응을 확인했으며 무효 qualified 속도28건은
UNKNOWN이었다. 이 수치가 큰 사례는 재호의 속도/운동 상태 검증 자료로 넘기고 Marker 문자열이나 JSON을 판제 정본으로 사용하지 않는다.

## 별도 실시간 시험과 실제 결함

사용자의 추가 주행·텔레포트 승인 후, 기존 사용자 프로세스는 건드리지 않고 독립 ROS master **11351**에서
bridge(local)·LiDAR(`/migration/lidar`, debug OFF)·판제 GPS/IMU 상태·객체 수신 어댑터를 실행했다.
일반 autonomy 제어기를 실행한 시험은 아니다. 기존 터널 P 제어 재생기의 명령/경로 계산을 사용한 한정 시험이다.

- A126에서 25초 가속·정속·제동·작은 조향: 최대 **5.53m/s**, 횡오차 최대 **0.130m**.
- 다른 actor는 그대로 유지하고, 12m 이내 접근 시 중단 조건을 추가했다. 명령 상한6.5m/s·횡오차 상한0.6m.
- 원래 위치·P 기어·제어 모드로 복구하고 이번 시험의 ROS/bridge/LiDAR 프로세스를 종료했다. 복원 기록은 live run에 보존한다.
- 초기 복원에는 yaw 1.344° 잔차가 있었다. 정차·원래 위치·제어 모드가 그대로인지 확인한 후 한 번 보정해
  잔차 −0.0176°를 기록했다. 텔레포트의 ACK만으로 정확한 좌표 복원을 선언하지 않는다.
- 기존 GPU 모델 메모리가 남은 상태에서 별도 카메라 실행이 겹쳐 큰 지연이 있었다. 시험용 카메라만 종료했다.
  이 조건은 3모델 성능 수락이나 카메라 처리율 비교에 사용하지 않는다.

**GPS `(0,0)` sentinel 처리의 계약 위반을 확인했다.**

| GPS0 구간 (전환 후100ms 제외) | 결과 |
|---|---|
| 판제 GPS/IMU `/planning/vehicle/state` | 1,561개 무효 |
| bridge `/vehicle/state` | 1,560개를 유효로 제공; 예: x=−42,025,237m, y=−4,124,145m |
| LiDAR map | GPS invalid/recovering 상태. 마지막 위치로 객체를 계속 만들지 않음 |
| LiDAR local | GPS0에서도 observed360·empty2. 별도 IMU stale14는 무효 |

bridge `ValidGps`가 `(0,0)`을 거부하지 않는 반면 판제 코어에는 sentinel 차단이 있다.
각75 scan 합성 입력에서도 bridge만9개를 유효 처리해 재현했다. 현재 canonical autonomy는 이 bridge
상태를 직접 소비하지 않으므로, 이 결과를 현재 자율주행 실패로 확대하지 않는다. **운영 수정은 하지 않았다.**

전체565 scan 중26개는 IMU stale로 무효였다. 전체 local 관측 평균11.91Hz·수신 최대공백232.65ms이나,
무효 scan이 있으므로 유효 객체 공급 수락으로 표시하지 않는다. map 무효 구간에 `/planning/objects`가 나오지 않는 것도
정상 빈 배열로 치환하지 않는 계약과 구분한다.

live timing에서 카메라/IMU 원본 clock은 `morai_sensor`, LiDAR는 `velodyne_device_hour`였다.
LiDAR scan 내부 첫–마지막 device timestamp 간격 p95는 **101.67ms**다. 이는 scan 취득 시간 범위이며
입력→발행 지연이 아니다. clock 대응 근거 없이 카메라 원본 시각과 빼거나 점별 deskew 완료로 표시하지 않는다.

## 결론과 다음 단위

| 분류 | 확인 내용 | 다음 처리 |
|---|---|---|
| 계약 위반 | bridge GPS0를 valid map으로 제공 | **다음 수정 1순위.** 기존 sentinel·freshness·중단/복귀 기준을 명확히 하고 C0 fixture 갱신. LAN2LAN/bridge 작업과 조율 |
| 의도된 정책 차이 | pose 기준점·ENU/grid yaw·sample/예측 시각, qualified/분류용 속도 | 무조건 통합하지 않음. 같은 시각 대조와 독립 근거부터 |
| 자료 부족 | LiDAR–카메라 공통 취득 clock, 실제 YOLO matching, 독립 위치/속도 GT | 다음 fusion 단계에 필요한 최소 동시 기록만 설계 |
| 미해결 | live IMU stale26, 기존 GPU 점유와 겹친 카메라 실행 지연 | 원인별 별도 조사. C0 통과로 면제하지 않음 |

최소 추가 기록은 같은 run에서 raw cloud·3뷰 RGB/관측/상태·SensorFrameTiming·IMU/GPS·현재 pose·trace를 함께 받는 것이다.
동일 clock이라는 증거와 원본/수신 시각을 구분해야 한다. 큰 새 모델 연구, ego motion, 점별 deskew,
pose 생산자 일괄 교체·공용 기하 패키지 생성·필터 임계값 변경은 이번 결과의 필수 후속으로 확대하지 않는다.
