# L1-B1 좌표·시간 코어 — 격리 검증 결과

**상태: 격리 시험 통과, 사용자 검토 대기. 운영 출력 교체 아님.**

[코어·재실행 명령](../../../tools/lidar/frame_time/README.md) ·
[고정 manifest](../../../config/baselines/lidar_l1b1_20260927.yaml) ·
[공통 소비 계약](../../contracts/perception/lidar-planning-consumer-v1.md) ·
[L1-A 원본 실험](./lidar-l1a-validation-20260926.md) ·
[상세 분석·정정 이력](./lidar-assessment-and-roadmap.md#131-l1-b1-재검수-기록--원-주장과-정정-이유-보존).

## 변경 범위

`tools/lidar/frame_time/`에 C++17/Eigen 코어, JSONL CLI, pinned 원본 adapter probe,
독립 CMake/test, 평가/해시 검증 도구를 추가했다. `src/`·bridge·카메라·판제·ROS 메시지·
HD맵·MORAI 센서 JSON에는 변경이 없다. simulator를 움직이거나 운영 노드를 재시작하지 않았다.
기존 L1-A 미커밋 자료도 유지했다.

| 기능 | 구현·시험 | 아직 하지 않은 것 |
|---|---|---|
| 좌표 | 실제 적용 leveling 역변환 → mount → optional 관측 pose. local은 rear-axle body `base_link` | raw 입력 자체의 수평화/지면 처리 변경 |
| geometry | 중심·8개 코너·long axis 동일 rigid transform, 코너 거리/순서/ID/class 보존 | yaw-only OBB 재생성, 객체 검출·tracking 평가 |
| 속도 | absolute vector 축 회전 / base 위치 상대 미분 + 병진·회전 항 / unknown 구분 | legacy 속도 의미 임의 확정, CAN 중복 가산, unknown=0 |
| 시각 | 정수 ns, stamp 출처, receipt/processing cutoff, 별도 steady_clock duration | 수신 시각을 ray 시각으로 승격, 다른 clock offset 자동 추정 |
| 상태 선택 | 도착 cutoff 적용 후 동일 clock bracket, slerp/position 보간, gap≤100ms, 외삽 금지 | GPS 복구 점프 gate/상태기계, tracker 연속성 |
| 실패 | local/map/velocity 가용성 분리, bad pose timing/NaN은 map 불가 사유로 보존 | 운영 메시지에서 invalid/empty/inactive 새 필드 정의 |

## 고정 소스·입력

2026-09-27 `git ls-remote`로 main/재호/서준 원격이 아래와 동일함을 재확인했다.
새 원격 commit을 적용하거나 기존 기준선을 갱신하지 않았다.

| 대상 | revision |
|---|---|
| main | `1ce434d0082e3aec6cf79145c05484e1f66fac1c` |
| 손재호 | `6435af10671235e735e4a9ed1bd747bd8d8389ac` |
| 양서준 | `8b3f0b1fab5f8e8f10531b42fb63b2953c8df822` |

원본 `PlannerObjectAdapter.cpp`를 직접 컴파일한 probe와 후보 코어에 **같은 합성 snapshot**을
넣었다. 실제 preprocessing matrix·장착 회전/이동·안테나 위치·관측 pose가 snapshot에 있다.
원본의 point/vector 함수만 호출하며 전체 detector/tracker/publisher를 재실행한 비교가 아니다.
강태 미공개 구현도 수락 범위 밖이다.

실험 결과는 `$ASMC_DATA/lidar_assessment/20260927/l1b1_v3/`에 있다. `report.json`이
소스/바이너리/compiler/입력/산출물 SHA와 명령을 연결하고 manifest가 report SHA를 고정한다.
원본 bag·Capture는 복사하지 않았다. benchmark용 5개 raw XYZI 추출물만 새로 생성했다.

| 입력 | 검사 범위 | 한계 |
|---|---|---|
| 명시적 합성 11 snapshot | 독립 NumPy 정답 vs C++ 중심/코너/속도, pose 부재·미래 도착·100ms 초과 | 실제 차량 운동 정확도 아님 |
| `stationary_v2/stationary_0.bag` 2,562 cloud | 기록된 도착 순서·수신 stamp 정수 ns 보존 | 당시 내부 leveling/pose snapshot 없음 |
| 정차 5 scene의 첫 raw cloud | sensor→base XYZI 변환·역변환·성능 | 5개 전체 raw scan 표본, 2,562개 전부 변환 계측 아님 |
| admitted Capture stage 100개 | 이미 ROI 처리된 단일-frame XYZI 보존·변환 | 원본 full cloud 아님. 이전 10 Hz 자료, 시각은 fixture용 1. 연속 시간/속도 평가 불가 |

bag SHA는 `c4ce3c2afe53167c38dba3f284fe166ef5ebc6cb755581815ea1200eb6c8b076`,
v3 센서 profile SHA는 `4c2e93cc4d1139caf92a4e7ed4e20b9c7a705599e7e3faaf247b596e0fd1ae9f`다.
실제 bag에는 rear-axle pose를 추정해 끼워 넣지 않았다. 최근접 IMU를 사용해 당시 leveling을
“정확히 복원”했다는 주장도 하지 않는다. 따라서 실자료 map 변환·velocity 정확도는 미검증이다.

## 전후 비교와 수락 결과

| 같은 합성 관측 | 기존 adapter map 중심 오차 | 기존 코너 최대 오차 | 후보 |
|---|---:|---:|---|
| pitch-only | `9.32e-8 m` | `1.07e-7 m` | double 정답과 일치 |
| roll=0.12, pitch=-0.07, yaw=0.61 rad | `0.249554 m` | `0.386989 m` | 실제 leveling 전체 역변환으로 통과 |
| 위 자세 + 비영점 mount 회전 | `0.249554 m` | `0.386989 m` | mount까지 합성해 통과 |

후보 전체 합성 최대 성분 오차는 `3.5527e-15`, cloud 역변환 최대 거리 오차는
`3.5597e-15 m`로 `1e-6` 기준 이하다. legacy local을 그대로 base라고 해석하면 이 fixture들에서
중심 차이가 `1.191~1.897 m`다. 이는 서로 다른 원점·축을 동일시했을 때의 차이이며
실주행 전 구간의 검출 위치 오차라는 뜻은 아니다.

- **C++ 34개 통과:** 비영점 RPY/mount, pitch-only/roll-pitch 구별, GPS lever arm 1회,
  중심/코너/벡터 역변환, 형상·속성 보존, 수직 방향 yaw 없음, proper rotation 검증,
  invalid/NaN pose, local 보존, clock/gap/중복/역행/미래 도착, twist 의미/시간 검사.
- **JSONL 9개 통과:** 독립 수학 정답·재실행 일치, 큰 int ns 보존, 영점/중복/역행 사유,
  잘못된 pose 시각·NaN과 local 분리, calibration 누락, twist 누락, 미래 도착 배제.
- **기존 평가 도구 24개 통과**, L1-A source/MD5/실자료 hash 기준 검사 통과.
- 객체 ID·개수·class와 point count/intensity 보존. 미래 도착 pose 사용 0건.
  timing을 제외한 같은 입력 결과가 동일하며 canonical SHA는 manifest에 고정했다.

검사 중 잘못된 회전행렬을 `Eigen::rotation()`이 정규화해 숨길 수 있어 **저장된 linear
행렬 자체**를 검사하도록 바로잡았다. 이는 이번 새 코어 내부 수정이며 운영 변경이 아니다.

## 비용 — 변환과 copy만, end-to-end/deskew 아님

i5-14600K, WSL2 x86_64 little-endian, `asmc-ros-noetic`, GCC 9.4.0, CMake Release.
warmup 3회 후 cloud 31회, object 100회, steady_clock nearest-rank p50/p95/p99.
메모리 할당·copy 포함, 파일/ROS decode·publish·센서 지연 제외. 해제/실행 환경 지터의
전체 시스템 비용을 이 표로 대체하지 않는다. 아래는 **한 실행**의 수치다.

| raw scene / 점 수 | copy p50/p95/p99 ms | copy+transform p50/p95/p99 ms |
|---|---|---|
| initial / 9,326 | .0348 / .0474 / .0718 | .0666 / .0849 / .0977 |
| highway / 9,286 | .0326 / .0480 / .0518 | .0626 / .0925 / .0969 |
| shadow entry / 18,424 | .0785 / .1185 / .1344 | .1197 / .1751 / .1949 |
| shadow middle / 18,756 | .0780 / .1139 / .1232 | .1194 / .1747 / .1835 |
| recovered / 9,315 | .0078 / .0090 / .0224 | .0248 / .0274 / .0313 |

Capture ROI 100개의 6,571~24,878점에서 각 표본 변환 p50 범위는 `.0112~.2253 ms`다.
이 범위를 통합 p95처럼 사용하지 않는다. 표본별 percentile은 report에 전부 보존했다.
객체 1개(중심+8코너+방향+속도), roll/pitch/mount 합성은 `.001192/.001216/.001258 ms`,
copy는 `.000033/.000034/.000035 ms`다. 다객체 선형 성능 보장·원본 raw deskew 비용이 아니다.

## 문서·참고 구현 검수

상세 문서의 기존 설계 논의를 남기고 [13.1 정정표](./lidar-assessment-and-roadmap.md#131-l1-b1-재검수-기록--원-주장과-정정-이유-보존)에
기존 주장·이유·변경 위치를 기록했다. 주요 정정은 타 환경 FPS로 채택/배제, point 수 비례
시간/VRAM 환산, map tracking의 속도0 보장, 항상 보수적인 관측 지평, 모든 CenterPoint의
두 map-view 필수, 정적/UNKNOWN 속도0 강제다.

기존 LiDAR/fusion 후보 **24개**는 공식 origin·SHA·license와 주요 입출력 코드 경로를
대조했다. 목록/부분 코드 확인은 빌드·실험과 다르다. Patchwork++ standalone의 이전 100장
비교 외에 후보를 새로 빌드하거나 성능 검증했다고 표시하지 않았다.
`$ASMC_DATA/lidar_assessment/reference_audit_20260927.json`과 로컬 workspace
`docs/papers/lidar-reference-readiness-20260926.md` 7절에 전체 검수표가 있다.

새 참고는 Autoware distortion corrector/LIO-SAM **2개만** sparse checkout했다.
SHA는 manifest에 고정했으며 ROS2·SLAM 의존성/weight는 추가하지 않았다.
기존 external의 수정/학습 자산도 그대로다. 다음 지면 비교 우선순위는 linefit → 기존
Patchwork++ 동일 조건 재검증이다. 이것은 다음 단위 GPS/시간 검토를 건너뛰라는 뜻이 아니다.

## 인계·중단선

이번에 해결한 것은 **격리 후보에서**의 좌표 합성·시각 보존·가용성 분리와 동일 입력 검증이다.
기존 runtime roll 누락·frame 표기·GPS 상태 문제를 운영에서 고친 것은 아니다.

사용자 검토 후 다음 한 단위에서 GPS valid/invalid/stale/NaN/복구 jump와 추적 상태/출력
projection의 분리를 설계·시험한다. 실제 관측 시점 pose/twist 이력, 적용 leveling snapshot을
producer에서 어떻게 전달할지, Behavior/Frenet/MPC의 차량 상태/trajectory와 어떻게 함께
전환할지 합의한다. local/map topic을 순간적으로 다른 frame으로 바꾸지 않는다.

카메라 50ms 경계·지연·누락 터널 문제, ground/ROI/cluster/tracker, 벽 근접 차량·낮은
장애물·횡단·고주로·연속 주행은 아직 해결/수락하지 않았다. 원점·시간이 맞는 것만으로
탐지 정확도가 검증된 것은 아니다.

Rollback 경계는 단순하다. 운영에는 미연결이므로 평가 도구 실행을 멈추면 기존 동작은
그대로다. 향후 연결은 별도 opt-in/호환 adapter와 동일 데이터 A/B, 담당자 검토 후만 한다.
이번 단위 종료 후 자동 runtime 교체·commit/push·PR은 하지 않는다.
