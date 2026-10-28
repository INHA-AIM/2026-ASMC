# lane_family 계산 지연 개선 — 2026-10-28

상태: **격리 작업트리 구현·CPU 재현·MORAI 터널 실행 및 사용자 전 구간 수동 확인 완료. PR 통합 검증 중.**
기준은 PR #127 통합 커밋 `d518b7c9fdf9af57c6a3dabf799802829bb2b703`이며,
작업 브랜치는 `perf/lane-family-latency-20261028`이다.
재현은 기준 revision과 각 run의 소스 snapshot/패치를 함께 사용한다.

후속 결정: 성능 개선은 현재 검증본에서 마감한다. 10월28일 사용자가 전 구간 실행에 문제가
없음을 확인하고 최신 main 정렬·PR 및 머지 후 전용 작업트리/컨테이너 정리를 요청했다.
다른 시뮬 시험의 실행/recording·공용 endpoint lease는 그대로 보존한다.

## 문제와 변경

기존 매칭은 같은 프레임의 차선 node를 연결하면서 작은 NumPy 배열·평균·내적을 반복 생성했다.
실제 저장 입력에서 이 단계가 약10ms를 사용했다. 다음 네 변경을 순차로 적용하고 각각 비교했다.

1. **차선 node 연결:** Python ROS 노드는 유지하고 C++/pybind11 함수 하나로 프레임당 한 번 호출한다.
   최근 여섯 점의 예측 통계를 track 변경 때 갱신하고, 후보 계산에서는 최근 다섯 점의 배열을 재사용한다.
   입력은 소유권이 있는 float64/int64 배열로 포장하고 결과는 원본 node index로 돌려받는다.
   Python 객체를 접근하지 않는 구간에서만 GIL을 해제한다. 포장·복원 비용도 측정에 포함했다.
2. **주행 활용성 진단:** support 양 끝점 판정을 배열로 처리하고, 소형 interval의 유한값 검사를
   scalar 연산으로 처리한다. 매 프레임 진단을 유지하며 OBSERVED/STRUCTURE_INFERRED/INVALID와
   겹침·역방향·내부 공백 규칙을 보존한다.
3. **카메라 근거 추출:** 고정 투영 영역의 pixel index를 미리 계산하고 uint8 class/confidence를
   먼저 선택한다. 256개 confidence의 원래 float32 정규화·임계값 판정을 lookup으로 재사용한다.
   float confidence 입력은 기존 전체 영상의 유한값·범위 검사를 유지한다. projector의 고정 valid
   배열은 읽기 전용이다. 캘리브레이션/ROI 변경 때는 projector를 새로 구성한다.
4. **공동 피팅:** joint solver가 곧 덮어쓰는 alternating 폭 추정과 임시 목표점 계산을 제거했다.
   joint solver, 반복 횟수, 강건 가중치, 폭 제한, 황색 부재 시 fallback은 그대로다.

native 계산에서 gate 또는 후보 residual의 동률 근처를 만나면 프레임 전체를 기존 Python으로
계산한다. float64 외 node 좌표도 원래 연산으로 처리한다. 추정 오차 허용으로 결과 차이를 덮지 않았다.

## 최종 동일 입력 비교

10월18일 가감속·조향 수집의 **226개 3카메라 묶음 × M1/M0 × 3회 = 1,356대조**다.
같은 프레임에서 reference/candidate 순서를 번갈아 실행했고, 8회 warm-up 후 profiler 없이 측정했다.
cProfile은 별도 pass로 수행했다. 1,356개는 독립 장면 수가 아니다.

| 구성 | 계산 p50 | 계산 p95 | 계산 p99 |
|---|---:|---:|---:|
| 기존 M1, 원본 시각 IMU 보정 | 43.13ms | 56.41ms | 63.00ms |
| 개선 M1 | **27.70ms** | **38.23ms** | **45.60ms** |
| 기존 M0, 고정 평면 복귀 | 36.21ms | 50.42ms | 58.02ms |
| 개선 M0 | **22.58ms** | **30.51ms** | **36.11ms** |

M1 p50/p95는 **35.8%/32.2%**, M0는 **37.7%/39.5%** 감소했다.
개선 M1이 더 빠른 paired sample은678개 중661개였다. 호스트 부하에 따른 개별 역전도 남아 있다.

| 최종 M1 단계 | 기존 p50 | 개선 p50 |
|---|---:|---:|
| 차선 연결 | 10.42ms | **0.34ms** |
| 3뷰 근거 추출 | 6.53ms | **4.28ms** |
| 활용성 진단 | 2.39ms | **0.60ms** |
| shared family 피팅 | 13.82ms | **12.85ms** |

환경: i5-14600K, WSL2, ROS Noetic/Python3.8.10/NumPy1.24.4,
격리 컨테이너 CPU quota2·BLAS thread1. 다른 호스트 작업을 중지하지 않았다.
관측된 GPU는 RTX4060Ti8GB였으나 실험 컨테이너에는 GPU 접근을 주지 않았다.
**캐시된 추론 출력 이후 계산 시간**이며, 영상 모델 추론·IMU 실제 대기·ROS 전달·판제 큐와
입력→소비자 지연, 실제 주행 FPS를 측정한 값은 아니다. 가중치·engine·해상도는 변경하지 않았다.

## 일관성과 실행 검증

- 1,356/1,356에서 evidence 전 필드, node 전 필드, track 소속/순서, 최종 family 전체 dataclass와
  진단값의 직렬화 hash가 같았다. 제외한 값은 wall-clock `fit_ms`뿐이다.
- 286개 CPU 회귀 시험: 실패0, 오류0, skip0. 실제 catkin에서 빌드한 extension을 로드했다.
- native 추가 시험: 무작위 곡선·가림·교차300장면, gate 경계, 동률, 반복x, dtype/shape,
  extension 부재 시 fallback. uint8 projection은 256개 confidence 값과 threshold 경계를 대조했다.
- 진단 endpoint 상태729조합, 역방향·겹침·0길이·비유한 구간을 확인했다.
- 기존 IMU source-time/복귀, 최신 입력 선택, 세대 변경 후 결과 폐기, readiness·직렬화 시험을 통과했다.
- 네트워크가 격리된 ROS master에서 native/python 각각 실제 노드를 생성하고 진단 발행·종료를 확인했다.
  운영 master나 차량 제어에는 연결하지 않았다.
- 최종 별도 profile452회에서는 native 호출452회, 수치 경계에 따른 Python 재계산0회였다.
  다른 장면의 fallback 빈도를 보장하는 수치는 아니다.

이는 **기존 매칭·피팅 결과를 보존하면서 지연을 줄인 검증**이다. 독립 GT 정확도 개선이나
MORAI 전체 실행에서의 공급 안정성 개선까지 입증한 것은 아니다.

## 파일과 실행 선택

- [native 연결 함수](../../../src/asmc_camera/native/lane_association.cpp),
  [Python adapter와 기준선](../../../src/asmc_camera/src/asmc_camera/bev/lane_tracking.py)
- [구간 진단](../../../src/asmc_camera/src/asmc_camera/bev/planning_usability.py),
  [근거 추출](../../../src/asmc_camera/src/asmc_camera/bev/model_grid_projection.py),
  [공동 피팅](../../../src/asmc_camera/src/asmc_camera/bev/shared_lane_family.py)
- [노드](../../../src/asmc_camera/src/asmc_camera/nodes/lane_vectorizer_node.py),
  [설정](../../../src/asmc_camera/config/bev/lane_fitting.yaml),
  [대조 도구](../../../tools/perception/benchmark_lane_family_compute.py)

`association_backend: auto`는 extension이 있으면 native를 사용하고 없으면 Python을 사용한다.
`python`은 명시적 복귀이며, `native`는 extension이 없을 때 시작을 거부한다.
진단 `association_backend`에서 요청/가용/선택 상태를 확인할 수 있다.
runtime profile1.3.0에 `planning_usability` 단계가 추가됐다. 운영 토픽·메시지 형식은 변경하지 않았다.

빌드 의존성은 `python3-dev`, `pybind11-dev`다. catkin은 `ASMC_CAMERA_NATIVE_ASSOCIATION=ON`으로
설정하며, headers가 없으면 Python 사용을 명시한다. 격리 검증에서는 기존 Noetic image에 포함된
headers를 `ASMC_PYBIND11_INCLUDE_DIR=/usr/local/lib/python3.8/dist-packages/torch/include`로 지정했다.
배포 대상 Python과 같은 ABI로 빌드해야 한다. `.so`는 Git에 추가하지 않았다.

## 재현 근거와 단계별 기록

워크스페이스 저장 규약으로 등록한 D의 `artifacts/<run-id>/`에 입력 SHA 목록, 설정/소스 snapshot,
패치, 원본 단계 시간, 별도 pstats를 보관했다. 각 단계에서 원래 Python 기준선과 비교했다.

| run ID | 목적/결과 |
|---|---|
| `lane-family-baseline-20261028-r1` | harness 설정 키 오류, 시간 표본 없음. 실패 기록 보존 |
| `lane-family-baseline-20261028-r2` | 동일 기준선904/904 일치 |
| `lane-family-native-20261028-r1` | native 연결,452/452 일치 |
| `lane-family-intervals-20261028-r1` | 위 변경+진단,452/452 일치 |
| `lane-family-projection-20261028-r1` | 위 변경+선택 추출,452/452 일치 |
| `lane-family-joint-path-20261028-r1` | 위 변경+중복 joint 경로 제거,452/452 일치 |
| `lane-family-final-20261028-r1` | 최종 native guard/backend/계측 포함,1,356/1,356 일치 |

최종 run의 `report.json`, `frames.jsonl`, `comparison-summary.json`, `regression.log`,
`test_final-result.json`, `isolated-node.log`, `smoke_node-result.json`이 수치와 시험의 정본이다.
`execution-config.configs`는 상속한 기준선 hash이며 실제 후보 설정은 `source_sha256`,
소스 snapshot, `report.config_sha256`에 고정했다. `provenance-clarification.json`에 구분을 남겼다.
첫 catkin 시험에서 발견한 module 출력 위치 문제는 package destination 정의 뒤 빌드하도록 수정했고,
수정 후 실제 devel 환경 import와 전체 시험을 다시 확인했다.

## 운영 반영 전 남은 확인

1. 진행 중인 세 작업과 통합 기준을 맞춘 뒤 전용 build/devel 경로에서 적용한다. 현재 다른 checkout의
   코드·index·브랜치·운영 컨테이너 마운트와 실행 노드를 변경하지 않았다.
2. 기존3뷰·YOLO·TwinLite·신호등·LiDAR·판제를 함께 켠 조건에서 같은 bag/시나리오를 비교한다.
   source→consumer p50/p95/p99, 유효 신규 family Hz·최대 공백, readiness 하락·CPU/GPU를 함께 기록한다.
3. 기존15Hz 목표 및 수락 기준을 유지하고 터널 진입·가림·조향·IMU M0 복귀·STANDBY/세대 전환을 확인한다.
   오래된 family 재발행이나 freshness350ms·IMU wait10ms 확대는 하지 않는다.
4. native 문제는 `association_backend: python`으로 분리한다. 전체 변경의 문제가 확인되면 보존한
   기존 배포본으로 복귀한다. 실행 중인 다른 작업을 자동 재시작하거나 되돌리지 않는다.

## 후속 MORAI 터널 실행 확인

사용자 요청으로 지정된 `camera.launch`, `lidar.launch`, `autonomy_no_bridges_v2.launch`를
인자 변경 없이 별도 작업트리 컨테이너에서 실행했다. 기존 브릿지는 유지했다.
MORAI 메시지 submodule을 pinned `45c6baf…`로 초기화하고 whitelist 없는 전체 Release 빌드를 완료했다.

첫 실행에서 계산 backend 선택값이 geometry config hash에 포함되어 판제가
`wrong_geometry_config_hash`로 결과를 거부하는 연결 결함을 발견했다. 두 geometry contract의
track 설정에서 실행 backend만 분리했다. 실제 거리·각도·지원 구간 임계값과 판제 hash 검사는 유지한다.
backend는 진단과 소스/실행 기록에서 계속 식별한다.
격리 ROS master에서 실제 노드 생성으로 python/native/auto가 기존 family hash
`2d17cf63adbf2ba049d70656233ee2f34e9a4bfb43373ee76b3371aab1e4dc2a`를 유지하고,
실제 lateral gate 변경은 family/boundary hash를 바꾸는 것을 확인했다.

| run | 결과 |
|---|---|
| `lane-family-tunnel-20261028-r1` | 출발 전 hash 불일치 검출. 명시적 주행 release 없음; 준비 단계 물리 이동은 미계측. 정리·원위치 복원 완료 |
| `lane-family-tunnel-20261028-r2` | 수정본 터널 도달, 측정128.37m/25.79초. 준비 중6.24m 이동이 있어 정지 시작 조건 재시험 |
| `lane-family-tunnel-20261028-r3` | native pause로 준비 상태 고정 및 출발 직전 위치/속도 확인. **134.45m/25.71초, 터널 종료 조건 도달** |

최종 r3은 예정 시작점과0.0004m 차이,0km/h에서 release했다. 약137m 경로의 기존
종료 판정(종점3m 이내 전진 접근 또는 종료선 통과)을 사용했다. 최고속도22.57km/h,
새 native contact0, 소유 노드 비정상 종료0, 주행 구간 rosout ERROR/FATAL0,
세 카메라 모델의 추론 error0이었다. 객체·날씨를 새로 배치하지 않은 기존 장면의 한 회차다.

주행 중 family383개는 모두 서로 다른 source stamp이며 모두 valid였다. 수신14.89Hz,
최대 수신 공백141.49ms, source header→시험 관측자 수신 p50/p95는112.54/157.80ms다.
이는 물리 센서 clock 보정 지연이나 판제 소비 지연이 아니며, 과거 CPU paired 계산 시간과
직접 합산하지 않는다. 마지막 rolling 진단의 계산 p50/p95는30.23/45.36ms,
native backend 선택과 geometry hash 일치를 확인했다. 실제 GPU는4060Ti8GB였다.

**무경고·전 구간 수락을 뜻하지 않는다.** r3에서 판제의 family segment width ODD 조건으로
CREEP 전환 경고1회가 있었다. r2에는 `projection_source_expired` 경고1회도 있었다.
이번 최적화로 생긴 현상인지 기준선에도 있는지는 분리하지 않았고, 임계값을 완화하지 않았다.
GPS 음영 중 일반 map/MPC 경로는 invalid 경고를 내며 JAMMING 제어가 선택돼 주행했다.
별도 BEV fusion은 입력/출력0 및 대기 진단을 보였으며, 최종 출력 구독을 요구하는 수요 기반
계산 구조다. 이번 시험은 해당 BEV 출력의 수락 검증을 포함하지 않는다.

실험 결과의 `trial-result.json`, `verified-summary.json`, `events.jsonl`, `tunnel.bag`,
launch별 로그와 source snapshot을 보존했다. 시험 종료 후 소유 launch/recording을 종료하고
시험 전 ego 위치·control mode를 복원한 뒤 P·정지 상태로 남겼다. 공용 lease를 반납했고
기존 브릿지는 유지했다. 이후 사용자가 전 구간 수동 실행에 문제가 없음을 보고했다.
이는 사용자 확인 기록이며 r3의 자동 계측 범위를 전 구간으로 확대하는 근거는 아니다.

## 최신 main 통합 범위

통합 대상 `562c99f9`는 PR #129의 E3 객체 검출기·3뷰 LiDAR fusion 변경을 포함한다.
이 최적화가 수정하는 lane 계산 파일과 겹치지 않으며 해당 main 변경을 유지한다.
위 CPU 대조·터널 계측·사용자 수동 확인은 PR #127 기준 작업트리에서 수행했다.
E3 기본 모델과 결합한 전 구간 실측으로 표현하지 않는다.

CI에는 `python3-dev`/`pybind11-dev`와 빌드 결과 import 검사를 추가해 native 시험이
미빌드로 생략되는 것을 막는다. 격리 ROS master에서 실제 lane 노드의 python/native/auto
geometry hash가 판제 기대값과 같고, 실제 lateral gate 변경은 hash를 바꾸는 회귀도 실행한다.
추론 모델·운영 토픽·판제 수락 임계값에는 이 PR의 변경이 없다.
