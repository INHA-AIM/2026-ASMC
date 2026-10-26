# Bridge GPS0 유효성·복귀 수정

2026-10-26, PR #115. [C0 수정 전 진단](../lidar/2026-10-26-frame-time-baseline.md) 이후
사용자가 같은 PR의 결함 수정을 승인했다. 대조 도구와 수정 전 실패 근거는 보존한다.

## 바뀐 동작

MORAI 음영구간은 `status=1`인 채 위도·경도 `(0,0)`을 제공할 수 있다. 기존 bridge는 이를
유효 GPS로 투영해 `/vehicle/state`에 약 x=−42,025,237m, y=−4,124,145m를 valid로 제공했다.
판제와 LiDAR는 이미 이 sentinel을 거부했다.

- `ValidGps`가 `(0,0)`을 거부한다. 무효 상태를 발행하며 해당 위치를 투영하지 않는다.
- GPS status·좌표·offset·stamp가 무효이면 위치 차분 창, 속도, 정지 기준, 가속도 필터를 초기화한다.
- GPS 수신 timeout(설정0.50초) 뒤 첫 수신과 수신 시각의 중복/역행도 이전 속도 기준을 재사용하지 않는다.
- 새 GPS 표본 사이에 `minimum_baseline=0.12초` 이상이 확보될 때까지 `valid=false`다.
  5Hz GPS에서는 보통 두 번째 정상 fix로 기준을 얻지만 IMU freshness 등 기존 조건도 만족해야 한다.
- 무효 상태의 수치 placeholder0은 유효 위치·속도0이라는 뜻이 아니다. 소비자는 `valid`를 확인한다.

bridge의 안테나 기준 위치·ENU yaw·steady 수신 시각 기반 추정, 정상 구간 gain·최대 속도·freshness는 유지한다.
판제의 후륜축/grid yaw/source 시각 정책으로 바꾸지 않았다. UDP 패킷·네트워크 파일·조향 부호·센서셋도 변경하지 않았다.
현재 autonomy 정본은 별도 판제 GPS/IMU pose와 종방향 속도 경로를 사용하므로, 이 수정이 판제 추정기를 교체하는 것은 아니다.
LiDAR local/map·tracking·HD맵·카메라 모델·공용 메시지/MD5는 그대로다.

## 구현과 회귀

- [bridge 추정기](../../../src/asmc_bridge/src/vehicle_state/vehicle_state_adapter.cpp): sentinel와 baseline 초기화.
- [C++ 회귀](../../../src/asmc_bridge/test/vehicle_state_adapter_test.cpp): GPS0/status0/NaN/무효 stamp/범위 밖 좌표/무한 offset,
  무수신 timeout, 먼 위치 복귀, 반복 GPS0, 수신 중복·역행. 기존 정상 속도·가속·제동 검사도 유지.
- [ROS 발행 시험](../../../src/asmc_bridge/test/vehicle_state.test): 실제 `vehicle_state_node`를 독립 master에서 실행하고
  `/bridge_fixture/*` 센서만 넣어 정상→GPS0→첫 fix→복귀→미수신→첫 fix→복귀를 검사한다. 차량 제어는 발행하지 않는다.
- [native 코어 대조](../../../tools/perception/frame_time/replay_gps_recovery.py): 이전 live bag의 GPS/IMU 순서를 보존한다.
  기록된 VehicleState 수신 이벤트를 질의 시각으로 사용하며 원래 ROS callback 스케줄과 같다고 주장하지 않는다.

## 검증 결과

run: `lidar-frame-time-c0-gps-fix-20261026-r1`.
정본: `D:\morai dataset\shahn03\artifacts\lidar-frame-time-c0-gps-fix-20261026-r1`.
소스 `4afbd6ee`, 실행 시 dirty patch 없음.
[수정 후 v2 manifest](../../../config/baselines/lidar_frame_time_c0_v2.json)와
[소형 결과·산출물 SHA](../../../config/baselines/evidence/lidar_frame_time_c0_v2/results.json)를 따른다.

| 검사 | 수정 전 | 수정 후 |
|---|---:|---:|
| 실제 터널 기록 GPS0 구간 유효 질의 (전환100ms 제외) | 1,560 / 1,560 | **0 / 1,560** |
| 같은 기록의 nonzero GPS 유효 질의 | 793 | 788; 복귀 직후 새 기준 대기 |
| 합성75 scan의 GPS0 유효 scan, bridge | 9 | **0** |
| 합성75 scan의 GPS0 유효 scan, 판제 | 0 | 0 |
| 정상 두 bag의 bridge 상태 행 | 기준선 | **모두 동일** |
| 857 scan의 LiDAR·투영·카메라·support 수치 | 기준선 | **모두 동일** |
| bridge CTest | — | **9/9 통과**, 16.89초 |
| C0 합성 회귀 | — | **7/7 통과** |

실제 터널 입력으로 결함 제거를 재확인했고, 별도의 ROS 발행 시험에서 무효 중에도 상태 발행이 계속되며
복귀 첫 표본을 유효0속도로 오인하지 않는지 확인했다. 새로운 MORAI 주행은 추가하지 않았다.
기존 IMU stale26·원본 clock 대응 부족·속도 정확도 GT 부족은 이 수정으로 해결한 항목이 아니다.

## CI 수정과 실행 반영

이전 CI run `40323467515`는 C0의 `Rotation.as_matrix()` 호출이 Ubuntu20.04 기본 SciPy에 없어 실패했다.
시험 입력을 명시적인 비자명 회전행렬로 바꿔 버전 의존성을 제거했다. 회전 왕복·잘못된 회전 검사는 그대로 실행한다.
검사를 skip하거나 SciPy 전체 설치 단계를 추가하지 않았다. bridge CTest 최소 개수도 새 ROS 회귀를 포함한9개로 올렸다.

검증 빌드는 `/tmp/asmc-b4-build`, `/tmp/asmc-b4-devel`에서 수행했다. 시험 devel을 source하여
이전 운영 라이브러리를 잘못 읽지 않는지 `ldd`로 확인했다. 기존 사용자 노드와 운영 master는 재시작하지 않았다.
병합·일반 workspace 재빌드 후 다음 bridge 실행부터 적용된다. launch 명령·옵션 변경은 필요하지 않다.

## 재현과 다음 과제

v1 manifest는 수정 전 소스를 요구하므로 해당 고정 revision에서만 재현한다. 현재 checkout은 v2 manifest를 사용한다.
기존 run을 덮어쓰거나 과거 SHA를 최신 결과로 치환하지 않는다. 전체 도구 명령은
[frame_time README](../../../tools/perception/frame_time/README.md)에 있다.

다음 진단 후보는 LiDAR IMU stale의 수신 지연·표본 선택·scan 취득 구간 구분이다.
카메라 `morai_sensor`와 LiDAR `velodyne_device_hour`를 근거 없이 차감하거나 age 제한을 늘리지 않는다.
pose 공통화·ego motion·3뷰 matching은 해당 입력 근거가 정리된 뒤 별도 범위로 진행한다.
