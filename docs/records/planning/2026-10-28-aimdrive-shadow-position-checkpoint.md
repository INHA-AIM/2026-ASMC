# 음영구간 위치 추종: 무장애 실제 주행 체크포인트

2026-10-28 KST. `research/shadow-aimdrive-localization-20261028`에서
[이전 정적 회피 이식](2026-10-28-aimdrive-shadow-static-port.md)에 이어 진행했다.
**5km/h의 짧은 확인과 10km/h의 터널 진입·탈출을 시험했다. 중단 시점에 15km/h 시험은
아직 시작하지 않았다.** 다른 두 worktree의 파일·index·기본 overlay는 변경하지 않았다.

## 코드와 실행 위치

```text
2026-ASMC-aimdrive/
  src/asmc_planner/src/shadow_map/
    anchor.hpp                 GPS-minus-odometry anchor와 DR hold
    sampled_motion.hpp         source 시각의 상태·자세 보간, 재적분 없음
    position_node.cpp          raw GPS/IMU·공유 LiDAR를 위치 추종 경계에 연결
    position_reference.cpp     고정 지도 참조 샘플링·map/OBB 검증
    static_planner.{hpp,cpp}   기존 정적 회피 core 연결, 후속 장애물 단계
  src/asmc_planner/src/local/
    node.cpp                   odometry_only 모드: 적분기 하나만 실행
    planner.cpp                공용 차체 검사·속도 시간화
  src/asmc_controller/src/local/node.cpp
                               기존 MPC 하나, LocalTrajectory 계약 유지
  src/asmc_planner/launch/shadow_map_position.launch
  tools/scen_runner/
    run_local_trial.py          기존 배치·제어 소유권·복구를 재사용
    analyze_shadow_position.py native 위치로 독립 평가
    prepare_shadow_position_runtime.py
                               다른 overlay를 덮어쓰지 않는 실행 번들 준비
  .runtime/position-b1,b2/      Git 제외된 코드·실행 파일 번들
```

지도는 기존 Frenet core가 읽은 같은 reference와 HD map을 사용한다. read-only
`ReferencePoseAt` 조회를 추가했고, 무장애 기준선에서는 lateral/lattice 후보 탐색을
호출하지 않는다. 차체의 지도 포함과 현재 local OBB는 계속 검사한다. 이 단계는
map 도로 형상을 사용하는 명시적 무장애 시험이며 semantic 자유공간 관측의 수락이 아니다.

bridge·카메라·LiDAR는 기존 실행을 공유했다. 필요하면 기존 상대 odometry를 재사용하고,
없으면 `odometry_only=true` 발행자 하나를 시작한다. 위치 참조 node는 상태를 보간할 뿐
속도/IMU를 다시 적분하지 않는다. raw IMU는 GPS anchor의 방향 표본으로만 읽고,
GPS fix마다 한 번 공통 UTM52N 변환을 수행한다. 경로 시간화와 MPC도 각각 한 번이다.
기존 bridge에 이미 있던 legacy GPS/IMU 발행자는 다른 작업을 위해 그대로 유지했다.

실행 파일은 별도 worktree에서 빌드했다. 초기에는 전용 host-network 컨테이너를 썼고,
이후 기존 ROS 네트워크 namespace 안의 `/tmp/asmc-aimdrive-position-20261028-b2`로
개인 번들을 옮겨 **그 번들의 자식 프로세스만** 실행했다. 기존 `/root/ws`의 source,
devel, 모델, sensor/network preset을 교체하지 않았다. 번들은 라이브러리 검색 경로만
해당 자식 환경에서 우선한다. 맵·waypoint 바이트는 기존 실행본과 SHA가 같다.

## 발견하고 고친 연결 문제

1. 최초 r01은 제어 라우터 실행 파일 누락으로 주행 전에 종료했다. 필수 실행 파일 검사를
   시뮬레이터 변경 전에 수행하도록 추가했다. 복구 오류는 없었다.
2. r02의 legacy `/vehicle/state`는 ENU yaw를 그대로 유지했고 sensor lever 보정도 이
   실험 계약과 달랐다. GPS 소실 뒤 횡오차가 늘었다. 정상 GPS 때라도 raw IMU/ActorState
   yaw가 서로 같다는 사실만으로 UTM grid yaw라고 볼 수 없었다.
3. 실험 node가 raw GPS/IMU에 기존 `asmc_geometry`의 ENU→UTM grid 변환을 적용하고,
   canonical GPS 장착 x=0.35m를 제거하도록 수정했다. 외부 GT로 yaw/scale/offset을
   맞추지 않았다. 실제 ActorState는 배치·접촉 감시·독립 평가에만 사용한다.
4. MPC의 계산 주기는 일정해도 작은 제어 토픽의 bag 수신에는 긴 공백이 있었다.
   지연 민감 subscriber에 TCP_NODELAY를 적용하고 같은 ROS namespace에서 실행했다.
   별도 수신 관찰로 실제 제어 수신과 bag 기록 누락을 구분했다.
5. 고대역 센서 recorder와 제어·상태 recorder를 분리했다. 각 토픽은 한 번만 녹화한다.
   r06에서 실제 수신 1069개와 같은 활동 구간의 bag 제어 1069개가 일치했고, recorder
   그룹 간 중복 토픽과 `.active` 파일이 없었다.

LiDAR의 GPS 판정, 벽/천장 처리와 canonical map 객체 발행은 수정하지 않았다.
추정 map pose를 LiDAR에 넣어 음영 전처리가 꺼지는 경로도 만들지 않았다.

## 실제 시험 결과

시뮬레이터는 SUNNY 13시, 생성 장애물 없음으로 시험했다. 기존 배경 actor는 보존했고
native 접촉을 감시했다. 각 run은 배치·날씨·시간·제어 모드·ego를 복구했다.
자료는 D `artifacts/aimdrive-position-20261028-r01`부터 `r06`까지 보존한다.
입력·설정·dirty source patch는 D `manifests/aimdrive-position-preparation/<run-id>`다.

| Run | 목표/이동 | native 고정 참조 횡오차 p95 | 실제 수신 최대 공백 | 제어 bag 최대 공백 | 판단 |
|---|---|---:|---:|---:|---|
| r01 | 5km/h / 0m | — | — | — | 실행 준비 실패, 복구 완료 |
| r02 | 5km/h / 35.05m | 약 40.8cm | 독립 수신 미계측 | 260.8ms, 종료 구간 포함 | 좌표계 연결 실패 근거 |
| r03 | 5km/h / 35.00m | 7.9cm | 독립 수신 미계측 | 활동 구간 68.5ms | 좌표 변환 후 저속 확인 |
| r04 | 10km/h / 145.05m | 6.4cm | 독립 수신 미계측 | 250.03ms | 위치 추종 양호, 기록 경로 추가 점검 |
| r05 | 10km/h / 145.28m | 4.9cm | 69.5ms | 299.7ms | 제어 수신 양호, recorder 누락 구분 |
| r06 | 10km/h / 145.18m | 12.2cm | 69.1ms | 69.1ms | 위치·제어·제어 기록 단계 검사 통과 |

r03 이후의 오차는 native 위치를 **고정된 실제 planner guide**와 비교한 값이다.
raw 팀 경로와의 차이도 따로 기록한다. 추정 위치에서 매번 기준선을 재설정하여 오차를
없애지 않는다. ActorState ENU heading도 공통 변환으로 grid에 맞춰 평가했다.
ROS header/receive clock과 native RPC 관찰을 정렬한 평가이며 acquisition clock을
독립적으로 교정한 측정이라고 주장하지 않는다. r02 원래 평가 파일은 그대로 보존하고
고정 참조에 대한 별도 진단을 추가했다.

r06은 `GPS 정상 → DR → GPS 복구` 순서를 확인했다. 이동 중 1005회 중 1002회(99.7%)가
유효했고, 나머지 3회는 무효 판단을 유지했다.
위치 참조 계산 p95 약 2.08ms, MPC p95 약 12.98ms였다. 새 참조 node의 추가 적분·추론·
lattice 호출은 0, 속도 프로필은 한 번이다. 접촉과 복구 오류는 모든 주행 run에서 0이다.
이는 기록된 SUNNY 무장애 조건의 결과이며 인지 오탐·정적 회피·동적 회피·다른 환경의
수락으로 확대하지 않는다. legacy `trial-result.closed_loop_acceptance=false`는 전체
자율주행 수락과 구분하기 위해 유지하고 단계 판정은 `position-assessment.json`에 둔다.

목표 5km/h의 r03 실제 최대속도는 출발 과도 응답에서 6.48km/h, 목표 10km/h의 r06은
11.11km/h였다. 목표 상한을 엄수하는 종방향 제어의 수락은 아니며 다음 속도 단계 전에
과도 응답도 함께 확인해야 한다. 15km/h 실제 주행은 아직 검증하지 않았다.

체크포인트에서 공통 좌표·시각 계약, LOCAL-1 core, 15km/h에서 이상적인 추종을 가정한 좌우 정적
회피 core, controller 경계·소유권 회귀시험을 다시 통과했다. 문서 migration·runtime 검사와
Python 구문 검사도 통과했다. 정적 회피 core 시험은 실제 15km/h 주행의 대체 검증이 아니다.

## 이어서 할 일

1. 공유 bridge·인지 상태와 소유권을 다시 확인한 뒤 같은 구성에서 15km/h 진입·탈출을 반복한다.
2. 횡오차 p95≤25cm·최대≤50cm, 방향 p95≤3°, 활동 중 제어 공백≤250ms,
   fresh 판단 95% 이상·접촉0·복구0을 확인한다. 출발 과속과 GPS 복구의 참조 연속성도 본다.
3. 그 기준선을 고정한 다음 기존 정적 회피 core의 단일 장애물 통과·본차로 복귀를 실제로 시험한다.
   이번 위치 참조 기준선을 정적·동적 장애물이 있는 운영 모드로 바로 승격하지 않는다.

중단 확인 시 시험용 제어·라우터·recorder는 실행 중이지 않았고, 여섯 run 모두 복구 오류가
없었다. 새 주행을 자동으로 재개하지 않은 상태에서 이 체크포인트를 남긴다.
